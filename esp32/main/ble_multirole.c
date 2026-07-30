#include "ble_multirole.h"
#include "ble_rcube.h"
#include "rcube_cmd.h"
#include "rcube_buzzer.h"
#include "rcube_config.h"
#include "rcube_status.h"
#include "rcube_sensor.h"

#include <string.h>
#include <stdbool.h>

#include "esp_log.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"

#include "rcube_protocol.h"

static const char *TAG = "multi";

/* 광고 이름 접두어(이 접두어의 기기만 멤버 후보로 본다). */
#define RCUBE_NAME_PREFIX "RCUBE"
#define RCUBE_NAME_PREFIX_LEN 5

/* 한 유닛 최대 8대(기획서 [연결 가능 최대 수]) → 자신을 뺀 멤버 최대 7대.
 * NimBLE 동시 연결 수(CONFIG_BT_NIMBLE_MAX_CONNECTIONS)가 이보다 커야 한다
 * — 서브 허브는 PC 연결(peripheral) 1개를 추가로 쓰므로 8이 필요하다. */
#define MAX_MEMBERS (RCUBE_MAX_NODES - 1)

/* 멤버로 중계할 프레임 임시 버퍼(LED 등 소형 명령). */
#define FWD_BUF_MAX 96

typedef enum {
    SLOT_FREE = 0,
    SLOT_CONNECTING,   /* ble_gap_connect 진행 중 */
    SLOT_DISCOVERING,  /* 연결됨, 서비스/특성 탐색 중 */
    SLOT_READY,        /* RCBE 특성 확보 — 중계 가능 */
} slot_state_t;

typedef struct {
    slot_state_t state;
    ble_addr_t   addr;
    uint16_t     conn_handle;
    uint16_t     svc_start;
    uint16_t     svc_end;
    uint16_t     chr_val_handle;
    uint16_t     cccd_handle;  /* 0x2902 — 멤버 회신(notify) 구독용 */
    uint8_t      vid;          /* PC와 맞춘 가상 노드 ID(2..N) */
    uint8_t      nn;           /* 광고이름에서 읽은 노드ID(고정형 식별용) */
} member_t;

/* --- 상태 (모두 NimBLE 호스트 태스크 단일 스레드에서만 접근) --- */
static bool     s_active;
static uint8_t  s_target_members;   /* 연결해야 할 멤버 수 = link_count-1 */
static uint8_t  s_group_mode;
static bool     s_ordered;          /* 고정형: 광고 nodeID(NN)를 그대로 가상ID로 사용 */
static bool     s_edge;             /* edge central 모드: 멤버 맵의 BLE 노드만 연결 */
static uint8_t  s_next_vid;         /* 비고정형 시 다음 멤버 가상ID(2부터, 연결순서) */
static bool     s_connect_pending;  /* 연결 시도 1건 진행 중(직렬화) */
static member_t s_members[MAX_MEMBERS];

static int disc_gap_event(struct ble_gap_event *event, void *arg);
static int member_gap_event(struct ble_gap_event *event, void *arg);

/* ---- 헬퍼 ------------------------------------------------------------ */
static uint8_t count_ready(void)
{
    uint8_t n = 0;
    for (int i = 0; i < MAX_MEMBERS; i++) {
        if (s_members[i].state == SLOT_READY) {
            n++;
        }
    }
    return n;
}

/* 연결 진행 중 + 완료를 합한 "점유" 슬롯 수(중복 연결 방지·목표 판정용). */
static uint8_t count_busy(void)
{
    uint8_t n = 0;
    for (int i = 0; i < MAX_MEMBERS; i++) {
        if (s_members[i].state != SLOT_FREE) {
            n++;
        }
    }
    return n;
}

static member_t *alloc_slot(void)
{
    for (int i = 0; i < MAX_MEMBERS; i++) {
        if (s_members[i].state == SLOT_FREE) {
            return &s_members[i];
        }
    }
    return NULL;
}

static void free_slot(member_t *m)
{
    memset(m, 0, sizeof(*m));
    m->state = SLOT_FREE;
}

/* 광고이름 "RCUBEROBOT.GG.NN"에서 NN(마지막 '.' 뒤 10진)을 뽑는다. */
static bool parse_name_nn(const uint8_t *name, uint8_t len, uint8_t *out)
{
    if (name == NULL || len == 0) {
        return false;
    }
    int lastdot = -1;
    for (int i = 0; i < len; i++) {
        if (name[i] == '.') lastdot = i;
    }
    if (lastdot < 0) {
        return false;
    }
    int v = 0;
    bool any = false;
    for (int i = lastdot + 1; i < len; i++) {
        char c = (char)name[i];
        if (c < '0' || c > '9') break;
        v = v * 10 + (c - '0');
        any = true;
    }
    if (!any) {
        return false;
    }
    *out = (uint8_t)v;
    return true;
}

static bool addr_known(const ble_addr_t *a)
{
    for (int i = 0; i < MAX_MEMBERS; i++) {
        if (s_members[i].state != SLOT_FREE &&
            ble_addr_cmp(&s_members[i].addr, a) == 0) {
            return true;
        }
    }
    return false;
}

/* 더 붙일 멤버가 남았으면 스캔을 (재)개시한다. */
static void scan_if_needed(void)
{
    if (!s_active || s_connect_pending) {
        return;
    }
    if (count_busy() >= s_target_members) {
        return;   /* 목표 달성 — 스캔 불필요 */
    }
    struct ble_gap_disc_params dp = {0};
    dp.passive = 0;              /* active scan (스캔응답까지 수집) */
    dp.filter_duplicates = 1;
    uint8_t own_addr_type = ble_rcube_own_addr_type();
    int rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &dp, disc_gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "scan 시작 실패; rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "멤버 스캔 중… (%u/%u 연결)", count_ready(), s_target_members);
    }
}

/* ---- 공개 API -------------------------------------------------------- */
void ble_multirole_start_aggregator(uint8_t link_count, uint8_t group_mode, uint8_t flags)
{
    /* 기존 아그리게이터 상태가 있으면 먼저 정리. */
    ble_multirole_stop_aggregator();

    uint8_t members = (link_count > 0) ? (uint8_t)(link_count - 1) : 0;
    if (members > MAX_MEMBERS) {
        ESP_LOGW(TAG, "요청 멤버 %u > 최대 %u → %u로 제한", members, MAX_MEMBERS, MAX_MEMBERS);
        members = MAX_MEMBERS;
    }

    s_active = true;
    s_edge = false;            /* 서브로봇유닛의 BLE 허브 역할 */
    s_target_members = members;
    s_group_mode = group_mode;
    /* ★ 절차는 PC가 지시한다(확장 규격 §2.2). 허브 자기 노드ID로 판정하면, 이미
     * 노드ID가 저장된 큐브들로 비고정형 재구성을 할 수 없다 — 노드1 허브가 무조건
     * 고정형이 되어 멤버가 연결 순서 대신 저장 노드ID를 받아 버린다. */
    s_ordered = (flags & RCUBE_AGG_FLAG_ORDERED) != 0;
    s_next_vid = 2;            /* 허브=1(0xFE), 멤버는 2부터 */
    s_connect_pending = false;

    ESP_LOGI(TAG, "BLE 허브 시작: 목표 멤버 %u대(group_mode=0x%02x, flags=0x%02x, %s)",
             members, group_mode, flags,
             s_ordered ? "고정형=광고 NN을 가상ID로"
                       : "비고정형=연결 순서로 가상ID(저장 노드ID 무시)");

    if (members == 0) {
        rcube_cmd_report_members(0);   /* R1 상당 — 멤버 없음 */
        return;
    }
    scan_if_needed();
}

void ble_multirole_stop_aggregator(void)
{
    if (!s_active && count_busy() == 0) {
        return;
    }
    ESP_LOGI(TAG, "aggregator 해제: 멤버 전원 연결 종료");
    s_active = false;              /* 이후 콜백들의 재스캔/보고를 막는다 */
    s_connect_pending = false;

    if (ble_gap_disc_active()) {
        ble_gap_disc_cancel();
    }
    for (int i = 0; i < MAX_MEMBERS; i++) {
        member_t *m = &s_members[i];
        if (m->state == SLOT_DISCOVERING || m->state == SLOT_READY) {
            ble_gap_terminate(m->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        free_slot(m);
    }
    s_target_members = 0;
    s_next_vid = 2;
    s_ordered = false;
    s_edge = false;
}

bool ble_multirole_start_edge(void)
{
    ble_multirole_stop_aggregator();

    /* 멤버 맵에서 BLE 멤버(자신 제외)를 센다. */
    uint8_t me = rcube_config_node_id();
    uint8_t ble_members = 0;
    for (uint8_t nid = 1; nid <= RCUBE_MAX_NODES; nid++) {
        if (nid != me && rcube_config_member_cmf(nid) == RCUBE_MEMBER_BLE) {
            ble_members++;
        }
    }
    if (ble_members == 0) {
        ESP_LOGI(TAG, "edge central: BLE 멤버 없음 → BLE 서버 미기동(CAN 전용 유닛)");
        return false;
    }
    if (ble_members > MAX_MEMBERS) {
        ESP_LOGW(TAG, "BLE 멤버 %u > 최대 %u → 제한", ble_members, MAX_MEMBERS);
        ble_members = MAX_MEMBERS;
    }

    s_active = true;
    s_edge = true;
    s_ordered = true;          /* 독립유닛은 항상 고정형 — 광고 노드ID로 식별 */
    s_target_members = ble_members;
    s_group_mode = 0;
    s_next_vid = 2;
    s_connect_pending = false;

    ESP_LOGI(TAG, "edge central 시작: BLE 멤버 %u대 직접 연결(노드ID 매핑)", ble_members);
    scan_if_needed();
    return true;
}

int ble_multirole_forward(uint8_t target_id, const uint8_t *frame, uint16_t len)
{
    if (!s_active) {
        return -1;
    }
    if (len < 4 || len > FWD_BUF_MAX) {
        ESP_LOGW(TAG, "forward: 길이 %u 부적합", len);
        return -1;
    }
    member_t *m = NULL;
    for (int i = 0; i < MAX_MEMBERS; i++) {
        if (s_members[i].state == SLOT_READY && s_members[i].vid == target_id) {
            m = &s_members[i];
            break;
        }
    }
    if (m == NULL) {
        return -1;   /* 그런 가상ID 멤버 없음 */
    }
    /* 멤버는 자신을 허브로 아므로 target을 0xFE로 재기입해 '자기 명령'으로 처리시킨다. */
    uint8_t buf[FWD_BUF_MAX];
    memcpy(buf, frame, len);
    buf[0] = RCUBE_ADDR_HUB;

    int rc = ble_gattc_write_flat(m->conn_handle, m->chr_val_handle, buf, len, NULL, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "forward: 멤버 vid=%u write 실패 rc=%d", target_id, rc);
        return -1;
    }
    ESP_LOGI(TAG, "forward: vid=%u(conn=%u) ← %u bytes", target_id, m->conn_handle, len);
    return 0;
}

int ble_multirole_broadcast(const uint8_t *frame, uint16_t len)
{
    if (!s_active) {
        return -1;
    }
    if (len < 4 || len > FWD_BUF_MAX) {
        ESP_LOGW(TAG, "broadcast: 길이 %u 부적합", len);
        return -1;
    }
    uint8_t buf[FWD_BUF_MAX];
    memcpy(buf, frame, len);
    buf[0] = RCUBE_ADDR_HUB;   /* 멤버가 '자기 명령'으로 처리하도록 */

    int sent = 0;
    for (int i = 0; i < MAX_MEMBERS; i++) {
        member_t *m = &s_members[i];
        if (m->state != SLOT_READY) {
            continue;
        }
        int rc = ble_gattc_write_flat(m->conn_handle, m->chr_val_handle, buf, len, NULL, NULL);
        if (rc == 0) {
            sent++;
        } else {
            ESP_LOGW(TAG, "broadcast: vid=%u write 실패 rc=%d", m->vid, rc);
        }
    }
    ESP_LOGI(TAG, "broadcast: %d개 멤버로 %u bytes 중계", sent, len);
    return sent;
}

uint8_t ble_multirole_member_count(void)
{
    return count_ready();
}

/* ---- 멤버 READY 확정 -------------------------------------------------- */
/* 특성(+가능하면 CCCD)까지 확보한 멤버를 READY로 올리고 PC에 보고한다. */
static void member_ready(member_t *m)
{
    m->state = SLOT_READY;
    /* 고정형이면 광고 nodeID(NN)를 그대로 가상ID로, 아니면 연결순서(2,3,…). */
    m->vid = (s_ordered && m->nn >= 1) ? m->nn : s_next_vid++;
    uint8_t ready = count_ready();
    ESP_LOGI(TAG, "%s 멤버 READY: conn=%u vid=%u chr=%u cccd=%u (%u/%u)",
             s_edge ? "[edge]" : "[hub]",
             m->conn_handle, m->vid, m->chr_val_handle, m->cccd_handle,
             ready, s_target_members);
    if (!s_edge) {
        rcube_cmd_report_members(ready);   /* 독립유닛에는 보고할 PC가 없다 */
    }
    /* 멤버 1개 연결음, 담당 분기가 전부 붙으면 유닛구성완료 멜로디(기획서 5장). */
    if (ready >= s_target_members) {
        rcube_buzzer_play(RCUBE_MELODY_LINK_COMPLETED);
        /* 허브/edge central LED: 멤버 전원 연결 → 점멸에서 상시 점등으로. */
        rcube_status_set_mode(RCUBE_LED_LINKED);
        if (s_edge) {
            /* 기획서 9장 [독립로봇유닛]: 연결이 완료되면 edge central이 전체 큐브에
             * 센서 전송 시작 명령을 보낸다. BLE 분기는 여기서, CAN 분기는
             * can_transport가 자기 완료 시점에 따로 지시한다. */
            rcube_cmd_sensor_stream_all(true, RCUBE_SENSOR_PERIOD_DEFAULT_MS);
            /* 기획서 7.4-6: 이어서 저장된 미션코드 실행으로 넘어간다.
             * 미션코드(F0~F4)는 8장 설계 확정 후 별도 구현. */
            ESP_LOGI(TAG, "edge central: BLE 멤버 전원 연결 완료 "
                          "(미션코드 실행은 8장에서 연결 예정)");
        }
    } else {
        rcube_buzzer_play(RCUBE_MELODY_LINK);
    }
    scan_if_needed();   /* 더 필요하면 계속 스캔 */
}

/* ---- GATT 클라이언트 탐색 콜백 --------------------------------------- */
/* CCCD(0x2902)를 찾아 notify를 구독한다. 멤버의 회신(CmdAck·D4 등)을 받아
 * PC로 중계하기 위한 역방향 경로다. 실패해도 연결 자체는 유지한다. */
static int dsc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
    member_t *m = (member_t *)arg;
    if (error->status == 0 && dsc != NULL) {
        if (ble_uuid_u16(&dsc->uuid.u) == BLE_GATT_DSC_CLT_CFG_UUID16) {
            m->cccd_handle = dsc->handle;
        }
        return 0;
    }
    if (!s_active || m->state != SLOT_DISCOVERING) {
        return 0;
    }
    if (m->cccd_handle != 0) {
        uint8_t val[2] = {0x01, 0x00};   /* notify enable */
        int rc = ble_gattc_write_flat(conn_handle, m->cccd_handle, val, sizeof(val), NULL, NULL);
        if (rc != 0) {
            ESP_LOGW(TAG, "멤버 conn=%u notify 구독 실패 rc=%d (회신 중계 불가)", conn_handle, rc);
        }
    } else {
        ESP_LOGW(TAG, "멤버 conn=%u CCCD 없음 → 회신 중계 불가", conn_handle);
    }
    member_ready(m);
    return 0;
}

static int chr_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg)
{
    member_t *m = (member_t *)arg;
    if (error->status == 0 && chr != NULL) {
        m->chr_val_handle = chr->val_handle;
        return 0;
    }
    if (error->status != BLE_HS_EDONE) {
        ESP_LOGW(TAG, "char 탐색 오류 conn=%u status=%d", conn_handle, error->status);
    }
    /* 탐색 종료(EDONE) 또는 오류 → 특성 확보 여부로 판정. */
    if (!s_active || m->state != SLOT_DISCOVERING) {
        return 0;
    }
    if (m->chr_val_handle == 0) {
        ESP_LOGW(TAG, "멤버 conn=%u RCBE 특성 없음 → 연결 종료", conn_handle);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }
    /* 회신 중계를 위해 CCCD를 찾아 구독한 뒤 READY로 올린다. */
    int rc = ble_gattc_disc_all_dscs(conn_handle, m->chr_val_handle, m->svc_end,
                                     dsc_disc_cb, m);
    if (rc != 0) {
        ESP_LOGW(TAG, "descriptor 탐색 시작 실패 rc=%d → 구독 없이 진행", rc);
        member_ready(m);
    }
    return 0;
}

static int svc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *service, void *arg)
{
    member_t *m = (member_t *)arg;
    if (error->status == 0 && service != NULL) {
        m->svc_start = service->start_handle;
        m->svc_end = service->end_handle;
        return 0;
    }
    if (!s_active || m->state != SLOT_DISCOVERING) {
        return 0;
    }
    if (error->status != BLE_HS_EDONE || m->svc_start == 0) {
        ESP_LOGW(TAG, "멤버 conn=%u RCBE 서비스 없음(status=%d) → 종료", conn_handle, error->status);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }
    /* 서비스 확보 → RCBE 특성 탐색. */
    int rc = ble_gattc_disc_chrs_by_uuid(conn_handle, m->svc_start, m->svc_end,
                                         &rcube_chr_uuid.u, chr_disc_cb, m);
    if (rc != 0) {
        ESP_LOGE(TAG, "char 탐색 시작 실패 rc=%d → 종료", rc);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    return 0;
}

/* ---- 멤버 연결 GAP 이벤트 -------------------------------------------- */
static int member_gap_event(struct ble_gap_event *event, void *arg)
{
    member_t *m = (member_t *)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        s_connect_pending = false;
        if (event->connect.status == 0) {
            m->conn_handle = event->connect.conn_handle;
            m->state = SLOT_DISCOVERING;
            m->svc_start = 0;
            m->svc_end = 0;
            m->chr_val_handle = 0;
            ESP_LOGI(TAG, "멤버 연결됨 conn=%u → 서비스 탐색", m->conn_handle);
            int rc = ble_gattc_disc_svc_by_uuid(m->conn_handle, &rcube_svc_uuid.u,
                                                svc_disc_cb, m);
            if (rc != 0) {
                ESP_LOGE(TAG, "서비스 탐색 시작 실패 rc=%d → 종료", rc);
                ble_gap_terminate(m->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            }
        } else {
            ESP_LOGW(TAG, "멤버 연결 실패 status=%d", event->connect.status);
            free_slot(m);
            scan_if_needed();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT: {
        bool was_ready = (m->state == SLOT_READY);
        ESP_LOGI(TAG, "멤버 연결 끊김 conn=%u reason=%d (vid=%u)",
                 event->disconnect.conn.conn_handle, event->disconnect.reason, m->vid);
        free_slot(m);
        if (s_active) {
            if (was_ready) {
                rcube_cmd_report_members(count_ready());   /* 멤버 수 감소 통지 */
                /* 담당 멤버가 빠졌으므로 허브 LED는 다시 대기 점멸로. */
                if (count_ready() < s_target_members) {
                    rcube_status_set_mode(RCUBE_LED_HUB_WAIT);
                }
            }
            scan_if_needed();   /* 빈자리 다시 채움 */
        }
        return 0;
    }

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "멤버 mtu conn=%u mtu=%d", event->mtu.conn_handle, event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        /* 멤버가 보낸 회신(CmdAck·GetNodeConfig 등)을 PC로 그대로 올린다.
         * 발신자를 알 수 있도록 target 바이트를 그 멤버의 가상ID로 재기입한다.
         * (멤버는 자기를 허브로 알고 0xFE로 보내므로 그대로 두면 구분이 안 된다.) */
        if (m->state != SLOT_READY) {
            return 0;
        }
        uint8_t buf[FWD_BUF_MAX];
        uint16_t len = 0;
        if (ble_hs_mbuf_to_flat(event->notify_rx.om, buf, sizeof(buf), &len) != 0 || len < 4) {
            ESP_LOGW(TAG, "멤버 vid=%u notify 파싱 실패(len=%u)", m->vid, len);
            return 0;
        }
        buf[0] = m->vid;
        if (ble_rcube_notify_pc(buf, len) == 0) {
            ESP_LOGI(TAG, "멤버 vid=%u 회신 %u bytes → PC 중계 (op=0x%02x)", m->vid, len, buf[1]);
        }
        return 0;
    }

    default:
        return 0;
    }
}

/* ---- 스캔(발견) GAP 이벤트 ------------------------------------------- */
static int disc_gap_event(struct ble_gap_event *event, void *arg)
{
    if (event->type == BLE_GAP_EVENT_DISC_COMPLETE) {
        /* BLE_HS_FOREVER면 우리가 취소하기 전엔 오지 않지만, 방어적으로 재개. */
        scan_if_needed();
        return 0;
    }
    if (event->type != BLE_GAP_EVENT_DISC) {
        return 0;
    }
    if (!s_active || s_connect_pending || count_busy() >= s_target_members) {
        return 0;
    }

    /* 광고 필드에서 이름을 뽑아 "RCUBE" 접두어인지 확인. */
    struct ble_hs_adv_fields fields;
    if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) != 0) {
        return 0;
    }
    if (fields.name == NULL || fields.name_len < RCUBE_NAME_PREFIX_LEN ||
        memcmp(fields.name, RCUBE_NAME_PREFIX, RCUBE_NAME_PREFIX_LEN) != 0) {
        return 0;
    }
    uint8_t nn = 0;
    bool have_nn = parse_name_nn(fields.name, fields.name_len, &nn);

    /* 고정형 연결: 저장 노드ID(NN≥1)가 있는 큐브만 연결한다(가상ID=NN).
     * 연결 순서는 무관 — 각 멤버는 광고의 nodeID로 식별되어 그 색을 받는다.
     * (비연속 노드ID·혼합유닛의 BLE 부분집합도 처리됨.) */
    if (s_ordered && (!have_nn || nn < 1)) {
        return 0;
    }
    /* edge central(7.4-4)은 저장된 멤버 맵에 BLE로 올라 있는 노드ID만 받아들인다.
     * 옆 유닛의 큐브나 CAN으로 세팅된 노드가 섞여 들어오는 것을 막는다. */
    if (s_edge) {
        if (nn == rcube_config_node_id() ||
            rcube_config_member_cmf(nn) != RCUBE_MEMBER_BLE) {
            return 0;
        }
    }
    if (addr_known(&event->disc.addr)) {
        return 0;   /* 이미 연결(시도)한 기기 */
    }

    member_t *m = alloc_slot();
    if (m == NULL) {
        return 0;
    }

    /* 후보 발견 → 스캔 중지하고 연결 시도(직렬화). */
    s_connect_pending = true;
    if (ble_gap_disc_active()) {
        ble_gap_disc_cancel();
    }
    m->state = SLOT_CONNECTING;
    m->addr = event->disc.addr;
    m->nn = have_nn ? nn : 0;

    uint8_t own_addr_type = ble_rcube_own_addr_type();
    int rc = ble_gap_connect(own_addr_type, &event->disc.addr, 10000 /*ms*/,
                             NULL, member_gap_event, m);
    if (rc != 0) {
        ESP_LOGW(TAG, "connect 시작 실패 rc=%d", rc);
        free_slot(m);
        s_connect_pending = false;
        scan_if_needed();
    } else {
        char addr_str[18];
        const uint8_t *a = event->disc.addr.val;
        snprintf(addr_str, sizeof(addr_str), "%02x:%02x:%02x:%02x:%02x:%02x",
                 a[5], a[4], a[3], a[2], a[1], a[0]);
        ESP_LOGI(TAG, "멤버 후보 발견 [%s] → 연결 시도", addr_str);
    }
    return 0;
}
