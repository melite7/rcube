#include "ble_multirole.h"
#include "ble_rcube.h"
#include "rcube_cmd.h"
#include "rcube_buzzer.h"

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

/* R4: 아그리게이터 제외 멤버 최대 3대. */
#define MAX_MEMBERS 3

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
    uint8_t      vid;          /* PC와 맞춘 가상 노드 ID(2..N) */
    uint8_t      nn;           /* 광고이름에서 읽은 노드ID(순서고정 연결용) */
} member_t;

/* --- 상태 (모두 NimBLE 호스트 태스크 단일 스레드에서만 접근) --- */
static bool     s_active;
static uint8_t  s_target_members;   /* 연결해야 할 멤버 수 = link_count-1 */
static uint8_t  s_group_mode;
static bool     s_ordered;          /* 고정형: 광고 nodeID(NN)를 그대로 가상ID로 사용 */
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
    s_target_members = members;
    s_group_mode = group_mode;
    s_ordered = (flags & RCUBE_AGG_FLAG_ORDERED) != 0;
    s_next_vid = 2;            /* 아그리게이터=1(0xFE), 멤버는 2부터 */
    s_connect_pending = false;

    ESP_LOGI(TAG, "aggregator 시작: 목표 멤버 %u대(group_mode=0x%02x, %s)",
             members, group_mode, s_ordered ? "순서고정 NN순" : "연결순");

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

int ble_multirole_fix_order(void)
{
    if (!s_active) {
        return -1;
    }
    int sent = 0;
    for (int i = 0; i < MAX_MEMBERS; i++) {
        member_t *m = &s_members[i];
        if (m->state != SLOT_READY) {
            continue;
        }
        /* D3 SET_NODE 프레임: 이 멤버의 가상ID를 노드ID로 저장시킨다. */
        uint8_t f[6];
        uint16_t total = sizeof(f);
        f[0] = RCUBE_ADDR_HUB;      /* 멤버가 자기 명령으로 처리 */
        f[1] = RCUBE_OP_SetNodeConfig;
        f[2] = (uint8_t)((total >> 8) & 0xFF);
        f[3] = (uint8_t)(total & 0xFF);
        f[4] = RCUBE_D3_SUB_SET_NODE;
        f[5] = m->vid;              /* 저장할 노드ID = 가상ID */
        int rc = ble_gattc_write_flat(m->conn_handle, m->chr_val_handle, f, total, NULL, NULL);
        if (rc == 0) {
            sent++;
            ESP_LOGI(TAG, "fix_order: vid=%u 멤버에 노드ID 저장 지시", m->vid);
        } else {
            ESP_LOGW(TAG, "fix_order: vid=%u write 실패 rc=%d", m->vid, rc);
        }
    }
    return sent;
}

uint8_t ble_multirole_member_count(void)
{
    return count_ready();
}

/* ---- GATT 클라이언트 탐색 콜백 --------------------------------------- */
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
    /* 멤버 READY. 가상ID 부여 후 PC에 0xA1 보고. */
    m->state = SLOT_READY;
    /* 고정형이면 광고 nodeID(NN)를 그대로 가상ID로, 아니면 연결순서(2,3,…). */
    m->vid = (s_ordered && m->nn >= 1) ? m->nn : s_next_vid++;
    uint8_t ready = count_ready();
    ESP_LOGI(TAG, "멤버 READY: conn=%u vid=%u chr=%u (%u/%u)",
             conn_handle, m->vid, m->chr_val_handle, ready, s_target_members);
    rcube_cmd_report_members(ready);
    /* 멤버 1개 연결음, 전원 연결되면 전체 완료 멜로디. */
    if (ready >= s_target_members) {
        rcube_buzzer_play(RCUBE_MELODY_LINK_COMPLETED);
    } else {
        rcube_buzzer_play(RCUBE_MELODY_LINK);
    }
    scan_if_needed();   /* 더 필요하면 계속 스캔 */
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
            }
            scan_if_needed();   /* 빈자리 다시 채움 */
        }
        return 0;
    }

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "멤버 mtu conn=%u mtu=%d", event->mtu.conn_handle, event->mtu.value);
        return 0;

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
