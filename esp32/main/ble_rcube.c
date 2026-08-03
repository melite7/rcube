#include "ble_rcube.h"
#include "board_led.h"
#include "rcube_cmd.h"
#include "ble_multirole.h"
#include "rcube_config.h"
#include "rcube_status.h"
#include "rcube_buzzer.h"

#include <stdio.h>

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble";

/* 광고 이름: 일반 "RCUBEROBOT.GG.NN", 설정모드 "RCUBECONFIG.GG.NN".
 * (GG=그룹번호, NN=노드ID 각 10진 2자리). 둘 다 접두어 "RCUBE" 공유. */
#define RCUBE_NAME_ROBOT  "RCUBEROBOT"
#define RCUBE_NAME_CONFIG "RCUBECONFIG"

static bool s_config_name;   /* 설정모드 광고 이름(RCUBECONFIG) 사용 여부 */

/* 현재 설정(group/node/모드)으로 광고 이름을 만들어 GAP 장치이름에 설정한다. */
static void update_device_name(void)
{
    char name[40];
    snprintf(name, sizeof(name), "%s.%02u.%02u",
             s_config_name ? RCUBE_NAME_CONFIG : RCUBE_NAME_ROBOT,
             rcube_config_group_id(), rcube_config_node_id());
    ble_svc_gap_device_name_set(name);
}

/* 노드LED는 rcube_status가 전담한다(기획서 5장 [ID 표시용 칼라LED 점등 규칙]).
 * 이 파일은 연결 상태 전이만 통보하고 색·점멸은 직접 만지지 않는다. */

/* ---- 커스텀 GATT 서비스 (연결 후 상호작용 확인용) --------------------
 * Service : 52434245-0000-1000-8000-00805f9b34fb  (ASCII "RCBE")
 * Char    : 52434245-0001-1000-8000-00805f9b34fb  (READ | WRITE | NOTIFY)
 * BLE_UUID128_INIT는 바이트를 LSB부터 나열한다(위 표기의 역순). */
const ble_uuid128_t rcube_svc_uuid =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x45, 0x42, 0x43, 0x52);
const ble_uuid128_t rcube_chr_uuid =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x01, 0x00, 0x45, 0x42, 0x43, 0x52);

static uint16_t s_chr_val_handle;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;  /* 현재 연결(회신 notify용) */
static uint8_t s_status_byte = 0x00;   /* READ 시 반환할 상태(추후 확장) */

/* ---- 상태 플래그(뮤텍스 보호) ---------------------------------------- */
static SemaphoreHandle_t s_lock;
static bool s_synced;          /* 호스트 sync 완료(주소 확정) */
static bool s_start_requested; /* 버튼으로 광고 요청됨 */
static bool s_advertising;     /* 광고 진행 중 */
static bool s_connected;       /* 연결됨 */
static uint8_t s_own_addr_type;

static int gap_event(struct ble_gap_event *event, void *arg);
static void try_advertise(void);

/* ---- 회신(notify) 송신 : 명령 레이어의 responder ---------------------
 * 완성된 와이어 프레임을 현재 연결의 R큐브 특성으로 notify 한다.
 * NimBLE 호스트 태스크 컨텍스트(write 콜백)에서 호출된다. */
int ble_rcube_notify_pc(const uint8_t *frame, uint16_t len)
{
    uint16_t conn;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    conn = s_conn_handle;
    xSemaphoreGive(s_lock);
    if (conn == BLE_HS_CONN_HANDLE_NONE) {
        return -1;   /* 연결 없음 */
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(frame, len);
    if (om == NULL) {
        ESP_LOGE(TAG, "notify: mbuf alloc 실패");
        return -1;
    }
    /* ble_gatts_notify_custom 은 성공/실패와 무관하게 om 을 소비한다. */
    int rc = ble_gatts_notify_custom(conn, s_chr_val_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "notify 실패; rc=%d (구독 전이면 정상)", rc);
    }
    return rc;
}

/* ---- GATT access ----------------------------------------------------- */
static int chr_access(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR: {
        int rc = os_mbuf_append(ctxt->om, &s_status_byte, sizeof(s_status_byte));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        uint8_t buf[64];
        uint16_t len = 0;
        int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &len);
        if (rc != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        /* 표준 프레임을 명령 레이어로 넘겨 파싱·디스패치(회신은 notify). */
        if (len > 0) {
            s_status_byte = buf[0];   /* 마지막 OpCode를 READ 상태로 노출(디버그) */
        }
        rcube_cmd_on_frame(buf, len);
        return 0;
    }
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &rcube_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &rcube_chr_uuid.u,
                .access_cb = chr_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                         BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_chr_val_handle,
            },
            { 0 }, /* 특성 끝 */
        },
    },
    { 0 }, /* 서비스 끝 */
};

static int gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        return rc;
    }
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        return rc;
    }
    update_device_name();   /* 현재 group/node로 이름 설정 */
    return 0;
}

/* ---- 광고 ------------------------------------------------------------ */
/* 실제 광고 시작. 조건 검사(try_advertise)를 통과한 뒤에만 호출한다. */
static void adv_start(void)
{
    struct ble_hs_adv_fields fields;
    struct ble_gap_adv_params advp;
    int rc;

    /* 광고 페이로드: 플래그 + TX파워 + 완전한 이름(31바이트 이내).
     * 128비트 서비스 UUID까지 넣으면 31바이트를 초과하므로 이름만 싣는다. */
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    update_device_name();   /* 광고 직전 현재 group/node로 이름 갱신 */
    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields failed; rc=%d", rc);
        return;
    }

    memset(&advp, 0, sizeof(advp));
    advp.conn_mode = BLE_GAP_CONN_MODE_UND;   /* 연결 가능 */
    advp.disc_mode = BLE_GAP_DISC_MODE_GEN;   /* 일반 발견 가능 */
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &advp, gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "adv_start failed; rc=%d", rc);
        return;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_advertising = true;
    xSemaphoreGive(s_lock);
    /* 광고중 = 아직 미연결 → IDLE(1초 점멸). 설정모드면 set_mode가 무시한다. */
    rcube_status_set_mode(RCUBE_LED_IDLE);
    ESP_LOGI(TAG, "advertising as \"%s\" (connectable)", name);
}

/* 조건이 맞으면 광고를 시작한다: sync 완료 && 요청됨 && 미광고 && 미연결. */
static void try_advertise(void)
{
    bool go = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_synced && s_start_requested && !s_advertising && !s_connected) {
        go = true;
    }
    xSemaphoreGive(s_lock);
    if (go) {
        adv_start();
    }
}

/* ---- GAP 이벤트 ------------------------------------------------------ */
static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_connected = true;
            s_advertising = false;
            s_conn_handle = event->connect.conn_handle;
            xSemaphoreGive(s_lock);
            /* 기획서 5장 [소리 규칙](2026-07-30): 상위(PC/앱/edge central)가 실제로
             * 붙은 이 순간에 연결음을 낸다. 연결모드 진입(버튼)은 "붙을 준비가 됐다"일
             * 뿐이라 그 시점에 소리를 내면 아직 안 붙었는데 붙은 것처럼 들린다.
             * 끊길 때의 DISCONNECT와 짝을 이룬다. */
            rcube_buzzer_play(RCUBE_MELODY_LINK);
            /* 연결 완료 → 상시 점등. 색은 고정형이면 자기 노드ID 색, 비고정형이면
             * 상위가 곧 보내줄 가상 노드ID 색(E0)이 덮어쓴다. */
            rcube_status_set_mode(RCUBE_LED_LINKED);
            ESP_LOGI(TAG, "connected (handle=%d)", event->connect.conn_handle);
        } else {
            ESP_LOGW(TAG, "connect failed; status=%d, resume adv",
                     event->connect.status);
            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_advertising = false;
            xSemaphoreGive(s_lock);
            try_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected; reason=%d", event->disconnect.reason);
        /* 기획서 5장 [소리 규칙](2026-07-30): 연결이 끊기면 START를 역순으로 연주해
         * 사용자가 붙었는지 떨어졌는지 귀로 구분할 수 있게 한다. */
        rcube_buzzer_play(RCUBE_MELODY_DISCONNECT);
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_connected = false;
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        xSemaphoreGive(s_lock);
        /* PC 연결이 끊기면 아그리게이터 역할도 해제(멤버 전원 종료). */
        ble_multirole_stop_aggregator();
        /* 상위가 지정했던 가상 노드ID 색을 버리고 미연결 표시로 되돌린다
         * (기획서 5장: 비고정형 가상색은 저장되지 않고 reset 후 흰색 점멸).
         * 설정모드 광고 중이었다면 설정모드 표시(흰색 0.25s)로 복귀한다 —
         * 설정모드 자체는 재부팅 전까지 유지되고, LED만 연결 동안 양보한 것이다. */
        rcube_status_clear_color();
        if (s_config_name) {
            rcube_status_enter_config_mode();
        } else {
            rcube_status_set_mode(RCUBE_LED_IDLE);
        }
        try_advertise();   /* 재연결을 위해 광고 재개 */
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "adv complete; reason=%d", event->adv_complete.reason);
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_advertising = false;
        xSemaphoreGive(s_lock);
        try_advertise();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "subscribe; attr=%d notify=%d",
                 event->subscribe.attr_handle, event->subscribe.cur_notify);
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "mtu update; mtu=%d", event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

/* ---- 호스트 콜백 ----------------------------------------------------- */
static void on_reset(int reason)
{
    ESP_LOGE(TAG, "nimble host reset; reason=%d", reason);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_synced = false;
    s_advertising = false;
    s_connected = false;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    xSemaphoreGive(s_lock);
}

static void on_sync(void)
{
    /* 공용 주소 확보 후 광고에 쓸 주소 타입 결정. */
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ensure_addr failed; rc=%d", rc);
    }
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "infer addr type failed; rc=%d", rc);
        return;
    }

    uint8_t addr[6] = {0};
    ble_hs_id_copy_addr(s_own_addr_type, addr, NULL);
    ESP_LOGI(TAG, "host synced; addr=%02x:%02x:%02x:%02x:%02x:%02x",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_synced = true;
    xSemaphoreGive(s_lock);
    try_advertise();   /* 버튼이 sync 전에 눌렸다면 여기서 광고 시작 */
}

static void host_task(void *param)
{
    ESP_LOGI(TAG, "nimble host task started");
    nimble_port_run();            /* nimble_port_stop() 전까지 리턴 안 함 */
    nimble_port_freertos_deinit();
}

/* ---- 공개 API -------------------------------------------------------- */
void ble_rcube_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    configASSERT(s_lock != NULL);

    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed; err=%d", ret);
        return;
    }

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;  /* 페어링 없이 연결 허용 */

    int rc = gatt_svr_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "gatt_svr_init failed; rc=%d", rc);
        return;
    }

    /* 명령 레이어 준비: 회신=peripheral notify, 멀티롤/중계=central 레이어. */
    static const rcube_cmd_ops_t cmd_ops = {
        .send        = ble_rcube_notify_pc,
        .agg_start   = ble_multirole_start_aggregator,
        .agg_stop    = ble_multirole_stop_aggregator,
        .forward     = ble_multirole_forward,
        .forward_all = ble_multirole_broadcast,
    };
    rcube_cmd_init(&cmd_ops);

    nimble_port_freertos_init(host_task);
    ESP_LOGI(TAG, "BLE initialized (name=\"%s\", not advertising yet)",
             ble_svc_gap_device_name());
}

bool ble_rcube_start_advertising(void)
{
    /* 기획서 5장 [연결모드 진입 동작 - ECF=0]·7.2-8:
     *   CMF=0(BLE) 큐브만 연결모드에서 RCUBEROBOT으로 광고한다.
     *   CMF=1(CAN) 큐브는 CAN 하트비트로 존재를 알리므로 BLE 광고를 하지 않는다.
     *   (CAN으로 세팅한 큐브를 되돌리는 복구 경로는 설정모드 광고 = RCUBECONFIG.) */
    if (rcube_config_cmf() == 1) {
        ESP_LOGW(TAG, "CMF=CAN 큐브 → 연결모드 BLE 광고 생략(CAN 하트비트로 연결). "
                      "BLE로 되돌리려면 버튼 3초 롱프레스로 설정모드 진입.");
        return false;
    }
    s_config_name = false;   /* 일반 이름(RCUBEROBOT) */
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_start_requested = true;
    xSemaphoreGive(s_lock);
    try_advertise();
    return true;
}

void ble_rcube_start_config_advertising(void)
{
    s_config_name = true;    /* 설정모드 이름(RCUBECONFIG) */
    bool restart;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_start_requested = true;
    restart = s_advertising;   /* 이미 광고 중이면 이름 바꾸려 재시작 */
    xSemaphoreGive(s_lock);
    if (restart) {
        ble_gap_adv_stop();
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_advertising = false;
        xSemaphoreGive(s_lock);
    }
    try_advertise();
}

bool ble_rcube_is_active(void)
{
    bool active;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    active = s_advertising || s_connected;
    xSemaphoreGive(s_lock);
    return active;
}

uint8_t ble_rcube_own_addr_type(void)
{
    return s_own_addr_type;
}
