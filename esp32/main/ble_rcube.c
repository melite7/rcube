#include "ble_rcube.h"
#include "board_led.h"
#include "rcube_cmd.h"

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

/* 외부에서 이 이름으로 R큐브를 찾는다. */
#define RCUBE_DEVICE_NAME "RCUBE00.00"

/* LED 상태색(순수 색상; 실제 밝기는 board_led의 LED_BRIGHTNESS로 스케일). */
#define LED_IDLE_R 0
#define LED_IDLE_G 0
#define LED_IDLE_B 255     /* 파랑 = 대기(부팅 성공) */
#define LED_ADV_R 0
#define LED_ADV_G 255
#define LED_ADV_B 255      /* 청록 = 광고중(발견 가능) */
#define LED_CONN_R 0
#define LED_CONN_G 255
#define LED_CONN_B 0       /* 초록 = 연결됨 */

/* ---- 커스텀 GATT 서비스 (연결 후 상호작용 확인용) --------------------
 * Service : 52434245-0000-1000-8000-00805f9b34fb  (ASCII "RCBE")
 * Char    : 52434245-0001-1000-8000-00805f9b34fb  (READ | WRITE | NOTIFY)
 * BLE_UUID128_INIT는 바이트를 LSB부터 나열한다(위 표기의 역순). */
static const ble_uuid128_t rcube_svc_uuid =
    BLE_UUID128_INIT(0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x45, 0x42, 0x43, 0x52);
static const ble_uuid128_t rcube_chr_uuid =
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
static int ble_rcube_send_notify(const uint8_t *frame, uint16_t len)
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
    return ble_svc_gap_device_name_set(RCUBE_DEVICE_NAME);
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
    board_led_set(LED_ADV_R, LED_ADV_G, LED_ADV_B);
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
            board_led_set(LED_CONN_R, LED_CONN_G, LED_CONN_B);
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
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_connected = false;
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        xSemaphoreGive(s_lock);
        board_led_set(LED_IDLE_R, LED_IDLE_G, LED_IDLE_B);
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

    /* 명령 레이어 준비(회신은 이 특성의 notify로 나간다). */
    rcube_cmd_init(ble_rcube_send_notify);

    nimble_port_freertos_init(host_task);
    ESP_LOGI(TAG, "BLE initialized (name=\"%s\", not advertising yet)",
             RCUBE_DEVICE_NAME);
}

void ble_rcube_start_advertising(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_start_requested = true;
    xSemaphoreGive(s_lock);
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
