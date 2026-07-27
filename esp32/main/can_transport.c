#include "can_transport.h"
#include "rcube_cmd.h"
#include "rcube_config.h"
#include "rcube_status.h"
#include "rcube_buzzer.h"
#include "rcube_sensor.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#include "rcube_protocol.h"

static const char *TAG = "can";

/* ---- 핀맵 ---- */
#define CAN_TX_GPIO      GPIO_NUM_4
#define CAN_RX_GPIO      GPIO_NUM_5
#define CAN_STB_GPIO     GPIO_NUM_6    /* Low=정상, High=대기 */
#define CAN_TERM_EN_GPIO GPIO_NUM_7    /* High=120Ω 종단 ON */

#define HEARTBEAT_MS 1000

static uint8_t s_node_id;
static uint8_t s_term_id;
static bool s_running;

/* ---- edge central(ECF=1)의 CAN 멤버 대기 상태 (기획서 7.4-4) ---- */
static bool    s_edge;                       /* CAN 멤버 대기 중 */
static uint8_t s_edge_expected;              /* 기대 CAN 멤버 수 */
static uint8_t s_edge_found;                 /* 발견된 수 */
static bool    s_edge_seen[RCUBE_MAX_NODES]; /* 인덱스 = 노드ID-1 */

/* 멤버 노드ID를 발견 처리한다. 전부 모이면 완료 표시. rx_task에서만 호출. */
static void edge_note_member(uint8_t node_id)
{
    if (!s_edge || node_id < 1 || node_id > RCUBE_MAX_NODES) {
        return;
    }
    if (node_id == s_node_id) {
        return;   /* 자기 자신 */
    }
    if (rcube_config_member_cmf(node_id) != RCUBE_MEMBER_CAN) {
        return;   /* 우리 유닛의 CAN 멤버가 아님 */
    }
    if (s_edge_seen[node_id - 1]) {
        return;   /* 이미 발견 */
    }
    s_edge_seen[node_id - 1] = true;
    s_edge_found++;
    ESP_LOGI(TAG, "edge central: CAN 멤버 노드 0x%02x 발견 (%u/%u)",
             node_id, s_edge_found, s_edge_expected);

    /* 기획서 7.4-5: 각 큐브는 고정형이라 자기 노드ID 색을 스스로 켠다.
     * edge central은 확인만 하되, 색 확인 명령을 한 번 보내 표시를 맞춘다. */
    uint8_t r, g, b;
    rcube_status_node_color(node_id, &r, &g, &b);
    uint8_t led[4] = {1, r, g, b};   /* SetSK6812LED payload = [n][R,G,B] */
    can_transport_send(RCUBE_PRI_PERIPHERAL, RCUBE_OP_SetSK6812LED, node_id, led, sizeof(led));

    if (s_edge_found >= s_edge_expected) {
        ESP_LOGI(TAG, "edge central: CAN 멤버 전원 발견(%u대)", s_edge_expected);
        rcube_buzzer_play(RCUBE_MELODY_LINK_COMPLETED);
        /* 기획서 9장 [독립로봇유닛]: CAN 분기에도 센서 전송 시작을 지시한다.
         * (BLE 분기는 ble_multirole이 자기 완료 시점에 따로 지시한다.) */
        uint16_t period = RCUBE_SENSOR_PERIOD_DEFAULT_MS;
        uint8_t p[3] = {1, (uint8_t)(period >> 8), (uint8_t)(period & 0xFF)};
        can_transport_send(RCUBE_PRI_CONFIG, RCUBE_OP_SetSensorStream,
                           RCUBE_ADDR_BROADCAST, p, sizeof(p));
    } else {
        rcube_buzzer_play(RCUBE_MELODY_LINK);
    }
}

bool can_transport_start_edge(void)
{
    uint8_t me = rcube_config_node_id();
    uint8_t expected = 0;
    for (uint8_t nid = 1; nid <= RCUBE_MAX_NODES; nid++) {
        if (nid != me && rcube_config_member_cmf(nid) == RCUBE_MEMBER_CAN) {
            expected++;
        }
    }
    if (expected == 0) {
        ESP_LOGI(TAG, "edge central: CAN 멤버 없음 → CAN 서버 대기 생략");
        return false;
    }
    if (!s_running) {
        ESP_LOGE(TAG, "edge central: CAN 멤버가 있으나 TWAI 미기동 — 초기화 실패 확인");
        return false;
    }
    memset(s_edge_seen, 0, sizeof(s_edge_seen));
    s_edge_found = 0;
    s_edge_expected = expected;
    s_edge = true;
    ESP_LOGI(TAG, "edge central: CAN 멤버 %u대 대기 시작(부팅/하트비트 수신)", expected);
    return true;
}

uint8_t can_transport_edge_found(void)    { return s_edge_found; }
uint8_t can_transport_edge_expected(void) { return s_edge_expected; }

/* ---- 저수준 송신 ---------------------------------------------------- */
esp_err_t can_transport_send(uint8_t priority, uint8_t op_code,
                             uint8_t dst, const uint8_t *data, uint8_t len)
{
    if (!s_running) return ESP_ERR_INVALID_STATE;
    if (len > 8) len = 8;   /* Classic CAN 데이터필드 최대 8B (초과분은 MULTI 필요 — TODO) */

    twai_message_t msg = {0};
    msg.identifier = RCUBE_CAN_ID(priority, op_code, 0 /*multi*/, 0 /*flag*/, s_node_id, dst);
    msg.extd = 1;
    msg.data_length_code = len;
    if (data && len) memcpy(msg.data, data, len);

    return twai_transmit(&msg, pdMS_TO_TICKS(10));
}

/* rcube_cmd 응답 콜백: 표준프레임을 CAN으로 회신(dst=마스터 0xFE). */
static int can_reply(const uint8_t *frame, uint16_t len)
{
    if (len < 4) return -1;
    uint8_t op = frame[1];
    const uint8_t *payload = frame + 4;
    uint8_t plen = (uint8_t)((len - 4) > 8 ? 8 : (len - 4));
    esp_err_t err = can_transport_send(RCUBE_PRI_QUERY, op, RCUBE_CAN_SRC_MASTER, payload, plen);
    return (err == ESP_OK) ? 0 : -1;
}

/* ---- 수신 → 표준프레임 재구성 → 명령 레이어 ------------------------- */
static void rx_task(void *arg)
{
    while (1) {
        twai_message_t msg;
        esp_err_t err = twai_receive(&msg, portMAX_DELAY);
        if (err != ESP_OK || !msg.extd) {
            continue;
        }
        uint32_t id = msg.identifier;
        uint8_t op  = RCUBE_CAN_OPCODE(id);
        uint8_t dst = RCUBE_CAN_DST(id);
        uint8_t src = RCUBE_CAN_SRC(id);

        /* edge central: 멤버의 부팅 알림/하트비트로 존재를 확인한다(7.4-4).
         * 이 두 프레임은 브로드캐스트로 오므로 아래 대상 필터보다 먼저 본다. */
        if (s_edge && (op == RCUBE_OP_NodeAnnounce || op == RCUBE_OP_Heartbeat)) {
            uint8_t nid = (msg.data_length_code >= 1) ? msg.data[0] : src;
            edge_note_member(nid);
            continue;
        }

        /* 나(node_id)/허브(0xFE)/브로드캐스트(0xFF) 대상만 처리. */
        if (dst != s_node_id && dst != RCUBE_ADDR_HUB && dst != RCUBE_ADDR_BROADCAST) {
            continue;
        }
        /* 표준프레임 [target][op][size BE][data...] 로 재구성해 공용 디스패처로. */
        uint8_t frame[4 + 8];
        uint8_t dlc = msg.data_length_code > 8 ? 8 : msg.data_length_code;
        uint16_t total = 4 + dlc;
        frame[0] = dst;
        frame[1] = op;
        frame[2] = (uint8_t)((total >> 8) & 0xFF);
        frame[3] = (uint8_t)(total & 0xFF);
        if (dlc) memcpy(frame + 4, msg.data, dlc);
        ESP_LOGI(TAG, "RX op=0x%02x src=0x%02x dst=0x%02x dlc=%u",
                 op, RCUBE_CAN_SRC(id), dst, dlc);
        rcube_cmd_on_frame(frame, total);
    }
}

/* ---- 하트비트/부팅 알림 --------------------------------------------- */
static void heartbeat_task(void *arg)
{
    /* 부팅 알림(NodeAnnounce, CAN-Discovery). data[0]=node_id. */
    uint8_t nd = s_node_id;
    can_transport_send(RCUBE_PRI_SAFETY_SYNC, RCUBE_OP_NodeAnnounce, RCUBE_ADDR_BROADCAST, &nd, 1);

    TickType_t last = xTaskGetTickCount();
    uint32_t fail = 0;
    while (1) {
        esp_err_t err = can_transport_send(RCUBE_PRI_SAFETY_SYNC, RCUBE_OP_Heartbeat,
                                           RCUBE_ADDR_BROADCAST, &nd, 1);
        if (err != ESP_OK) {
            if ((fail++ % 10) == 0) {
                ESP_LOGW(TAG, "heartbeat TX 실패(%s) — 트랜시버/버스 확인(개발보드는 정상)",
                         esp_err_to_name(err));
            }
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(HEARTBEAT_MS));
    }
}

/* ---- 초기화 --------------------------------------------------------- */
static void gpio_out(gpio_num_t pin, int level)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(pin, level);
}

esp_err_t can_transport_init(uint8_t node_id, uint8_t term_id)
{
    s_node_id = node_id;
    s_term_id = term_id;

    /* 트랜시버 정상모드(STB=Low). */
    gpio_out(CAN_STB_GPIO, 0);
    /* 자가종단: 이 큐브가 CAN 버스 끝단(최대 CAN 노드ID == term_id)이면 종단 ON. */
    int terminate = (term_id != 0 && node_id == term_id) ? 1 : 0;
    gpio_out(CAN_TERM_EN_GPIO, terminate);
    ESP_LOGI(TAG, "STB=정상(Low), TERM_EN=%s (node=0x%02x, term=0x%02x)",
             terminate ? "ON(120Ω)" : "OFF", node_id, term_id);

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    g.tx_queue_len = 5;
    g.rx_queue_len = 10;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g, &t, &f);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install 실패: %s", esp_err_to_name(err));
        return err;
    }
    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_start 실패: %s", esp_err_to_name(err));
        twai_driver_uninstall();
        return err;
    }
    s_running = true;

    /* 명령 회신을 CAN으로 보내도록 응답 콜백 교체(이 큐브는 CAN 경로). */
    rcube_cmd_override_send(can_reply);

    xTaskCreatePinnedToCore(rx_task, "can_rx", 4096, NULL, 9, NULL, 0);
    xTaskCreatePinnedToCore(heartbeat_task, "can_hb", 3072, NULL, 5, NULL, 0);

    ESP_LOGI(TAG, "CAN transport ready (500kbit, TX%d RX%d, node=0x%02x)",
             CAN_TX_GPIO, CAN_RX_GPIO, node_id);
    return ESP_OK;
}
