#include "rcube_cmd.h"
#include "board_led.h"

#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

/* opcode/주소/결과코드는 shared-protocol(단일 소스)에서 그대로 가져온다. */
#include "rcube_protocol.h"

static const char *TAG = "cmd";

#define HEADER_LEN 4

/* 회신(notify) 송신 콜백. */
static rcube_send_fn s_send;

/* ---- 아그리게이터 상태(A0 이후) ----
 * Phase 2에서는 상태만 보관하고 실제 멤버 BLE 연결은 하지 않는다(Phase 5). */
static SemaphoreHandle_t s_lock;
static bool    s_is_aggregator;      /* A0로 아그리게이터 승격됨 */
static uint8_t s_link_count;         /* 목표 큐브 수(본인 포함 총 N) */
static uint8_t s_group_mode;         /* 0x0A=그룹무관, 0x1A=같은그룹 */
static uint8_t s_members_connected;  /* 현재 연결된 멤버 수(본인 제외) */

/* 이 큐브가 직접 처리해야 하는 대상 주소인지(직접연결 허브 · 브로드캐스트). */
static inline bool addr_for_me(uint8_t target)
{
    return target == RCUBE_ADDR_HUB || target == RCUBE_ADDR_BROADCAST;
}

/* PacketSize(BE)를 프레임 앞 4바이트에 채운다. */
static inline void put_header(uint8_t *f, uint8_t target, uint8_t op, uint16_t total)
{
    f[0] = target;
    f[1] = op;
    f[2] = (uint8_t)((total >> 8) & 0xFF);
    f[3] = (uint8_t)(total & 0xFF);
}

/* ---- 회신 프레임 ----------------------------------------------------- */

/* CmdAck(0xAF) 회신: payload = [요청 OpCode][ResultCode].
 * ※ CmdAck payload 세부 레이아웃은 xlsx 원본에만 있음 → 이 [op][rc] 2바이트는
 *   펌웨어가 먼저 정한 계약(app과 맞춰야 함, 미검증). 발신=자기(허브). */
static void reply_cmd_ack(uint8_t req_op, uint8_t result)
{
    if (s_send == NULL) {
        return;
    }
    uint8_t f[HEADER_LEN + 2];
    uint16_t total = sizeof(f);
    put_header(f, RCUBE_ADDR_HUB, RCUBE_OP_CmdAck, total);
    f[4] = req_op;
    f[5] = result;
    s_send(f, total);
}

/* SetMultiroleInAction(0xA1) 회신: payload[0]=현재 연결된 멤버 수.
 * app(gui.py)이 이 값을 보고 R2~R4 진행도를 갱신한다(계약). */
static void reply_multirole_event(uint8_t members_connected)
{
    if (s_send == NULL) {
        return;
    }
    uint8_t f[HEADER_LEN + 1];
    uint16_t total = sizeof(f);
    put_header(f, RCUBE_ADDR_HUB, RCUBE_OP_SetMultiroleInAction, total);
    f[4] = members_connected;
    s_send(f, total);
}

/* ---- 개별 명령 처리 -------------------------------------------------- */

/* E0 SetSK6812LED: payload = [n][R,G,B]×n.
 * 온보드 LED는 1개뿐이므로 LED0 색을 대표로 점등하고 전체 세트는 로깅한다. */
static uint8_t handle_set_led(uint8_t target, const uint8_t *p, uint16_t len)
{
    if (!addr_for_me(target)) {
        /* 멤버(가상노드ID) 대상 → 아그리게이터가 하위로 중계(Phase 5). */
        ESP_LOGW(TAG, "SetSK6812LED for node 0x%02x — forward not implemented (Phase 5)", target);
        return RCUBE_RC_NODE_NOT_FOUND;
    }
    if (len < 1) {
        return RCUBE_RC_BAD_LENGTH;
    }
    uint8_t n = p[0];
    if ((uint16_t)1 + (uint16_t)n * 3 > len) {
        ESP_LOGW(TAG, "SetSK6812LED: n=%u 인데 payload %u bytes (부족)", n, len);
        return RCUBE_RC_BAD_LENGTH;
    }
    if (n == 0) {
        board_led_set(0, 0, 0);   /* 소등 */
        ESP_LOGI(TAG, "SetSK6812LED: n=0 → off");
        return RCUBE_RC_OK;
    }
    uint8_t r = p[1], g = p[2], b = p[3];   /* LED0 색을 대표로 사용 */
    board_led_set(r, g, b);
    ESP_LOGI(TAG, "SetSK6812LED: n=%u, led0=(%u,%u,%u) → 온보드 점등", n, r, g, b);
    return RCUBE_RC_OK;
}

/* A0 SetMultiroleAggregator: payload = [ConnectionLinkCount][GroupMode] (+ vids).
 * 이 큐브를 아그리게이터로 승격, 자기 RED 점등 후 0xA1로 회신.
 * 실제 멤버 스캔·연결(BLE central)은 Phase 5. */
static uint8_t handle_set_aggregator(uint8_t target, const uint8_t *p, uint16_t len)
{
    if (!addr_for_me(target)) {
        return RCUBE_RC_NODE_NOT_FOUND;
    }
    if (len < 2) {
        return RCUBE_RC_BAD_LENGTH;
    }
    uint8_t link_count = p[0];
    uint8_t group_mode = p[1];

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_is_aggregator = true;
    s_link_count = link_count;
    s_group_mode = group_mode;
    s_members_connected = 0;
    uint8_t members = s_members_connected;
    xSemaphoreGive(s_lock);

    board_led_set(255, 0, 0);   /* 아그리게이터 표시 = 빨강(순수색, 밝기는 board_led) */
    ESP_LOGI(TAG, "SetMultiroleAggregator: total=%u, group_mode=0x%02x → 아그리게이터 승격(RED)",
             link_count, group_mode);
    ESP_LOGW(TAG, "멤버 BLE 스캔·연결은 Phase 5 미구현 → 0xA1(members=%u)만 회신", members);

    /* 아그리게이터 승격 사실을 0xA1로 알린다(현재 멤버 수 = 0). */
    reply_multirole_event(members);
    return RCUBE_RC_OK;
}

/* ---- 파서 + 디스패처 ------------------------------------------------- */
void rcube_cmd_on_frame(const uint8_t *data, uint16_t len)
{
    if (len < HEADER_LEN) {
        ESP_LOGW(TAG, "프레임이 헤더보다 짧음: %u bytes", len);
        return;
    }
    uint8_t  target   = data[0];
    uint8_t  op       = data[1];
    uint16_t declared = ((uint16_t)data[2] << 8) | data[3];
    const uint8_t *payload = data + HEADER_LEN;
    uint16_t plen = len - HEADER_LEN;

    /* PacketSize 는 검증만 하고, 실제 수신 길이를 신뢰한다(BLE 조각화 방어). */
    if (declared != 0 && declared != len) {
        ESP_LOGW(TAG, "PacketSize 불일치: 선언=%u, 실제=%u (실제 길이로 처리)", declared, len);
    }

    ESP_LOGI(TAG, "RX frame: target=0x%02x op=0x%02x payload=%u bytes", target, op, plen);

    uint8_t rc;
    switch (op) {
    case RCUBE_OP_SetSK6812LED:
        rc = handle_set_led(target, payload, plen);
        reply_cmd_ack(op, rc);
        break;

    case RCUBE_OP_SetMultiroleAggregator:
        rc = handle_set_aggregator(target, payload, plen);
        /* A0는 0xA1로 회신하므로 CmdAck는 실패 시에만 보낸다. */
        if (rc != RCUBE_RC_OK) {
            reply_cmd_ack(op, rc);
        }
        break;

    default:
        ESP_LOGW(TAG, "미구현 OpCode 0x%02x (Phase 2 미지원)", op);
        reply_cmd_ack(op, RCUBE_RC_BAD_OPCODE);
        break;
    }
}

void rcube_cmd_init(rcube_send_fn responder)
{
    s_send = responder;
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        configASSERT(s_lock != NULL);
    }
    ESP_LOGI(TAG, "command layer ready (E0/A0 구현, 그 외 CmdAck NAK)");
}
