#include "rcube_cmd.h"
#include "board_led.h"
#include "rcube_config.h"

#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

/* opcode/주소/결과코드는 shared-protocol(단일 소스)에서 그대로 가져온다. */
#include "rcube_protocol.h"

static const char *TAG = "cmd";

#define HEADER_LEN 4

/* 전송계층 위임 콜백. */
static rcube_cmd_ops_t s_ops;

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
    if (s_ops.send == NULL) {
        return;
    }
    uint8_t f[HEADER_LEN + 2];
    uint16_t total = sizeof(f);
    put_header(f, RCUBE_ADDR_HUB, RCUBE_OP_CmdAck, total);
    f[4] = req_op;
    f[5] = result;
    s_ops.send(f, total);
}

/* SetMultiroleInAction(0xA1) 회신: payload[0]=현재 연결된 멤버 수(본인 제외).
 * app(gui.py)이 이 값을 보고 R2~R4 진행도를 갱신한다(계약). */
void rcube_cmd_report_members(uint8_t members_connected)
{
    if (s_ops.send == NULL) {
        return;
    }
    uint8_t f[HEADER_LEN + 1];
    uint16_t total = sizeof(f);
    put_header(f, RCUBE_ADDR_HUB, RCUBE_OP_SetMultiroleInAction, total);
    f[4] = members_connected;
    s_ops.send(f, total);
    ESP_LOGI(TAG, "→ 0xA1 report: members=%u", members_connected);
}

/* ---- 개별 명령 처리 -------------------------------------------------- */

/* E0 SetSK6812LED: payload = [n][R,G,B]×n.
 * 온보드 LED는 1개뿐이므로 LED0 색을 대표로 점등하고 전체 세트는 로깅한다.
 * (자기 대상 프레임에서만 호출된다 — 멤버 대상은 상위에서 중계 처리) */
static uint8_t handle_set_led(const uint8_t *p, uint16_t len)
{
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
 * 이 큐브를 아그리게이터로 승격, 자기 RED 점등 후 멤버 스캔·연결을 시작한다.
 * 멤버가 붙을 때마다 멀티롤 레이어가 rcube_cmd_report_members()로 0xA1을 보낸다. */
static uint8_t handle_set_aggregator(const uint8_t *p, uint16_t len)
{
    if (len < 2) {
        return RCUBE_RC_BAD_LENGTH;
    }
    uint8_t link_count = p[0];
    uint8_t group_mode = p[1];

    board_led_set(255, 0, 0);   /* 아그리게이터 표시 = 빨강(순수색, 밝기는 board_led) */
    ESP_LOGI(TAG, "SetMultiroleAggregator: total=%u, group_mode=0x%02x → 아그리게이터 승격(RED)",
             link_count, group_mode);

    if (s_ops.agg_start != NULL) {
        s_ops.agg_start(link_count, group_mode);   /* 멤버 스캔·연결 시작(비동기) */
    } else {
        ESP_LOGW(TAG, "agg_start 미등록 → 멤버 연결 불가, 0xA1(0)만 회신");
        rcube_cmd_report_members(0);
    }
    return RCUBE_RC_OK;
}

/* ---- 지연 재부팅 ----------------------------------------------------- */
/* 회신(notify)·멤버 중계 write가 나갈 시간을 준 뒤 재부팅한다. */
#define REBOOT_DELAY_MS 800

static bool s_reboot_scheduled;

static void reboot_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(REBOOT_DELAY_MS));
    ESP_LOGW(TAG, "재부팅 실행(esp_restart)");
    esp_restart();
}

static void schedule_reboot(void)
{
    if (s_reboot_scheduled) {
        return;
    }
    s_reboot_scheduled = true;
    ESP_LOGW(TAG, "설정 적용 → %d ms 후 재부팅 예약", REBOOT_DELAY_MS);
    xTaskCreate(reboot_task, "reboot", 2048, NULL, 5, NULL);
}

/* D3 SetNodeConfig: payload=[group_id]. 그룹번호를 NVS에 저장하고 재부팅.
 * (계약: 지금은 group_id 1바이트. 추후 node_id 등 필드 확장 가능.) */
static uint8_t handle_set_node_config(const uint8_t *p, uint16_t len)
{
    if (len < 1) {
        return RCUBE_RC_BAD_LENGTH;
    }
    uint8_t group = p[0];
    esp_err_t err = rcube_config_set_group_id(group);
    if (err != ESP_OK) {
        return RCUBE_RC_FLASH_FAIL;
    }
    ESP_LOGI(TAG, "SetNodeConfig: group_id=0x%02x 저장", group);
    schedule_reboot();
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

    /* 자기 대상이 아니면 → 아그리게이터일 때 멤버로 중계. */
    if (!addr_for_me(target)) {
        if (s_ops.forward != NULL && s_ops.forward(target, data, len) == 0) {
            ESP_LOGI(TAG, "target 0x%02x → 멤버로 중계", target);
        } else {
            ESP_LOGW(TAG, "target 0x%02x 중계 불가(아그리게이터 아님/대상 없음)", target);
            reply_cmd_ack(op, RCUBE_RC_NODE_NOT_FOUND);
        }
        return;
    }

    uint8_t rc;
    switch (op) {
    case RCUBE_OP_SetSK6812LED:
        rc = handle_set_led(payload, plen);
        reply_cmd_ack(op, rc);
        break;

    case RCUBE_OP_SetMultiroleAggregator:
        rc = handle_set_aggregator(payload, plen);
        /* 성공 시 회신은 0xA1(멤버 연결)로 대체. 실패만 CmdAck. */
        if (rc != RCUBE_RC_OK) {
            reply_cmd_ack(op, rc);
        }
        break;

    case RCUBE_OP_SetNodeConfig:
        /* 브로드캐스트면 아그리게이터가 먼저 전 멤버로 중계(멤버도 저장+재부팅). */
        if (target == RCUBE_ADDR_BROADCAST && s_ops.forward_all != NULL) {
            s_ops.forward_all(data, len);
        }
        rc = handle_set_node_config(payload, plen);
        reply_cmd_ack(op, rc);
        break;

    default:
        ESP_LOGW(TAG, "미구현 OpCode 0x%02x (미지원)", op);
        reply_cmd_ack(op, RCUBE_RC_BAD_OPCODE);
        break;
    }
}

void rcube_cmd_init(const rcube_cmd_ops_t *ops)
{
    if (ops != NULL) {
        s_ops = *ops;
    } else {
        memset(&s_ops, 0, sizeof(s_ops));
    }
    ESP_LOGI(TAG, "command layer ready (E0/A0 + 멤버 중계, 그 외 CmdAck NAK)");
}
