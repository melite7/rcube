#include "rcube_cmd.h"
#include "rcube_config.h"
#include "rcube_status.h"
#include "rcube_sensor.h"
#include "board_led.h"

#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_sleep.h"

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
 * 상위(PC/허브)가 지정한 색 = 비고정형 단계의 "가상 노드ID 색"이다(기획서 5장).
 * 점멸/점등 여부는 rcube_status의 모드가 정하므로 여기서는 색만 넘긴다.
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
        rcube_status_set_color(0, 0, 0);   /* 노드LED 소등 */
        ESP_LOGI(TAG, "SetSK6812LED: n=0 → off");
        return RCUBE_RC_OK;
    }
    uint8_t r = p[1], g = p[2], b = p[3];   /* LED0 색을 대표로 사용(노드LED에 적용) */
    rcube_status_set_color(r, g, b);
    ESP_LOGI(TAG, "SetSK6812LED: n=%u, led0=(%u,%u,%u) → 지정색 적용", n, r, g, b);
    return RCUBE_RC_OK;
}

/* A0 SetMultiroleAggregator: payload = [ConnectionLinkCount][GroupMode] (+ flags).
 * 이 큐브를 BLE 허브(멀티롤)로 승격하고 멤버 스캔·연결을 시작한다.
 * 멤버가 붙을 때마다 멀티롤 레이어가 rcube_cmd_report_members()로 0xA1을 보낸다.
 *
 * 허브 LED(기획서 5장 [ID 표시용 칼라LED 점등 규칙] · 7.2-7):
 *   - 비고정형 초기구성(노드ID 미할당) → 가상1 = Red
 *   - 고정형 재연결(노드ID 저장됨)     → 자기 저장 노드ID 색 (무조건 빨강 아님)
 *   두 경우 모두 담당 멤버가 전원 연결되기 전엔 점멸(HUB_WAIT), 완료 시 상시 점등. */
static uint8_t handle_set_aggregator(const uint8_t *p, uint16_t len)
{
    if (len < 2) {
        return RCUBE_RC_BAD_LENGTH;
    }
    uint8_t link_count = p[0];
    uint8_t group_mode = p[1];

    uint8_t node_id = rcube_config_node_id();
    uint8_t r, g, b;
    if (node_id != 0) {
        rcube_status_node_color(node_id, &r, &g, &b);   /* 고정형: 자기 노드ID 색 */
    } else {
        r = 255; g = 0; b = 0;                          /* 비고정형: 가상1 = Red */
    }
    rcube_status_set_color(r, g, b);
    rcube_status_set_mode(RCUBE_LED_HUB_WAIT);
    ESP_LOGI(TAG, "SetMultiroleAggregator: total=%u, group_mode=0x%02x → BLE 허브 승격 "
                  "(node=0x%02x, %s, 멤버 대기 점멸)",
             link_count, group_mode, node_id, node_id ? "고정형/노드색" : "비고정형/Red");

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
static bool s_shutdown_scheduled;

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

/* D3 SetNodeConfig 서브커맨드 처리(payload[0]=subcmd). target은 브로드캐스트 여부 판단용. */
static uint8_t handle_set_node_config(uint8_t target, const uint8_t *p, uint16_t plen)
{
    if (plen < 1) {
        return RCUBE_RC_BAD_LENGTH;
    }
    uint8_t sub = p[0];
    switch (sub) {
    case RCUBE_D3_SUB_SET_GROUP: {
        if (plen < 2) return RCUBE_RC_BAD_LENGTH;
        if (rcube_config_set_group_id(p[1]) != ESP_OK) return RCUBE_RC_FLASH_FAIL;
        ESP_LOGI(TAG, "D3 SET_GROUP: group_id=0x%02x 저장 → 재부팅", p[1]);
        schedule_reboot();
        return RCUBE_RC_OK;
    }
    case RCUBE_D3_SUB_SET_NODE: {
        if (plen < 2) return RCUBE_RC_BAD_LENGTH;
        if (rcube_config_set_node_id(p[1]) != ESP_OK) return RCUBE_RC_FLASH_FAIL;
        ESP_LOGI(TAG, "D3 SET_NODE: node_id=0x%02x 저장 → 재부팅", p[1]);
        schedule_reboot();
        return RCUBE_RC_OK;
    }
    case RCUBE_D3_SUB_SET_NETCONF: {
        /* payload=[0x04, node_id, cmf, term_id]. 통신방식 세팅 저장(재부팅은 별도 REBOOT). */
        if (plen < 4) return RCUBE_RC_BAD_LENGTH;
        uint8_t node_id = p[1], cmf = p[2], term_id = p[3];
        esp_err_t e = rcube_config_set_node_id(node_id);
        if (e == ESP_OK) e = rcube_config_set_cmf(cmf);
        if (e == ESP_OK) e = rcube_config_set_term_id(term_id);
        if (e != ESP_OK) return RCUBE_RC_FLASH_FAIL;
        ESP_LOGI(TAG, "D3 SET_NETCONF: node=0x%02x cmf=%u(%s) term=0x%02x 저장(재부팅 대기)",
                 node_id, cmf, cmf ? "CAN" : "BLE", term_id);
        return RCUBE_RC_OK;
    }
    case RCUBE_D3_SUB_REBOOT: {
        ESP_LOGI(TAG, "D3 REBOOT: 재부팅 예약");
        schedule_reboot();
        return RCUBE_RC_OK;
    }
    default:
        ESP_LOGW(TAG, "D3 알 수 없는 subcmd 0x%02x", sub);
        return RCUBE_RC_BAD_PARAM;
    }
}

/* D4 GetNodeConfig 회신: payload = [group_id, node_id, cmf, term_id].
 * 멤버 큐브의 저장 결과를 PC가 확인할 수 있게 하는 관측 경로(기획서 7.2-3 검증용).
 * 멤버가 보낸 회신은 아그리게이터가 notify를 받아 PC로 중계한다(ble_multirole). */
static void reply_node_config(void)
{
    if (s_ops.send == NULL) {
        return;
    }
    uint8_t f[HEADER_LEN + 4];
    uint16_t total = sizeof(f);
    put_header(f, RCUBE_ADDR_HUB, RCUBE_OP_GetNodeConfig, total);
    f[4] = rcube_config_group_id();
    f[5] = rcube_config_node_id();
    f[6] = rcube_config_cmf();
    f[7] = rcube_config_term_id();
    s_ops.send(f, total);
    ESP_LOGI(TAG, "→ 0xD4 회신: group=0x%02x node=0x%02x cmf=%u term=0x%02x",
             f[4], f[5], f[6], f[7]);
}

/* ---- 센서 모니터링 (기획서 9장) -------------------------------------- */

/* B1 SetSensorStream: payload = [on] 또는 [on, period_hi, period_lo].
 * 브로드캐스트로 오면 상위(허브/edge central)가 전 멤버로 먼저 팬아웃한 뒤 자기도 적용. */
static uint8_t handle_set_sensor_stream(const uint8_t *p, uint16_t plen)
{
    if (plen < 1) {
        return RCUBE_RC_BAD_LENGTH;
    }
    bool on = (p[0] != 0);
    uint16_t period = (plen >= 3) ? (uint16_t)((p[1] << 8) | p[2])
                                  : RCUBE_SENSOR_PERIOD_DEFAULT_MS;
    if (!rcube_sensor_set_stream(on, period)) {
        return RCUBE_RC_BAD_PARAM;
    }
    return RCUBE_RC_OK;
}

int rcube_cmd_send_frame(const uint8_t *frame, uint16_t len)
{
    if (s_ops.send == NULL) {
        return -1;
    }
    return s_ops.send(frame, len);
}

int rcube_cmd_sensor_stream_all(bool on, uint16_t period_ms)
{
    /* 자기 자신부터 적용(허브/edge central도 유닛의 한 큐브다 — 9장 "자신의 데이터와 함께"). */
    rcube_sensor_set_stream(on, period_ms);

    uint8_t f[HEADER_LEN + 3];
    uint16_t total = sizeof(f);
    put_header(f, RCUBE_ADDR_HUB, RCUBE_OP_SetSensorStream, total);
    f[4] = on ? 1 : 0;
    f[5] = (uint8_t)((period_ms >> 8) & 0xFF);
    f[6] = (uint8_t)(period_ms & 0xFF);

    int sent = (s_ops.forward_all != NULL) ? s_ops.forward_all(f, total) : 0;
    ESP_LOGI(TAG, "센서 전송 %s 지시: 자신 + BLE 멤버 %d대 (주기 %u ms)",
             on ? "시작" : "중지", sent < 0 ? 0 : sent, period_ms);
    return sent;
}

/* D5 SetEdgeCentralConfig: 독립로봇유닛 전환/강등 (기획서 7.3-3 ★보강).
 *   payload = [ecf, unit_n, term_id, map[8]]  (총 11바이트)
 *     ecf     : 1=edge central(리드 큐브), 0=강등(일반 큐브 — 맵도 삭제)
 *     unit_n  : 유닛 전체 큐브 수 N
 *     term_id : CAN 세팅 멤버 중 최대 노드ID(자가 종단 판단용)
 *     map[i]  : 노드ID(i+1)의 통신방식 — 0=BLE, 1=CAN, 0xFF=유닛에 없음
 * 저장만 하고 재부팅하지 않는다. 배선 정리를 위해 이어서 E7(shutdown)을 받는다. */
#define D5_PAYLOAD_LEN (3 + RCUBE_MAX_NODES)

static uint8_t handle_set_edge_config(const uint8_t *p, uint16_t plen)
{
    if (plen < D5_PAYLOAD_LEN) {
        ESP_LOGW(TAG, "D5: payload %u bytes (필요 %u)", plen, D5_PAYLOAD_LEN);
        return RCUBE_RC_BAD_LENGTH;
    }
    uint8_t ecf = p[0], unit_n = p[1], term_id = p[2];
    const uint8_t *map = &p[3];

    if (ecf && (unit_n < 1 || unit_n > RCUBE_MAX_NODES)) {
        ESP_LOGW(TAG, "D5: unit_n=%u 범위 초과(1~%u)", unit_n, RCUBE_MAX_NODES);
        return RCUBE_RC_BAD_PARAM;
    }
    /* 종단노드ID는 CAN 큐브만 의미가 있다(7.2-2 ★). edge central도 자기 맵으로 판단. */
    esp_err_t e = rcube_config_set_term_id(term_id);
    if (e == ESP_OK) e = rcube_config_set_edge(ecf, unit_n, map);
    if (e != ESP_OK) {
        return RCUBE_RC_FLASH_FAIL;
    }
    ESP_LOGI(TAG, "D5 SetEdgeCentralConfig: ecf=%u unit_n=%u term=0x%02x 저장(재부팅 대기)",
             ecf, unit_n, term_id);
    return RCUBE_RC_OK;
}

/* D6 GetEdgeCentralConfig 회신: D5와 같은 레이아웃으로 현재 저장값을 돌려준다. */
static void reply_edge_config(void)
{
    if (s_ops.send == NULL) {
        return;
    }
    uint8_t f[HEADER_LEN + D5_PAYLOAD_LEN];
    uint16_t total = sizeof(f);
    put_header(f, RCUBE_ADDR_HUB, RCUBE_OP_GetEdgeCentralConfig, total);
    f[4] = rcube_config_ecf();
    f[5] = rcube_config_unit_count();
    f[6] = rcube_config_term_id();
    memcpy(&f[7], rcube_config_member_map(), RCUBE_MAX_NODES);
    s_ops.send(f, total);
    ESP_LOGI(TAG, "→ 0xD6 회신: ecf=%u unit_n=%u term=0x%02x", f[4], f[5], f[6]);
}

/* E7 SetPowerState: payload=[state]. 0=shut down(전원 끄기), 그 외=무시.
 * 기획서 7.3-4 "저장이 완료되면 PC는 모든 R큐브에 shut down 명령을 주어 모두 끈다."
 * 개발보드에는 전원 차단 회로가 없으므로 딥슬립으로 대신한다(버튼으로 깨어남).
 * 메인보드에서는 STM6601 전원 IC 제어로 교체해야 한다. */
#define SHUTDOWN_DELAY_MS 600
/* 딥슬립에서 깨울 핀 = BOOT 버튼(main.c의 BOOT_BTN_GPIO와 같은 핀). RTC IO여야 한다. */
#define RCUBE_WAKE_GPIO 0

static void shutdown_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(SHUTDOWN_DELAY_MS));   /* 회신·중계가 나갈 시간 */
    board_led_set_all(0, 0, 0);
    ESP_LOGW(TAG, "shutdown: 딥슬립 진입(BOOT 버튼으로 깨어남). "
                  "메인보드에서는 STM6601 전원 차단으로 교체 필요.");
    esp_sleep_enable_ext1_wakeup(1ULL << RCUBE_WAKE_GPIO, ESP_EXT1_WAKEUP_ANY_LOW);
    esp_deep_sleep_start();
}

static uint8_t handle_set_power_state(const uint8_t *p, uint16_t plen)
{
    if (plen < 1) {
        return RCUBE_RC_BAD_LENGTH;
    }
    if (p[0] != 0) {
        ESP_LOGW(TAG, "E7: state=%u 미지원(0=shutdown만)", p[0]);
        return RCUBE_RC_BAD_PARAM;
    }
    if (s_shutdown_scheduled) {
        return RCUBE_RC_OK;
    }
    s_shutdown_scheduled = true;
    ESP_LOGW(TAG, "E7 SetPowerState: shut down 예약(%d ms 후)", SHUTDOWN_DELAY_MS);
    xTaskCreate(shutdown_task, "shutdown", 2560, NULL, 5, NULL);
    return RCUBE_RC_OK;
}

/* D7 ResetConfig: 공장 초기화(노드ID=0, CMF=BLE, 종단ID=0) 후 재부팅.
 * 기획서 7.3 [노드ID/세팅 초기화 - 공통]. 브로드캐스트면 전 멤버로 팬아웃된다. */
static uint8_t handle_reset_config(void)
{
    if (rcube_config_reset_factory() != ESP_OK) {
        return RCUBE_RC_FLASH_FAIL;
    }
    ESP_LOGW(TAG, "D7 ResetConfig: 공장 초기화 완료 → 재부팅(비고정형으로 복귀)");
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
        /* 브로드캐스트(예: SET_GROUP)면 아그리게이터가 먼저 전 멤버로 그대로 중계. */
        if (target == RCUBE_ADDR_BROADCAST && s_ops.forward_all != NULL) {
            s_ops.forward_all(data, len);
        }
        rc = handle_set_node_config(target, payload, plen);
        reply_cmd_ack(op, rc);
        break;

    case RCUBE_OP_GetNodeConfig:
        reply_node_config();   /* 회신 자체가 응답이므로 CmdAck 생략 */
        break;

    case RCUBE_OP_ResetConfig:
        if (target == RCUBE_ADDR_BROADCAST && s_ops.forward_all != NULL) {
            s_ops.forward_all(data, len);   /* 전 멤버도 초기화 */
        }
        rc = handle_reset_config();
        reply_cmd_ack(op, rc);
        break;

    case RCUBE_OP_GetSensors:
        rcube_sensor_report_once();   /* 회신 자체가 응답 */
        break;

    case RCUBE_OP_SetSensorStream:
        /* 브로드캐스트면 허브/edge central이 먼저 전 멤버로 중계한 뒤 자기도 적용. */
        if (target == RCUBE_ADDR_BROADCAST && s_ops.forward_all != NULL) {
            s_ops.forward_all(data, len);
        }
        rc = handle_set_sensor_stream(payload, plen);
        reply_cmd_ack(op, rc);
        break;

    case RCUBE_OP_SetEdgeCentralConfig:
        rc = handle_set_edge_config(payload, plen);
        reply_cmd_ack(op, rc);
        break;

    case RCUBE_OP_GetEdgeCentralConfig:
        reply_edge_config();   /* 회신 자체가 응답 */
        break;

    case RCUBE_OP_SetPowerState:
        if (target == RCUBE_ADDR_BROADCAST && s_ops.forward_all != NULL) {
            s_ops.forward_all(data, len);   /* 전 멤버도 함께 끈다(7.3-4) */
        }
        rc = handle_set_power_state(payload, plen);
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

void rcube_cmd_override_send(rcube_send_fn send)
{
    s_ops.send = send;
    ESP_LOGI(TAG, "응답 콜백 교체(CAN 회신 경로)");
}
