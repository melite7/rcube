#include "rcube_motion.h"
#include "motion_core.h"
#include "motor_uart.h"
#include "rcube_cmd.h"
#include "rcube_params.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "rcube_protocol.h"

static const char *TAG = "rmotion";

#define HEADER_LEN 4

/* 웨이포인트 순번. 상위가 지정하지 않는 단발 명령(C0/C1/C8)에 자동 부여한다. */
static uint16_t s_seq;

/* ---- 바이트 도우미 (와이어는 big-endian) ---- */
static int32_t rd_i32(const uint8_t *p)
{
    return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                     ((uint32_t)p[2] << 8) | p[3]);
}
static uint16_t rd_u16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static int16_t  rd_i16(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }

static void wr_i32(uint8_t *p, int32_t v)
{
    p[0] = (uint8_t)((v >> 24) & 0xFF); p[1] = (uint8_t)((v >> 16) & 0xFF);
    p[2] = (uint8_t)((v >> 8) & 0xFF);  p[3] = (uint8_t)(v & 0xFF);
}
static void wr_i16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)((v >> 8) & 0xFF); p[1] = (uint8_t)(v & 0xFF);
}

/* ★ 3단 안전 1단(기획서 8장): 각도 소프트리밋 밖 목표는 큐브가 거부한다.
 * 상위가 잘못 계산한 목표로 기구를 밀어붙이는 것을 여기서 끊는다. */
static bool angle_ok(int32_t position)
{
    if (rcube_params_angle_ok(position)) {
        return true;
    }
    const rcube_params_t *p = rcube_params();
    ESP_LOGE(TAG, "각도 소프트리밋 위반: 목표 %.2f° (허용 %.2f~%.2f°) → 거부",
             position / 100.0f, p->angle_min / 100.0f, p->angle_max / 100.0f);
    return false;
}

static bool push_one(int32_t position, uint16_t t_ms, int16_t velocity, uint8_t mode)
{
    motion_waypoint_t wp = {
        .t_offset_us = (uint32_t)t_ms * 1000u,
        .position = position,
        .velocity = velocity,
        /* 단발 명령은 도달 시 완료를 알린다 — 미션코드의 wait_done()이 여기에 걸린다. */
        .flags = MOTION_WP_FLAG_INPOS,
        .seq = s_seq++,
    };
    return motion_core_push(&wp, mode == 0x01 /* Buffered */);
}

/* ---- C1 SetSingleAngle: [mode][pos i32][t_ms u16] ---- */
static uint8_t handle_single_angle(const uint8_t *p, uint16_t len)
{
    if (len < 7) return RCUBE_RC_BAD_LENGTH;
    uint8_t mode = p[0];
    int32_t pos = rd_i32(&p[1]);
    uint16_t t_ms = rd_u16(&p[5]);
    if (!angle_ok(pos)) return RCUBE_RC_ANGLE_LIMIT;
    if (!push_one(pos, t_ms, 0, mode)) return RCUBE_RC_BUFFER_FULL;
    ESP_LOGI(TAG, "C1 SetSingleAngle: %.2f° in %u ms (%s)",
             pos / 100.0f, t_ms, mode ? "Buffered" : "Immediate");
    return RCUBE_RC_OK;
}

/* ---- C0 SetSingleSpeed: [mode][vel i32] ----
 * 속도 지령은 목표 위치가 없으므로 큐를 거치지 않고 드라이버 속도 루프로 바로 간다. */
static uint8_t handle_single_speed(const uint8_t *p, uint16_t len)
{
    if (len < 5) return RCUBE_RC_BAD_LENGTH;
    int32_t vel_centi_dps = rd_i32(&p[1]);
    /* 0.01°/s → ERPM 변환은 극쌍수·감속비 확정 전까지 보류(확장 규격 §6-2).
     * 지금은 값을 그대로 ERPM으로 넘겨 실보드에서 스케일을 잡을 수 있게 한다. */
    if (motor_uart_set_rpm(vel_centi_dps / 100) != ESP_OK) return RCUBE_RC_BAD_STATE;
    ESP_LOGW(TAG, "C0 SetSingleSpeed: %.2f°/s → ERPM 환산 미확정, 임시로 %ld 전송",
             vel_centi_dps / 100.0f, (long)(vel_centi_dps / 100));
    return RCUBE_RC_OK;
}

/* ---- C2 SetScheduledAngles: [mode][seq_base u16][count][wp × count] ----
 * 웨이포인트 12바이트: t_offset_us(u32) pos(i32) vel(i16) flags(1) rsvd(1) */
#define WP_WIRE_LEN 12

static uint8_t handle_scheduled(const uint8_t *p, uint16_t len)
{
    if (len < 4) return RCUBE_RC_BAD_LENGTH;
    uint8_t mode = p[0];
    uint16_t seq_base = rd_u16(&p[1]);
    uint8_t count = p[3];
    if ((uint32_t)4 + (uint32_t)count * WP_WIRE_LEN > len) {
        ESP_LOGW(TAG, "C2: count=%u 인데 payload %u bytes (부족)", count, len);
        return RCUBE_RC_BAD_LENGTH;
    }
    uint8_t pushed = 0;
    for (uint8_t i = 0; i < count; i++) {
        const uint8_t *w = &p[4 + i * WP_WIRE_LEN];
        if (!angle_ok(rd_i32(&w[4]))) {
            return RCUBE_RC_ANGLE_LIMIT;   /* 한 점이라도 밖이면 묶음 전체를 거부 */
        }
        motion_waypoint_t wp = {
            .t_offset_us = (uint32_t)rd_i32(w),
            .position = rd_i32(&w[4]),
            .velocity = rd_i16(&w[8]),
            .flags = w[10],
            .seq = (uint16_t)(seq_base + i),
        };
        if (!motion_core_push(&wp, mode == 0x01)) {
            break;   /* 큐가 찼다 — 받은 만큼만 적재하고 알린다 */
        }
        pushed++;
    }
    ESP_LOGI(TAG, "C2 SetScheduledAngles: %u/%u개 적재(seq %u~, %s)",
             pushed, count, seq_base, mode ? "Buffered" : "Immediate");
    return (pushed == count) ? RCUBE_RC_OK : RCUBE_RC_BUFFER_FULL;
}

/* ---- C7 ExecuteBuffer: [action][t_start_us u64] ---- */
static uint8_t handle_execute(const uint8_t *p, uint16_t len)
{
    if (len < 1) return RCUBE_RC_BAD_LENGTH;
    uint8_t action = p[0];
    uint64_t t0 = 0;
    if (len >= 9) {
        for (int i = 0; i < 8; i++) {
            t0 = (t0 << 8) | p[1 + i];
        }
    }
    return motion_core_execute(action, t0) ? RCUBE_RC_OK : RCUBE_RC_BAD_PARAM;
}

/* ---- C8 MoveToOrigin: [mode][t_ms u16] ---- */
static uint8_t handle_move_origin(const uint8_t *p, uint16_t len)
{
    if (len < 3) return RCUBE_RC_BAD_LENGTH;
    uint16_t t_ms = rd_u16(&p[1]);
    if (!push_one(0, t_ms, 0, p[0])) return RCUBE_RC_BUFFER_FULL;
    ESP_LOGI(TAG, "C8 MoveToOrigin: 0° in %u ms", t_ms);
    return RCUBE_RC_OK;
}

/* ---- CB SetDriveState: [state] ---- */
static uint8_t handle_drive_state(const uint8_t *p, uint16_t len)
{
    if (len < 1) return RCUBE_RC_BAD_LENGTH;
    switch (p[0]) {
    case 0x00:   /* Disable */
        motor_uart_safe_stop();
        break;
    case 0x01:   /* Enable */
        motor_uart_set_gate(true);
        break;
    case 0x02:   /* QuickStop */
        motion_core_emergency_stop("SetDriveState QuickStop");
        break;
    case 0x03:   /* FaultReset — 차단 후 재인에이블 */
        motor_uart_set_gate(false);
        motor_uart_set_gate(true);
        break;
    default:
        return RCUBE_RC_BAD_PARAM;
    }
    ESP_LOGI(TAG, "CB SetDriveState: state=0x%02x (게이트 %s)",
             p[0], motor_uart_gate_enabled() ? "ON" : "OFF");
    return RCUBE_RC_OK;
}

/* ---- 파라미터·리밋 (확장 규격 §2.12) ---------------------------------
 * 적용은 즉시, NVS 저장은 DB(SaveParameters)를 받아야 한다. 튜닝 중 잘못된 값을
 * 저장해 다음 부팅부터 못 쓰게 되는 상황을 막기 위해 둘을 분리한다. */

static uint8_t handle_angle_limits(const uint8_t *p, uint16_t len)
{
    if (len < 8) return RCUBE_RC_BAD_LENGTH;
    int32_t lo = rd_i32(&p[0]), hi = rd_i32(&p[4]);
    if (!(lo == 0 && hi == 0) && lo >= hi) {
        ESP_LOGW(TAG, "D8: min(%.2f) >= max(%.2f)", lo / 100.0f, hi / 100.0f);
        return RCUBE_RC_BAD_PARAM;
    }
    rcube_params_set_angle_limits(lo, hi);
    return RCUBE_RC_OK;
}

static uint8_t handle_motion_limits(const uint8_t *p, uint16_t len)
{
    if (len < 6) return RCUBE_RC_BAD_LENGTH;
    rcube_params_set_motion_limits(rd_u16(&p[0]), rd_u16(&p[2]), rd_u16(&p[4]));
    return RCUBE_RC_OK;
}

static uint8_t handle_fault_thresholds(const uint8_t *p, uint16_t len)
{
    if (len < 5) return RCUBE_RC_BAD_LENGTH;
    rcube_params_set_fault_thresholds(rd_u16(&p[0]), rd_u16(&p[2]), p[4]);
    return RCUBE_RC_OK;
}

/* ---- D2 TimeSync (확장 규격 §3.2) ------------------------------------
 * 시각 값은 48비트 마이크로초(6바이트). 64비트로 잡으면 요청이 9바이트가 되어
 * CAN 단일 프레임(8B)을 넘긴다.
 *
 * 다축 동기의 정확도가 여기서 결정된다 — 드라이버에는 예약 실행이 없으므로
 * ExecuteBuffer(Run, T0)의 T0를 각 큐브가 "자기 시계로" 기다린다(§3.1). */
#define TS_FLAG_ROUNDTRIP 0x01u   /* 왕복 측정 요청 */
#define TS_FLAG_OFFSET    0x02u   /* value가 절대시각이 아니라 오프셋 보정값 */
#define TS_FLAG_REPLY     0x80u

static void wr_u48(uint8_t *p, int64_t v)
{
    p[0] = (uint8_t)((v >> 40) & 0xFF); p[1] = (uint8_t)((v >> 32) & 0xFF);
    p[2] = (uint8_t)((v >> 24) & 0xFF); p[3] = (uint8_t)((v >> 16) & 0xFF);
    p[4] = (uint8_t)((v >> 8) & 0xFF);  p[5] = (uint8_t)(v & 0xFF);
}

/* 부호 있는 48비트로 해석(오프셋은 음수일 수 있다). */
static int64_t rd_i48(const uint8_t *p)
{
    uint64_t v = ((uint64_t)p[0] << 40) | ((uint64_t)p[1] << 32) | ((uint64_t)p[2] << 24) |
                 ((uint64_t)p[3] << 16) | ((uint64_t)p[4] << 8) | p[5];
    if (v & 0x0000800000000000ull) {
        v |= 0xFFFF000000000000ull;   /* 부호 확장 */
    }
    return (int64_t)v;
}

static uint8_t handle_timesync(const uint8_t *p, uint16_t len)
{
    /* 수신 시각을 가장 먼저 찍는다 — 뒤의 처리 시간이 측정에 섞이지 않게. */
    int64_t t_recv = esp_timer_get_time();

    if (len < 7) return RCUBE_RC_BAD_LENGTH;
    uint8_t flags = p[0];
    int64_t value = rd_i48(&p[1]);

    if (flags & TS_FLAG_OFFSET) {
        /* 마스터가 왕복 측정으로 구한 보정값을 그대로 적용한다. */
        motion_core_set_time_offset(value);
    } else if (!(flags & TS_FLAG_ROUNDTRIP)) {
        /* 단방향 배포: 편도 지연만큼 오차가 남는다(정밀도 필요 없을 때만). */
        motion_core_set_time_offset(value - t_recv);
    }

    if (flags & TS_FLAG_ROUNDTRIP) {
        uint8_t f[HEADER_LEN + 19];
        uint16_t total = sizeof(f);
        f[0] = RCUBE_ADDR_HUB;
        f[1] = RCUBE_OP_TimeSync;
        f[2] = (uint8_t)((total >> 8) & 0xFF);
        f[3] = (uint8_t)(total & 0xFF);
        f[4] = (uint8_t)(flags | TS_FLAG_REPLY);
        wr_u48(&f[5], value);            /* 마스터가 보낸 T1 그대로 되돌린다 */
        wr_u48(&f[11], t_recv);
        wr_u48(&f[17], esp_timer_get_time());   /* 회신 직전 시각 */
        rcube_cmd_send_frame(f, total);
    }
    return RCUBE_RC_OK;
}

/* ---- 조회 회신 ------------------------------------------------------- */

void rcube_motion_report_status(void)
{
    motor_status_t st;
    motor_uart_get_status(&st);

    uint8_t f[HEADER_LEN + 7];
    uint16_t total = sizeof(f);
    f[0] = RCUBE_ADDR_HUB;
    f[1] = RCUBE_OP_GetMotorStatus;
    f[2] = (uint8_t)((total >> 8) & 0xFF);
    f[3] = (uint8_t)(total & 0xFF);
    f[4] = st.error_code;
    wr_i32(&f[5], (int32_t)(st.position_deg * 100.0f));
    wr_i16(&f[9], (int16_t)(st.current_a * 100.0f));
    rcube_cmd_send_frame(f, total);
    ESP_LOGI(TAG, "→ 0xB2: err=%u pos=%.2f° cur=%.2fA%s",
             st.error_code, st.position_deg, st.current_a,
             st.valid ? "" : " (드라이버 수신 이력 없음)");
}

void rcube_motion_report_position(void)
{
    motor_status_t st;
    motor_uart_get_status(&st);

    uint8_t f[HEADER_LEN + 4];
    uint16_t total = sizeof(f);
    f[0] = RCUBE_ADDR_HUB;
    f[1] = RCUBE_OP_GetPosition;
    f[2] = (uint8_t)((total >> 8) & 0xFF);
    f[3] = (uint8_t)(total & 0xFF);
    wr_i32(&f[4], (int32_t)(st.position_deg * 100.0f));
    rcube_cmd_send_frame(f, total);
}

/* ---- 디스패치 -------------------------------------------------------- */

bool rcube_motion_handle(uint8_t op, const uint8_t *payload, uint16_t plen, uint8_t *result)
{
    uint8_t rc;
    switch (op) {
    case RCUBE_OP_SetSingleSpeed:     rc = handle_single_speed(payload, plen); break;
    case RCUBE_OP_SetSingleAngle:     rc = handle_single_angle(payload, plen); break;
    case RCUBE_OP_SetScheduledAngles: rc = handle_scheduled(payload, plen); break;
    case RCUBE_OP_ExecuteBuffer:      rc = handle_execute(payload, plen); break;
    case RCUBE_OP_TimeSync:           rc = handle_timesync(payload, plen); break;
    case RCUBE_OP_SetAngleLimits:     rc = handle_angle_limits(payload, plen); break;
    case RCUBE_OP_SetMotionLimits:    rc = handle_motion_limits(payload, plen); break;
    case RCUBE_OP_SetFaultThresholds: rc = handle_fault_thresholds(payload, plen); break;

    case RCUBE_OP_SaveParameters:
        rc = (rcube_params_save() == ESP_OK) ? RCUBE_RC_OK : RCUBE_RC_FLASH_FAIL;
        break;
    case RCUBE_OP_MoveToOrigin:       rc = handle_move_origin(payload, plen); break;
    case RCUBE_OP_SetDriveState:      rc = handle_drive_state(payload, plen); break;

    case RCUBE_OP_SetThisToOrigin:
        rc = (motor_uart_set_origin(0) == ESP_OK) ? RCUBE_RC_OK : RCUBE_RC_BAD_STATE;
        ESP_LOGI(TAG, "C9 SetThisToOrigin: 현재 위치를 원점으로");
        break;

    case RCUBE_OP_EmergencyStop:
        motion_core_emergency_stop("EmergencyStop(0xD0) 수신");
        rc = RCUBE_RC_OK;
        break;

    case RCUBE_OP_GetMotorStatus:
        rcube_motion_report_status();   /* 회신 자체가 응답 */
        *result = RCUBE_RC_OK;
        return true;

    case RCUBE_OP_GetPosition:
        rcube_motion_report_position();
        *result = RCUBE_RC_OK;
        return true;

    default:
        return false;   /* 모션 명령이 아니다 */
    }
    *result = rc;
    return true;
}
