/*
 * motion_core — Core 1 실시간 모션 (기획서 12.3 / 12.9)
 * ------------------------------------------------------------------
 * 고정주기로 깨어나 "현재 목표 웨이포인트"를 모터 드라이버에 내려보내고, 드라이버가
 * 따라오는지 감시한다. 점과 점 사이 보간과 1kHz 서보 루프는 드라이버의 몫이다.
 *
 * ★ 12.3의 핵심 원칙: 목표가 어디서(BLE/CAN/미션코드/로컬) 왔는지 이 레이어는 모른다.
 *   Core 0의 모든 경로가 같은 큐에 웨이포인트를 넣고, 이 무지가 구조를 단순하게 만든다.
 *
 * 버퍼 실행 모델(12.9):
 *   Immediate — 넣는 즉시 다음 주기에 실행
 *   Buffered  — 큐에 쌓아만 두고, ExecuteBuffer(Run, T0)를 받아야 출발
 * 다축 동기는 이 Buffered + 공통 시각 T0로 구현한다. 드라이버에는 예약 실행이 없으므로
 * T0까지 기다리는 주체는 각 큐브의 이 태스크다(확장 규격 §3.1).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 웨이포인트 — 전송계층 무관 포맷(확장 규격 §3.3과 동일 필드). */
typedef struct {
    uint32_t t_offset_us;   /* T0 기준 도달 시각 */
    int32_t  position;      /* 0.01° 단위 */
    int16_t  velocity;      /* 0.01°/s. 0=드라이버 자동 */
    uint8_t  flags;         /* bit0 = in-position 확인 지점(도달 시 완료 통보) */
    uint16_t seq;           /* 순번(완료 통보에 실어 보낸다) */
} motion_waypoint_t;

#define MOTION_WP_FLAG_INPOS  0x01u

/* ExecuteBuffer action (확장 규격 §3.4) */
#define MOTION_EXEC_RUN     0x01u
#define MOTION_EXEC_CANCEL  0x02u
#define MOTION_EXEC_FLUSH   0x03u

/* 큐 깊이. look-ahead 2~3개면 BLE 지터를 흡수한다(12.9)지만, 궤적 적재를 받으려면
 * 더 깊어야 한다. CAN 멀티프레임 최대 446B ÷ 12B = 37개를 담을 수 있게 잡는다. */
#define MOTION_QUEUE_DEPTH 48

/* 기본 키프레임 주기(기획서 11장 표: PTP 20~50ms). */
#define MOTION_PERIOD_DEFAULT_MS 50
#define MOTION_PERIOD_MIN_MS      5
#define MOTION_PERIOD_MAX_MS    200

/* 목표 갱신이 이 시간 동안 없으면 안전 감속 후 정지(12.9 상호 watchdog). */
#define MOTION_STARVE_TIMEOUT_MS 500

/* Core 1 태스크 기동. motor_uart_init() 이후 1회. */
void motion_core_init(void);

/* 웨이포인트 적재. buffered=false면 즉시 실행 대상. 큐가 가득 차면 false. */
bool motion_core_push(const motion_waypoint_t *wp, bool buffered);

/* ExecuteBuffer. t_start_us는 마스터 시계 기준(0이면 즉시). 성공 시 true. */
bool motion_core_execute(uint8_t action, uint64_t t_start_us);

/* 즉시 정지(E-Stop/폴트). 큐를 비우고 드라이버를 안전 정지시킨다. */
void motion_core_emergency_stop(const char *reason);

/* TimeSync로 구한 마스터 시계 오프셋(us). 마스터시각 = 로컬시각 + offset. */
void motion_core_set_time_offset(int64_t offset_us);
int64_t motion_core_master_now_us(void);

/* 키프레임 주기 변경. 범위 밖이면 false. */
bool motion_core_set_period(uint16_t period_ms);

/* 현재 적재된(미실행) 웨이포인트 수. */
uint16_t motion_core_pending(void);
