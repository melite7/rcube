/*
 * rcube_params — 모터 제어 파라미터·리밋 (기획서 8장 / 확장 규격 §2.12)
 * ------------------------------------------------------------------
 * 각도 소프트리밋·모션 리밋·폴트 임계를 보관하고, 목표값이 허용 범위인지 판정한다.
 *
 * ★ 왜 미션코드보다 먼저인가
 *   기획서 8장의 3단 안전은 (1) 각도 소프트리밋 → (2) 폴트 임계 트립 →
 *   (3) edge central IK 해 배제 순이다. 미션이 자율로 목표를 뿌리기 시작하면
 *   "잘못된 목표를 큐브가 스스로 거부하는 층"이 반드시 아래에 있어야 한다.
 *   이 파일이 그 1·2단이다.
 *
 * ★ 적용과 저장을 분리한다
 *   D8/CD/CE는 받는 즉시 적용되지만 NVS에는 쓰지 않는다. DB(SaveParameters)를
 *   받아야 영구 저장된다. 튜닝 중 잘못된 값을 저장해 다음 부팅부터 못 쓰게 되는
 *   상황을 막기 위해서다.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    /* 각도 소프트리밋 (0.01°). min==max==0 이면 리밋 없음. */
    int32_t  angle_min;
    int32_t  angle_max;
    /* 모션 리밋. 0이면 그 항목은 제한하지 않는다. */
    uint16_t max_vel;      /* 0.1°/s  */
    uint16_t max_acc;      /* °/s²    */
    uint16_t max_jerk;     /* °/s³    */
    /* 폴트 임계. 0이면 해당 감시 끔. */
    uint16_t follow_err;   /* 0.01°   추종오차 */
    uint16_t over_current; /* 0.01A            */
    uint8_t  over_temp;    /* °C               */
} rcube_params_t;

/* NVS에서 로드(없으면 기본값=전부 해제). app_main 초기화에서 1회. */
esp_err_t rcube_params_init(void);

const rcube_params_t *rcube_params(void);

/* 즉시 적용(NVS 저장 안 함). */
void rcube_params_set_angle_limits(int32_t min_centi_deg, int32_t max_centi_deg);
void rcube_params_set_motion_limits(uint16_t max_vel, uint16_t max_acc, uint16_t max_jerk);
void rcube_params_set_fault_thresholds(uint16_t follow_err, uint16_t over_current,
                                       uint8_t over_temp);

/* 현재 값을 NVS에 영구 저장(DB SaveParameters). */
esp_err_t rcube_params_save(void);

/* 목표 각도(0.01°)가 소프트리밋 안인지. 리밋이 없으면 항상 true. */
bool rcube_params_angle_ok(int32_t centi_deg);

/* 속도(0.1°/s)를 max_vel로 깎는다. 제한이 없으면 그대로 돌려준다. */
uint16_t rcube_params_clamp_vel(uint16_t vel_deci_dps);
