/*
 * rcube_ledc — LEDC(PWM) 타이머·채널 배정 단일 소스
 * ------------------------------------------------------------------
 * ★ 왜 한곳에 모으는가
 *   LEDC는 "타이머가 주파수를, 채널이 duty를" 담당한다. 즉 **같은 타이머에 물린
 *   채널들은 주파수를 공유**한다. 부저는 음마다 주파수를 바꾸고(262Hz~1046Hz),
 *   서보는 50Hz로 고정이어야 하므로, 둘이 같은 타이머를 쓰면 서로의 주파수를
 *   덮어써 부저는 음정이 틀어지고 서보는 펄스폭이 망가진다.
 *   그래서 배정을 여기서 못 박고, 아래 _Static_assert로 충돌을 컴파일 단계에서 막는다.
 *
 * 배정 (ESP32-S3 low-speed: 타이머 4개(0~3), 채널 8개(0~7))
 *   부저  IO14  TIMER_0  CHANNEL_0  가변 주파수(음높이) · 10bit
 *   서보  IO21  TIMER_1  CHANNEL_1  50Hz 고정      · 14bit
 *
 * 서보를 14비트로 잡은 이유: 50Hz(주기 20ms)에서 14bit면 20ms/16384 ≈ 1.22us 분해능이라
 * RC 서보의 1~2ms 펄스를 충분히 세밀하게 만든다. 10bit면 19.5us라 각도가 뭉갠다.
 *
 * 새 PWM 주변장치를 붙일 때는 반드시 여기에 줄을 추가하고 다른 타이머를 쓴다.
 */
#pragma once

#include "driver/ledc.h"

/* 공통: 저속 모드를 쓴다(고속 모드는 S3에 없다). */
#define RCUBE_LEDC_MODE            LEDC_LOW_SPEED_MODE

/* ---- 부저 (rcube_buzzer.c) ---- */
#define RCUBE_LEDC_BUZZER_TIMER    LEDC_TIMER_0
#define RCUBE_LEDC_BUZZER_CHANNEL  LEDC_CHANNEL_0
#define RCUBE_LEDC_BUZZER_RES      LEDC_TIMER_10_BIT   /* duty 0~1023 */

/* ---- RC 서보 (미구현 — 자리 예약) ---- */
#define RCUBE_LEDC_SERVO_TIMER     LEDC_TIMER_1
#define RCUBE_LEDC_SERVO_CHANNEL   LEDC_CHANNEL_1
#define RCUBE_LEDC_SERVO_RES       LEDC_TIMER_14_BIT   /* 50Hz에서 ≈1.22us 분해능 */
#define RCUBE_LEDC_SERVO_FREQ_HZ   50

/* 배정이 겹치면 빌드를 실패시킨다 — 주석만으로는 언젠가 어긋난다. */
_Static_assert(RCUBE_LEDC_BUZZER_TIMER != RCUBE_LEDC_SERVO_TIMER,
               "부저와 서보가 같은 LEDC 타이머를 쓰면 주파수가 서로 덮어써진다");
_Static_assert(RCUBE_LEDC_BUZZER_CHANNEL != RCUBE_LEDC_SERVO_CHANNEL,
               "부저와 서보가 같은 LEDC 채널을 쓸 수 없다");
