/*
 * rcube_buzzer — 부저 톤/멜로디 재생 (LEDC PWM)
 * ------------------------------------------------------------------
 * 메인보드 부저는 IO14(AO3400A 게이트, 기본 Low=무음). LEDC PWM으로 톤 생성:
 * 주파수=음높이, duty≈진폭. 멜로디는 전용 태스크에서 논블로킹 재생하며,
 * 임의 컨텍스트(호스트 태스크/버튼/부팅)에서 큐로 재생 요청한다.
 *
 * 주의: ESP32-S3 개발보드엔 IO14에 부저가 없어 소리는 안 나지만 PWM/로그는 동작.
 * 메인보드에서 실제 소리가 난다.
 */
#pragma once

#include "rcube_melody.h"

/* LEDC 타이머/채널 준비 + 재생 태스크 기동. app_main에서 1회. */
void rcube_buzzer_init(void);

/* 멜로디 재생을 요청(큐에 넣고 즉시 반환). 태스크 미기동이면 무시. */
void rcube_buzzer_play(rcube_melody_id_t id);

/* 단일 톤 즉시 출력(freq_hz=0 이면 무음). 저수준/테스트용. */
void rcube_buzzer_tone(uint16_t freq_hz);
