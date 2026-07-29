/*
 * rcube_buzzer — 부저 톤/멜로디 재생 (LEDC PWM)
 * ------------------------------------------------------------------
 * 메인보드 부저는 IO14(AO3400A 게이트, 기본 Low=무음). LEDC PWM으로 톤 생성:
 * 주파수=음높이, duty≈진폭. 멜로디는 전용 태스크에서 논블로킹 재생하며,
 * 임의 컨텍스트(호스트 태스크/버튼/부팅)에서 큐로 재생 요청한다.
 *
 * LEDC 타이머·채널 배정은 rcube_ledc.h에 모아 두었다(서보와의 주파수 충돌 방지).
 *
 * 개발보드: IO14에 부저를 직접 배선하면 소리가 난다(2026-07-29 확인). 배선이 없어도
 * PWM 출력과 로그는 정상 동작하므로 멜로디 재생 여부는 로그로 확인할 수 있다.
 */
#pragma once

#include "rcube_melody.h"

/* LEDC 타이머/채널 준비 + 재생 태스크 기동. app_main에서 1회. */
void rcube_buzzer_init(void);

/* 멜로디 재생을 요청(큐에 넣고 즉시 반환). 태스크 미기동이면 무시. */
void rcube_buzzer_play(rcube_melody_id_t id);

/* 그룹번호 알림음(두 자리 → 두 음). 기획서 5장 [소리 규칙] 2026-07-29 재정의.
 * 음이 런타임에 정해지므로 전용 진입점을 둔다. */
void rcube_buzzer_play_group(uint8_t group_id);

/* 단일 톤 즉시 출력(freq_hz=0 이면 무음). 저수준/테스트용. */
void rcube_buzzer_tone(uint16_t freq_hz);
