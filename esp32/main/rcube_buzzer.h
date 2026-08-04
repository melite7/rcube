/*
 * rcube_buzzer — 부저 톤/멜로디 재생 (LEDC PWM)
 * ------------------------------------------------------------------
 * 부저는 IO14로 구동한다. LEDC PWM으로 톤 생성: 주파수=음높이, duty≈진폭.
 * 멜로디는 전용 태스크에서 논블로킹 재생하며, 임의 컨텍스트(호스트 태스크/버튼/부팅)
 * 에서 큐로 재생 요청한다.
 *
 * ★ 구동 극성은 Active-LOW다(개발보드 회로도 2026-07-30 확인).
 *   CONTROL_BUZZ ─[1K]─ PNP(Q3) 베이스, 이미터=VCC_BUZZ, 컬렉터=부저─GND.
 *   LOW=소리, HIGH=무음. 무음을 LOW로 두면 부저가 계속 켜져 발열한다.
 *   자세한 이유와 5V/3.3V 주의는 rcube_buzzer.c 상단 주석 참조.
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

/* 음 하나를 지정 주파수·길이로 재생 요청(큐에 넣고 즉시 반환).
 * 미션코드의 TONE 키프레임과 0xE6 GenerateBuzzerTone이 쓴다. freq_hz=0이면 그 길이만큼
 * 쉰다(재생 태스크가 점유된다). dur_ms=0이면 아무 것도 하지 않는다. */
void rcube_buzzer_play_tone(uint16_t freq_hz, uint16_t dur_ms);

/* 전역 음량 0~100(%). 사각파 ON 비율을 줄여 조절하므로 음정은 변하지 않는다.
 * 기본값은 rcube_buzzer.c의 BUZZER_VOLUME_DEFAULT. */
void rcube_buzzer_set_volume(uint8_t percent);
uint8_t rcube_buzzer_volume(void);
