/*
 * board_led — 온보드 SK6812 RGB LED 상태등 제어 (3개)
 * ------------------------------------------------------------------
 * 보드 실물 기준 데이터 라인 GPIO38에 SK6812 3개가 데이지체인.
 *   - LED0, LED1 : 노드ID 표시(코너 2개)
 *   - LED2       : 그룹번호 표시(전원버튼 근처)
 * RMT 백엔드(led_strip)로 구동. 여러 태스크에서 색을 바꾸므로 내부 뮤텍스로 보호.
 * (개발보드엔 LED가 1개뿐이라 LED0만 눈에 보이고 LED1/2는 무시된다.)
 */
#pragma once

#include <stdint.h>

/* 스트립 생성 + 뮤텍스 준비. app_main 초기화 단계에서 1회 호출. */
void board_led_init(void);

/* 노드ID LED(LED0·LED1)를 같은 색으로. (0~255, 밝기는 내부 스케일) */
void board_led_set_node(uint8_t r, uint8_t g, uint8_t b);

/* 그룹번호 LED(LED2)를 지정 색으로. */
void board_led_set_group(uint8_t r, uint8_t g, uint8_t b);

/* 3개 전부 같은 색으로(부팅/오류 표시 등). */
void board_led_set_all(uint8_t r, uint8_t g, uint8_t b);
