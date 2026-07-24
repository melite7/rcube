/*
 * board_led — 온보드 SK6812 RGB LED 상태등 제어
 * ------------------------------------------------------------------
 * 보드 실물 기준 데이터 라인은 GPIO38. RMT 백엔드(led_strip)로 구동한다.
 * 여러 태스크(부팅/버튼/BLE 호스트)에서 색을 바꾸므로 내부 뮤텍스로 보호한다.
 */
#pragma once

#include <stdint.h>

/* 스트립 생성 + 뮤텍스 준비. app_main 초기화 단계에서 1회 호출. */
void board_led_init(void);

/* 단색 점등(0~255). 스트립은 마지막 값을 래치하므로 계속 켜져 있다. */
void board_led_set(uint8_t r, uint8_t g, uint8_t b);
