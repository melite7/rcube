/*
 * ble_rcube — R큐브 BLE 페리페럴(NimBLE)
 * ------------------------------------------------------------------
 * 로드맵 12.1/12.2: 통신 스택은 Core 0. 지금은 "이름 광고 + 연결 수락"만.
 *
 * 동작:
 *   - ble_rcube_init(): NimBLE 스택/GATT 초기화. 광고는 아직 시작하지 않는다.
 *   - ble_rcube_start_advertising(): 광고 시작 요청(BOOT 버튼에서 호출).
 *     스택 sync 이전에 호출돼도 sync 완료 시 자동으로 광고를 시작한다.
 *   - 연결이 끊기면 재연결을 위해 광고를 자동 재개한다.
 *
 * 광고 이름: "RCUBE00.00"
 */
#pragma once

#include <stdbool.h>

/* NimBLE 스택 초기화(광고 시작 안 함). app_main에서 1회 호출. */
void ble_rcube_init(void);

/* 광고 시작을 요청한다. 버튼 등 임의 태스크에서 호출 가능(스레드 안전). */
void ble_rcube_start_advertising(void);

/* 현재 광고 중이거나 연결된 상태인지. */
bool ble_rcube_is_active(void);
