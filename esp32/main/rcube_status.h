/*
 * rcube_status — 상태 표시(노드ID/그룹번호 LED) + 모드 전이
 * ------------------------------------------------------------------
 * SK6812 3개: LED0·LED1=노드ID, LED2=그룹번호.
 *
 * 그룹번호(LED2): 전원 내내 표시(00~99 색상, [[rcube-fw-indicators]] 참조).
 *
 * 노드ID LED(LED0·LED1) 상태:
 *   - identity : 부팅 후, 자기 노드ID 색을 1s ON / 1s OFF (0=White 미할당).
 *   - config   : 설정모드 — 흰색 0.25s ON / 0.25s OFF 빠른 점멸(버튼 놓아도 유지).
 *   - yield    : BLE 연결/광고로 LED 소유권을 ble_rcube·명령 레이어에 넘김
 *                (연결 후 상위가 지정한 색으로 점등).
 */
#pragma once

#include <stdbool.h>

/* 노드ID/그룹 표시 태스크 기동(부팅 시 1회). 초기 상태=identity. */
void rcube_status_start_identity(void);

/* 짧게 누름 = 연결모드: 노드LED를 BLE/명령 레이어에 넘긴다(광고 cyan 등).
 * 최초 전환이면 true. */
bool rcube_status_enter_connect_mode(void);

/* 길게(3초) 누름 = 설정모드: 노드LED 흰색 0.25s 빠른 점멸(연결 전까지 유지). */
void rcube_status_enter_config_mode(void);

/* BLE 연결됨: 노드LED 소유권을 상위(지정색)에 넘긴다. */
void rcube_status_on_connected(void);

/* BLE 끊김: 설정모드였다면 흰색 점멸을 재개한다. */
void rcube_status_on_disconnected(void);

/* 현재 설정모드인지. */
bool rcube_status_in_config_mode(void);
