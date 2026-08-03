/*
 * rcube_status — 상태 표시(노드ID/그룹번호 LED) + 모드 전이
 * ------------------------------------------------------------------
 * SK6812 3개: LED0·LED1=노드ID, LED2=그룹번호.
 *
 * 그룹번호(LED2): 전원 내내 표시(00~99 두 자리 색상). 그룹 0=흰색.
 *
 * 노드ID LED(LED0·LED1) — 기획서 5장 [ID 표시용 칼라LED 점등 규칙]:
 *   - 미연결(대기/연결모드 진행중) : 1초 주기 점멸
 *       · 고정형(노드ID 1~8 저장) → 저장 노드ID 색
 *       · 비고정형(노드ID 0)      → 흰색
 *   - 연결 완료                   : 상시 점등
 *       · 고정형   → 저장 노드ID 색
 *       · 비고정형 → 상위가 배정한 가상 노드ID 색(E0로 지정받은 색)
 *   - BLE 허브/edge central       : 담당 멤버 전원 연결 전엔 점멸, 완료 시 상시 점등
 *       · 비고정형 초기구성 → Red(가상1),  고정형 → 자기 저장 노드ID 색
 *   - 설정모드(미연결)            : 흰색 0.25초 빠른 점멸(위 1초 점멸들과 구분)
 *       · 설정모드에서 상위가 붙으면 위의 "연결 완료" 표시로 넘어간다.
 *         (CAN으로 바꾼 큐브는 설정모드 광고가 유일한 BLE 접속 경로라, 여기서
 *          점멸이 안 풀리면 연결 후에도 흰색 점멸에 갇힌다.)
 *
 * 색은 "표시색"과 "모드"로 분리한다. 표시색은 기본이 자기 노드ID 색이고,
 * 상위(PC/허브)가 E0·A0로 지정하면 그 색으로 덮인다(비고정형 가상ID 색).
 * 점멸/점등 여부는 오직 모드가 정한다.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    RCUBE_LED_IDLE = 0,   /* 미연결(대기/광고중) — 1초 점멸 */
    RCUBE_LED_LINKED,     /* 상위와 연결 완료 — 상시 점등 */
    RCUBE_LED_HUB_WAIT,   /* 허브/edge central, 담당 멤버 미완료 — 1초 점멸 */
    RCUBE_LED_CONFIG,     /* 설정모드 — 흰색 0.25초 점멸 */
} rcube_led_mode_t;

/* 노드ID/그룹 표시 태스크 기동(부팅 시 1회). 초기 모드=IDLE. */
void rcube_status_start(void);

/* 표시 모드 전환. */
void rcube_status_set_mode(rcube_led_mode_t mode);
rcube_led_mode_t rcube_status_mode(void);

/* 상위 지정색(E0 SetSK6812LED / A0 승격). 이후 표시색으로 쓰인다. */
void rcube_status_set_color(uint8_t r, uint8_t g, uint8_t b);

/* 지정색을 지우고 자기 노드ID 색으로 되돌린다(연결 해제 시). */
void rcube_status_clear_color(void);

/* 노드ID(0~9) → 기획서 규약색. 0=White, 1=Red, 2=Green, 3=Blue … */
void rcube_status_node_color(uint8_t node_id, uint8_t *r, uint8_t *g, uint8_t *b);

/* 짧게 누름 = 연결모드 진입. 최초 전환이면 true(진입 멜로디 재생용). */
bool rcube_status_enter_connect_mode(void);

/* 길게(3초) 누름 = 설정모드. 흰색 0.25초 점멸을 연결 전까지 유지한다.
 * (상위가 붙으면 LINKED/HUB_WAIT로 넘어가고, 끊기면 다시 이 표시로 돌아온다.) */
void rcube_status_enter_config_mode(void);

/* 현재 설정모드인지. */
bool rcube_status_in_config_mode(void);
