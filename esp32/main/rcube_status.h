/*
 * rcube_status — 부팅 후 상태 표시(노드ID / 그룹번호 LED) + 연결모드 게이트
 * ------------------------------------------------------------------
 * SK6812 3개: LED0·LED1=노드ID, LED2=그룹번호.
 *
 * 그룹번호(LED2): 전원이 켜진 동안 항상 표시. 00~99를 10진 두 자리로 분해해
 *   십의자리=첫색(0.5s ON) → 0.5s OFF → 일의자리=둘째색(1.0s ON) → 0.5s OFF 반복.
 *   자리색(기획서 노드ID 규약과 정렬): 0White 1Red 2Green 3Blue 4Cyan
 *   5Magenta 6Yellow 7Violet 8Orange 9=어두운Red. (자리0=White라 그룹00=흰+흰)
 *
 * 노드ID(LED0·LED1): 부팅~연결모드 진입 전까지 자기 노드ID 색을 1s ON / 1s OFF
 *   반복 점멸. 색: 0=White(미할당/공장) 1Red 2Green 3Blue 4Cyan 5Magenta 6Yellow
 *   7Violet 8Orange. 연결모드 진입 후엔 BLE/명령 레이어가 노드LED를 제어한다.
 */
#pragma once

#include <stdbool.h>

/* 노드ID/그룹 표시 태스크 기동(부팅 시 1회). */
void rcube_status_start_identity(void);

/* 연결모드로 전환(노드ID 점멸 중지). 최초 전환이면 true 반환. 그룹LED는 계속. */
bool rcube_status_enter_connect_mode(void);

/* 현재 연결모드인지. */
bool rcube_status_in_connect_mode(void);
