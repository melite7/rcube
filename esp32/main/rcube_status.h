/*
 * rcube_status — 부팅 후 그룹번호 표시(아이덴티티 LED) + 모드 게이트
 * ------------------------------------------------------------------
 * 부팅 성공 후 "연결모드"로 진입하기 전까지, 그룹번호를 색상 LED로 표시한다.
 *   - 그룹번호(00~99)를 10진 두 자리로 분해: 십의자리=첫색(0.5s), 일의자리=둘째색(1.0s).
 *     자리색(기획서 노드ID 규약과 정렬): 0White 1Red 2Green 3Blue 4Cyan
 *             5Magenta 6Yellow 7Violet 8Orange 9=어두운Red(Red의 1/2 밝기).
 *     패턴: 첫색 0.5s ON → 0.5s OFF → 둘째색 1.0s ON → 0.5s OFF … 반복.
 *   - 자리 0=White라 그룹 00은 자연히 흰색+흰색(별도 예외 없음).
 *
 * 버튼을 눌러 연결모드로 진입하면(rcube_status_enter_connect_mode) 아이덴티티
 * 표시를 멈추고, 이후 LED는 BLE 연결/명령 레이어가 제어한다(모드 게이트).
 */
#pragma once

#include <stdbool.h>

/* 아이덴티티 표시 태스크 기동(부팅 시 1회). */
void rcube_status_start_identity(void);

/* 연결모드로 전환(아이덴티티 표시 중지). 최초 전환이면 true 반환. */
bool rcube_status_enter_connect_mode(void);

/* 현재 연결모드인지. */
bool rcube_status_in_connect_mode(void);
