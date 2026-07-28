/*
 * rcube_motion — 모션 명령 파싱 (Core 0)
 * ------------------------------------------------------------------
 * 네트워크(BLE/CAN)나 미션코드가 준 모션 명령을 웨이포인트로 바꿔 motion_core의
 * 목표 큐에 넣는다. 규격은 docs/R큐브_프로토콜_확장_20260728.md §2.11·§3.3·§3.4.
 *
 * 이 레이어가 rcube_cmd에서 분리된 이유: rcube_cmd는 설정·표시 명령이 대부분이고
 * 모션은 궤적·버퍼·좌표 변환으로 계속 커진다. 섞으면 디스패처가 비대해진다.
 *
 * ★ 여기서 만든 웨이포인트가 어느 전송계층으로 들어왔는지 motion_core는 모른다
 *   (기획서 12.3). 그 무지를 유지하는 것이 이 파일의 책임이다.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 모션 계열 OpCode를 처리한다. 처리했으면 true를 반환하고 *result에 ResultCode를
 * 채운다. 모션 명령이 아니면 false(rcube_cmd가 다른 처리를 이어간다). */
bool rcube_motion_handle(uint8_t op, const uint8_t *payload, uint16_t plen, uint8_t *result);

/* 조회 명령 회신(B2/B6). 요청을 받은 쪽에서 호출한다. */
void rcube_motion_report_status(void);
void rcube_motion_report_position(void);
