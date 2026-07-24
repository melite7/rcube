/*
 * rcube_cmd — R큐브 명령 레이어 (Phase 2: 표준 프레임 파싱 + 디스패치)
 * ------------------------------------------------------------------
 * BLE(추후 CAN)로 들어온 표준 프레임을 파싱해 명령별로 처리한다.
 *   와이어 프레임: [0]TargetId [1]OpCode [2:3]PacketSize(BE) [4:]payload
 *   (정의: shared-protocol/rcube_protocol.h · 단일 소스)
 *
 * 전송계층과 분리: 응답(notify)은 rcube_cmd_init()에 등록한 콜백으로 내보낸다.
 * → BLE(ble_rcube.c)는 이 레이어를 몰라도 되고, 이 레이어도 NimBLE를 모른다.
 *
 * Phase 2 구현 범위:
 *   - E0 SetSK6812LED        : payload=[n][R,G,B]×n → 온보드 LED 점등, CmdAck 회신
 *   - A0 SetMultiroleAggregator: 아그리게이터로 승격 → 자기 RED → 0xA1 회신
 *                                (실제 멤버 BLE 연결은 Phase 5)
 *   - 그 외 OpCode           : 로깅 + CmdAck(BAD_OPCODE) 회신
 */
#pragma once

#include <stdint.h>

/* 응답 프레임(완성된 와이어 프레임)을 전송하는 콜백. 성공 시 0. */
typedef int (*rcube_send_fn)(const uint8_t *frame, uint16_t len);

/* 명령 레이어 초기화. responder 는 회신(notify) 송신 함수(없으면 NULL). */
void rcube_cmd_init(rcube_send_fn responder);

/* 수신한 원시 프레임 1개를 파싱·디스패치한다. (BLE write 콜백에서 호출) */
void rcube_cmd_on_frame(const uint8_t *data, uint16_t len);
