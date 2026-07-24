/*
 * rcube_cmd — R큐브 명령 레이어 (표준 프레임 파싱 + 디스패치)
 * ------------------------------------------------------------------
 * BLE(추후 CAN)로 들어온 표준 프레임을 파싱해 명령별로 처리한다.
 *   와이어 프레임: [0]TargetId [1]OpCode [2:3]PacketSize(BE) [4:]payload
 *   (정의: shared-protocol/rcube_protocol.h · 단일 소스)
 *
 * 전송계층과 분리: 회신/멀티롤/중계는 rcube_cmd_init()에 등록한 ops 콜백으로
 * 위임한다. → 이 레이어는 NimBLE를 모른다.
 *
 * 구현 범위:
 *   - E0 SetSK6812LED         : payload=[n][R,G,B]×n → 온보드 LED 점등, CmdAck 회신
 *   - A0 SetMultiroleAggregator: 자기 RED + 아그리게이터 승격(ops.agg_start 위임)
 *   - 자기 대상이 아닌 프레임   : 아그리게이터면 멤버로 중계(ops.forward)
 *   - 그 외 OpCode            : 로깅 + CmdAck(BAD_OPCODE) 회신
 */
#pragma once

#include <stdint.h>

/* 완성된 와이어 프레임을 PC로 회신(notify). 성공 시 0. */
typedef int (*rcube_send_fn)(const uint8_t *frame, uint16_t len);

/* 명령 레이어가 전송계층에 위임하는 동작들. NULL이면 해당 기능 미지원. */
typedef struct {
    rcube_send_fn send;                                  /* PC로 프레임 회신 */
    void (*agg_start)(uint8_t link_count, uint8_t group_mode); /* 아그리게이터 승격 */
    void (*agg_stop)(void);                              /* 아그리게이터 해제 */
    int  (*forward)(uint8_t target_id, const uint8_t *frame, uint16_t len); /* 멤버 중계 */
    int  (*forward_all)(const uint8_t *frame, uint16_t len);  /* 전 멤버로 브로드캐스트 중계 */
} rcube_cmd_ops_t;

/* 명령 레이어 초기화. ops 는 호출 이후에도 유효한 저장소를 가리켜야 한다. */
void rcube_cmd_init(const rcube_cmd_ops_t *ops);

/* 수신한 원시 프레임 1개를 파싱·디스패치한다. (BLE write 콜백에서 호출) */
void rcube_cmd_on_frame(const uint8_t *data, uint16_t len);

/* 아그리게이터의 현재 멤버 수를 PC에 0xA1로 보고한다. (멀티롤 레이어가 호출) */
void rcube_cmd_report_members(uint8_t members_connected);
