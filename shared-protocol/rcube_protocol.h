/*
 * rcube_protocol.h
 * ----------------------------------------------------------------
 * ★ 자동 생성 파일 — 직접 수정 금지.
 *   원본: docs/R큐브_프로토콜_BLE_CAN__20260703.xlsx
 *   생성: tools/gen_protocol.py  (2026-07-23)
 *   수정이 필요하면 xlsx를 고치고 생성기를 다시 실행하세요.
 * ----------------------------------------------------------------
 */
#pragma once
#include <stdint.h>
#include "rcube_opcodes.h"
#include "rcube_resultcodes.h"

/* =====================================================================
 * 표준 패킷 헤더 (APPENDIX A/D · CAN 시트 A절)
 *   [0] TargetId  [1] OpCode  [2:3] PacketSize(uint16, 와이어에서 BE)
 *   [4...] Property + Value (= CAN 데이터필드와 동일)
 * ===================================================================== */
typedef struct __attribute__((packed)) {
    uint8_t  target_id;    /* [0]   대상 주소 (아래 RCUBE_ADDR_*) */
    uint8_t  op_code;      /* [1]   rcube_opcode_t */
    uint16_t packet_size;  /* [2:3] 전체 패킷 길이 — 와이어에서 big-endian */
} rcube_header_t;

/* ---- TargetId / 주소 규약 (APPENDIX A) ---- */
#define RCUBE_ADDR_NODE_MIN   0x01u   /* 개별 노드 0x01~0x08 */
#define RCUBE_ADDR_NODE_MAX   0x08u
#define RCUBE_ADDR_HUB        0xFEu   /* 대표(edge central=노드01) */
#define RCUBE_ADDR_BROADCAST  0xFFu   /* 전 큐브 브로드캐스트 */

/* ---- CmdAck Status (APPENDIX E, ASCII ACK/NAK) ---- */
#define RCUBE_ACK  0x06u
#define RCUBE_NAK  0x15u

/* =====================================================================
 * CAN 29비트 확장 ID 레이아웃 (CAN 시트 B절)
 *   [28:26] Priority | [25:18] OpCode | [17] MULTI | [16] FLAG
 *   [15:8]  SrcId    | [7:0]   DstId
 * ===================================================================== */
#define RCUBE_CAN_ID(pri, op, multi, flag, src, dst)  \
    ( ((uint32_t)((pri)  & 0x7u ) << 26) |            \
      ((uint32_t)((op)   & 0xFFu) << 18) |            \
      ((uint32_t)((multi)& 0x1u ) << 17) |            \
      ((uint32_t)((flag) & 0x1u ) << 16) |            \
      ((uint32_t)((src)  & 0xFFu) <<  8) |            \
      ((uint32_t)((dst)  & 0xFFu)      ) )

#define RCUBE_CAN_PRI(id)    (((id) >> 26) & 0x7u)
#define RCUBE_CAN_OPCODE(id) (((id) >> 18) & 0xFFu)
#define RCUBE_CAN_MULTI(id)  (((id) >> 17) & 0x1u)
#define RCUBE_CAN_FLAG(id)   (((id) >> 16) & 0x1u)
#define RCUBE_CAN_SRC(id)    (((id) >>  8) & 0xFFu)
#define RCUBE_CAN_DST(id)    ( (id)        & 0xFFu)

#define RCUBE_CAN_SRC_MASTER 0xFEu   /* PC 또는 edge central */

/* ---- CAN 우선순위 클래스 (CAN 시트 C절, 낮을수록 우선) ---- */
typedef enum {
    RCUBE_PRI_ESTOP        = 0,  /* D0 EmergencyStop 최우선 */
    RCUBE_PRI_SAFETY_SYNC  = 1,  /* D1·D2·D9·C7 */
    RCUBE_PRI_MOTION       = 2,  /* C0~C3·C5·C8~CF */
    RCUBE_PRI_QUERY        = 3,  /* AF·B0~B6 */
    RCUBE_PRI_PERIPHERAL   = 4,  /* E0~E3·E5~E7·EA~ED */
    RCUBE_PRI_CONFIG       = 5,  /* D3~D8·DA·DB */
    RCUBE_PRI_MISSION_OTA  = 6,  /* F0~F8 */
    RCUBE_PRI_RESERVED     = 7,
} rcube_can_pri_t;
