/*
 * rcube_resultcodes.h
 * ----------------------------------------------------------------
 * ★ 자동 생성 파일 — 직접 수정 금지.
 *   원본: docs/R큐브_프로토콜_BLE_CAN__20260703.xlsx + R큐브_프로토콜_확장_20260728.md
 *   생성: tools/gen_protocol.py  (2026-07-28)
 *   수정이 필요하면 xlsx를 고치고 생성기를 다시 실행하세요.
 * ----------------------------------------------------------------
 */
#pragma once
#include <stdint.h>

/* CmdAck(0xAF) ResultCode — ACK 시 0x00(OK) */
typedef enum {
    RCUBE_RC_OK               = 0x00,  /* 성공 (Status=ACK) */
    RCUBE_RC_BAD_OPCODE       = 0x01,  /* 미지원·미정의 OpCode */
    RCUBE_RC_BAD_LENGTH       = 0x02,  /* PacketSize 불일치 */
    RCUBE_RC_CRC_FAIL         = 0x03,  /* CRC16/CRC32 검증 실패 */
    RCUBE_RC_BAD_PARAM        = 0x04,  /* 파라미터 범위 초과 */
    RCUBE_RC_BAD_STATE        = 0x05,  /* 현재 상태에서 불가 (armed/실행 중 등) */
    RCUBE_RC_BUFFER_FULL      = 0x06,  /* 키프레임 FIFO/데이터셋 적재 초과 */
    RCUBE_RC_BUFFER_UNDERRUN  = 0x07,  /* 실시간 스트림 언더런 */
    RCUBE_RC_SEQ_GAP          = 0x08,  /* 스트림·청크 시퀀스 누락 */
    RCUBE_RC_NODE_NOT_FOUND   = 0x09,  /* 대상 NodeID 미존재 */
    RCUBE_RC_MOTOR_FAULT      = 0x0A,  /* 모터 오류 (상세는 B2 FaultCode) */
    RCUBE_RC_EXT5V_CONFLICT   = 0x0B,  /* 외부 5V 연결 충돌 */
    RCUBE_RC_FLASH_FAIL       = 0x0C,  /* NVS/LittleFS 저장 실패 */
    RCUBE_RC_MISSION_VER      = 0x0D,  /* .mpy 버전·펌웨어 불일치 */
    RCUBE_RC_ESTOP_ACTIVE     = 0x0E,  /* E-stop/heartbeat fail-safe 상태로 거부 */
    RCUBE_RC_TIMEOUT          = 0x0F,  /* 내부 처리 타임아웃 */
    RCUBE_RC_IK_FAIL          = 0x10,  /* 도달 불가 좌표·특이점 근접 (IK 해 없음) */
    RCUBE_RC_ANGLE_LIMIT      = 0x11,  /* 각도 한계·금지구간 위반 */
} rcube_result_t;
