/*
 * rcube_opcodes.h
 * ----------------------------------------------------------------
 * ★ 자동 생성 파일 — 직접 수정 금지.
 *   원본: docs/R큐브_프로토콜_BLE_CAN__20260703.xlsx + R큐브_프로토콜_확장_20260728.md
 *   생성: tools/gen_protocol.py  (2026-07-29)
 *   수정이 필요하면 xlsx를 고치고 생성기를 다시 실행하세요.
 * ----------------------------------------------------------------
 */
#pragma once
#include <stdint.h>

/* 총 62 명령 (BLE/CAN 응용계층 공유) */
typedef enum {
    RCUBE_OP_CmdAck                     = 0xAF,  /* APPENDIX-Response */
    RCUBE_OP_NodeAnnounce               = 0xD9,  /* CAN-Discovery */
    RCUBE_OP_SetMultiroleAggregator     = 0xA0,  /* CONFIG-Assembly */
    RCUBE_OP_SetMultiroleInAction       = 0xA1,  /* CONFIG-Assembly */
    RCUBE_OP_SetMultiroleVirtualColor   = 0xA2,  /* CONFIG-Assembly */
    RCUBE_OP_GetMultiroleVirtualColor   = 0xA3,  /* CONFIG-Assembly */
    RCUBE_OP_SetNodeConfig              = 0xD3,  /* CONFIG-Config */
    RCUBE_OP_GetNodeConfig              = 0xD4,  /* CONFIG-Config */
    RCUBE_OP_SetEdgeCentralConfig       = 0xD5,  /* CONFIG-Config */
    RCUBE_OP_GetEdgeCentralConfig       = 0xD6,  /* CONFIG-Config */
    RCUBE_OP_ResetConfig                = 0xD7,  /* CONFIG-Config */
    RCUBE_OP_SetAngleLimits             = 0xD8,  /* CONFIG-Config */
    RCUBE_OP_GetProducts                = 0xDA,  /* CONFIG-Config */
    RCUBE_OP_SaveParameters             = 0xDB,  /* CONFIG-Config */
    RCUBE_OP_MissionUploadBegin         = 0xF0,  /* CONFIG-Mission/OTA */
    RCUBE_OP_MissionUploadChunk         = 0xF1,  /* CONFIG-Mission/OTA */
    RCUBE_OP_MissionUploadCommit        = 0xF2,  /* CONFIG-Mission/OTA */
    RCUBE_OP_GetMissionInfo             = 0xF3,  /* CONFIG-Mission/OTA */
    RCUBE_OP_DeleteMission              = 0xF4,  /* CONFIG-Mission/OTA */
    RCUBE_OP_OtaBegin                   = 0xF5,  /* CONFIG-Mission/OTA */
    RCUBE_OP_OtaChunk                   = 0xF6,  /* CONFIG-Mission/OTA */
    RCUBE_OP_OtaCommit                  = 0xF7,  /* CONFIG-Mission/OTA */
    RCUBE_OP_OtaActivate                = 0xF8,  /* CONFIG-Mission/OTA */
    RCUBE_OP_SetSK6812LED               = 0xE0,  /* CONTROL-Control */
    RCUBE_OP_SetSingleServo             = 0xE1,  /* CONTROL-Control */
    RCUBE_OP_SetExtDigitalOut           = 0xE2,  /* CONTROL-Control */
    RCUBE_OP_SetMusicNotes              = 0xE3,  /* CONTROL-Control */
    RCUBE_OP_SetAggregateMusicNotes     = 0xE4,  /* CONTROL-Control */
    RCUBE_OP_PlayMusicNotes             = 0xE5,  /* CONTROL-Control */
    RCUBE_OP_GenerateBuzzerTone         = 0xE6,  /* CONTROL-Control */
    RCUBE_OP_SetPowerState              = 0xE7,  /* CONTROL-Control */
    RCUBE_OP_SetSingleSpeed             = 0xC0,  /* CONTROL-Motion */
    RCUBE_OP_SetSingleAngle             = 0xC1,  /* CONTROL-Motion */
    RCUBE_OP_SetScheduledAngles         = 0xC2,  /* CONTROL-Motion */
    RCUBE_OP_PlayScheduledAngles        = 0xC3,  /* CONTROL-Motion */
    RCUBE_OP_SetAggregateAngles         = 0xC4,  /* CONTROL-Motion */
    RCUBE_OP_SetKeyframeStream          = 0xC5,  /* CONTROL-Motion */
    RCUBE_OP_ExecuteBuffer              = 0xC7,  /* CONTROL-Motion */
    RCUBE_OP_MoveToOrigin               = 0xC8,  /* CONTROL-Motion */
    RCUBE_OP_SetThisToOrigin            = 0xC9,  /* CONTROL-Motion */
    RCUBE_OP_SetTargetTorque            = 0xCA,  /* CONTROL-Motion */
    RCUBE_OP_SetDriveState              = 0xCB,  /* CONTROL-Motion */
    RCUBE_OP_SetComplianceMode          = 0xCC,  /* CONTROL-Motion */
    RCUBE_OP_SetMotionLimits            = 0xCD,  /* CONTROL-Motion */
    RCUBE_OP_SetFaultThresholds         = 0xCE,  /* CONTROL-Motion */
    RCUBE_OP_SetControlGains            = 0xCF,  /* CONTROL-Motion */
    RCUBE_OP_SetKinematicModel          = 0xEA,  /* CONTROL-Task-space */
    RCUBE_OP_SetWorkOrigin              = 0xEB,  /* CONTROL-Task-space */
    RCUBE_OP_MoveToPosition             = 0xEC,  /* CONTROL-Task-space */
    RCUBE_OP_SetScheduledPositions      = 0xED,  /* CONTROL-Task-space */
    RCUBE_OP_EmergencyStop              = 0xD0,  /* SAFETY-Safety */
    RCUBE_OP_Heartbeat                  = 0xD1,  /* SAFETY-Safety */
    RCUBE_OP_TimeSync                   = 0xD2,  /* SAFETY-Safety */
    RCUBE_OP_GetSensors                 = 0xB0,  /* SENSOR-Sensor */
    RCUBE_OP_SetSensorStream            = 0xB1,  /* SENSOR-Sensor */
    RCUBE_OP_GetMotorStatus             = 0xB2,  /* SENSOR-Sensor */
    RCUBE_OP_GetNodeState               = 0xB3,  /* SENSOR-Sensor */
    RCUBE_OP_GetUnitRoster              = 0xB4,  /* SENSOR-Sensor */
    RCUBE_OP_GetExtPortMode             = 0xB5,  /* SENSOR-Sensor */
    RCUBE_OP_GetPosition                = 0xB6,  /* SENSOR-Sensor */
    RCUBE_OP_MotionComplete             = 0xB7,  /* CONTROL-Motion */
    RCUBE_OP_MissionControl             = 0xF9,  /* CONFIG-Mission/OTA */
} rcube_opcode_t;
