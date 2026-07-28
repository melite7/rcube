"""
rcube_protocol.py
★ 자동 생성 파일 — 직접 수정 금지.
  원본: docs/R큐브_프로토콜_BLE_CAN__20260703.xlsx + R큐브_프로토콜_확장_20260728.md
  생성: tools/gen_protocol.py  (2026-07-28)
"""
from enum import IntEnum


class OpCode(IntEnum):
    CmdAck = 0xAF
    NodeAnnounce = 0xD9
    SetMultiroleAggregator = 0xA0
    SetMultiroleInAction = 0xA1
    SetMultiroleVirtualColor = 0xA2
    GetMultiroleVirtualColor = 0xA3
    SetNodeConfig = 0xD3
    GetNodeConfig = 0xD4
    SetEdgeCentralConfig = 0xD5
    GetEdgeCentralConfig = 0xD6
    ResetConfig = 0xD7
    SetAngleLimits = 0xD8
    GetProducts = 0xDA
    SaveParameters = 0xDB
    MissionUploadBegin = 0xF0
    MissionUploadChunk = 0xF1
    MissionUploadCommit = 0xF2
    GetMissionInfo = 0xF3
    DeleteMission = 0xF4
    OtaBegin = 0xF5
    OtaChunk = 0xF6
    OtaCommit = 0xF7
    OtaActivate = 0xF8
    SetSK6812LED = 0xE0
    SetSingleServo = 0xE1
    SetExtDigitalOut = 0xE2
    SetMusicNotes = 0xE3
    SetAggregateMusicNotes = 0xE4
    PlayMusicNotes = 0xE5
    GenerateBuzzerTone = 0xE6
    SetPowerState = 0xE7
    SetSingleSpeed = 0xC0
    SetSingleAngle = 0xC1
    SetScheduledAngles = 0xC2
    PlayScheduledAngles = 0xC3
    SetAggregateAngles = 0xC4
    SetKeyframeStream = 0xC5
    ExecuteBuffer = 0xC7
    MoveToOrigin = 0xC8
    SetThisToOrigin = 0xC9
    SetTargetTorque = 0xCA
    SetDriveState = 0xCB
    SetComplianceMode = 0xCC
    SetMotionLimits = 0xCD
    SetFaultThresholds = 0xCE
    SetControlGains = 0xCF
    SetKinematicModel = 0xEA
    SetWorkOrigin = 0xEB
    MoveToPosition = 0xEC
    SetScheduledPositions = 0xED
    EmergencyStop = 0xD0
    Heartbeat = 0xD1
    TimeSync = 0xD2
    GetSensors = 0xB0
    SetSensorStream = 0xB1
    GetMotorStatus = 0xB2
    GetNodeState = 0xB3
    GetUnitRoster = 0xB4
    GetExtPortMode = 0xB5
    GetPosition = 0xB6
    MotionComplete = 0xB7


class ResultCode(IntEnum):
    OK = 0x00
    BAD_OPCODE = 0x01
    BAD_LENGTH = 0x02
    CRC_FAIL = 0x03
    BAD_PARAM = 0x04
    BAD_STATE = 0x05
    BUFFER_FULL = 0x06
    BUFFER_UNDERRUN = 0x07
    SEQ_GAP = 0x08
    NODE_NOT_FOUND = 0x09
    MOTOR_FAULT = 0x0A
    EXT5V_CONFLICT = 0x0B
    FLASH_FAIL = 0x0C
    MISSION_VER = 0x0D
    ESTOP_ACTIVE = 0x0E
    TIMEOUT = 0x0F
    IK_FAIL = 0x10
    ANGLE_LIMIT = 0x11


# ---- 주소 규약 (APPENDIX A) ----
ADDR_NODE_MIN = 0x01
ADDR_NODE_MAX = 0x08
ADDR_HUB = 0xFE       # 대표(edge central=노드01)
ADDR_BROADCAST = 0xFF

# ---- CmdAck Status ----
ACK = 0x06
NAK = 0x15

# ---- CAN 29비트 ID 레이아웃 (CAN 시트 B절) ----
CAN_SRC_MASTER = 0xFE

def can_id(pri, op, multi, flag, src, dst):
    return ((pri & 0x7) << 26 | (op & 0xFF) << 18 | (multi & 1) << 17 |
            (flag & 1) << 16 | (src & 0xFF) << 8 | (dst & 0xFF))

class CanPri(IntEnum):
    ESTOP = 0
    SAFETY_SYNC = 1
    MOTION = 2
    QUERY = 3
    PERIPHERAL = 4
    CONFIG = 5
    MISSION_OTA = 6
    RESERVED = 7
