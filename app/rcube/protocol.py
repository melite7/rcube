"""
protocol.py — R큐브 표준 패킷 프레이밍 (BLE/CAN 공용 응용계층).

와이어 프레임 (shared-protocol/rcube_protocol.h 의 rcube_header_t):
    [0]   TargetId      대상 주소 (ADDR_* 규약)
    [1]   OpCode        rcube_protocol.OpCode
    [2:3] PacketSize    전체 패킷 길이(헤더4 + payload), uint16 **big-endian**
    [4:]  Property+Value  명령별 페이로드 (= CAN 데이터필드와 동일)

OpCode/ResultCode/주소 규약은 shared-protocol(단일 소스)에서 그대로 가져온다.
값을 여기 손으로 옮겨 적지 않는다(로드맵 Phase 0 원칙).
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

# ---- shared-protocol(단일 소스) 로드 ---------------------------------------
# 리포 레이아웃: <root>/app/rcube/protocol.py, <root>/shared-protocol/rcube_protocol.py
_SHARED = Path(__file__).resolve().parents[2] / "shared-protocol"
if _SHARED.is_dir() and str(_SHARED) not in sys.path:
    sys.path.insert(0, str(_SHARED))

try:
    from rcube_protocol import (  # type: ignore
        OpCode,
        ResultCode,
        ADDR_NODE_MIN,
        ADDR_NODE_MAX,
        ADDR_HUB,
        ADDR_BROADCAST,
    )
except ImportError as exc:  # pragma: no cover - 배치 오류시 명확한 안내
    raise ImportError(
        f"shared-protocol을 찾을 수 없습니다({_SHARED}). "
        "리포 구조가 <root>/app, <root>/shared-protocol 인지 확인하세요."
    ) from exc

HEADER_LEN = 4
MAX_PACKET = 0xFFFF  # PacketSize 가 uint16 이므로 상한


@dataclass(frozen=True)
class Frame:
    """디코딩된 한 개의 R큐브 프레임."""

    target_id: int
    op_code: int
    payload: bytes

    @property
    def op_name(self) -> str:
        try:
            return OpCode(self.op_code).name
        except ValueError:
            return f"0x{self.op_code:02X}"

    def __str__(self) -> str:
        body = self.payload.hex(" ").upper() if self.payload else "-"
        return (f"target=0x{self.target_id:02X} op={self.op_name}"
                f"(0x{self.op_code:02X}) len={len(self.payload)} [{body}]")


def build_frame(target_id: int, op_code: int, payload: bytes = b"") -> bytes:
    """표준 헤더 + 페이로드로 와이어 프레임(bytes)을 만든다.

    PacketSize 는 헤더 포함 전체 길이이며 big-endian 으로 실린다.
    """
    op_code = int(op_code)  # OpCode(IntEnum) 도 허용
    if not 0 <= target_id <= 0xFF:
        raise ValueError(f"target_id 범위 초과: {target_id}")
    if not 0 <= op_code <= 0xFF:
        raise ValueError(f"op_code 범위 초과: {op_code}")

    total = HEADER_LEN + len(payload)
    if total > MAX_PACKET:
        raise ValueError(f"패킷이 너무 큼: {total} > {MAX_PACKET}")

    header = bytes((target_id & 0xFF, op_code & 0xFF, (total >> 8) & 0xFF, total & 0xFF))
    return header + bytes(payload)


def parse_frame(data: bytes) -> Frame:
    """와이어 프레임(bytes)을 Frame 으로 디코딩한다.

    PacketSize 필드는 검증하되, 실제 수신 길이를 신뢰해 페이로드를 잘라낸다
    (BLE MTU 조각화/여분 바이트 방어).
    """
    if len(data) < HEADER_LEN:
        raise ValueError(f"프레임이 헤더보다 짧음: {len(data)} bytes")

    target_id = data[0]
    op_code = data[1]
    declared = (data[2] << 8) | data[3]

    if declared and declared != len(data):
        # 불일치는 치명 오류로 보지 않고 payload 는 실제 수신분으로 처리.
        # (호출부에서 필요하면 declared 로 재검증)
        pass

    return Frame(target_id=target_id, op_code=op_code, payload=bytes(data[HEADER_LEN:]))


# =====================================================================
# 명령별 프레임 빌더 (프로토콜 xlsx의 payload 스펙 기준)
#   ※ 아직 펌웨어가 파싱하지 않는 명령이 많다. 여기의 바이트 레이아웃이
#     esp32 파서와 맞춰야 할 "계약"이다(출처: docs/…프로토콜….xlsx).
# =====================================================================

# ---- 자주 쓰는 색상 (순수 색상; 실제 밝기는 큐브 board_led가 스케일) ----
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
OFF = (0, 0, 0)

# ---- SetMultiroleAggregator GroupMode (CONFIG 시트 A0) ----
GROUP_DISABLED = 0x0A   # 그룹번호 무관하게 연결(비고정형)
GROUP_ENABLED = 0x1A    # 같은 그룹번호만 연결

# 아그리게이터 응답/멤버연결 알림 OpCode (A0 처리 후 A1로 회신)
OP_AGGREGATOR_EVENT = int(OpCode.SetMultiroleInAction)  # 0xA1


def build_set_led(target_id: int, leds) -> bytes:
    """SetSK6812LED(0xE0). payload = [LED개수 n][R,G,B]×n.

    leds: (r,g,b) 튜플의 순서열. 큐브의 LED 0..n-1 에 순서대로 적용.
    (프로토콜: 큐브당 SK6812 3개 — 2개 노드ID, 1개 그룹번호 표시용)
    """
    leds = list(leds)
    payload = bytearray([len(leds) & 0xFF])
    for (r, g, b) in leds:
        payload += bytes((r & 0xFF, g & 0xFF, b & 0xFF))
    return build_frame(target_id, OpCode.SetSK6812LED, bytes(payload))


def build_set_led_solid(target_id: int, rgb, n: int = 3) -> bytes:
    """큐브의 LED n개를 모두 같은 색으로. (전체 색상 = 시각적 상태 표시)"""
    r, g, b = rgb
    return build_set_led(target_id, [(r, g, b)] * n)


# ---- SetNodeConfig(0xD3) 서브커맨드 (펌웨어 rcube_cmd.h와 일치) ----
# ※ 0x02(FIX_ORDER)는 기획서 7.5 [새 방식]에서 폐기 — 노드ID 저장 여부가 곧 고정형
#   판정이라 별도의 고정형 전환 명령이 없다(SET_NETCONF 저장이 그 역할을 한다).
D3_SUB_SET_GROUP = 0x01   # payload=[0x01, group_id] : 그룹 저장 후 재부팅
D3_SUB_SET_NODE = 0x03    # payload=[0x03, node_id]  : 노드ID 저장 후 재부팅
D3_SUB_SET_NETCONF = 0x04 # payload=[0x04, node_id, cmf, term_id] : 통신방식 세팅 저장(재부팅 X)
D3_SUB_REBOOT = 0x05      # payload=[0x05]           : 재부팅(브로드캐스트=전체)


def build_set_group(group_id: int, *, target_id: int = ADDR_BROADCAST) -> bytes:
    """SetNodeConfig/SET_GROUP. 그룹번호를 저장시키고 큐브를 재부팅시킨다.

    payload = [SET_GROUP, group_id]. 기본 브로드캐스트(0xFF): 연결된 큐브가
    아그리게이터면 전 멤버로 중계 → 모든 큐브가 저장 후 재부팅.
    """
    if not 0 <= group_id <= 99:
        raise ValueError(f"group_id 범위(0~99) 초과: {group_id}")
    payload = bytes((D3_SUB_SET_GROUP, group_id & 0xFF))
    return build_frame(target_id, OpCode.SetNodeConfig, payload)


# ---- 통신방식(CMF) ----
CMF_BLE = 0x00
CMF_CAN = 0x01


def build_set_netconf(node_id: int, cmf: int, term_id: int = 0,
                      *, target_id: int = ADDR_HUB) -> bytes:
    """SetNodeConfig/SET_NETCONF. 통신방식 세팅 저장(기획서 7.2). 재부팅은 별도.

    payload = [SET_NETCONF, node_id, cmf(0=BLE/1=CAN), term_id]
    - node_id : 이 큐브의 노드ID(현재 연결 순서값). 저장 순간 고정형이 된다.
    - cmf     : 다음 부팅부터 사용할 통신방식.
    - term_id : (CAN 큐브만 의미) 종단노드ID = 유닛의 CAN 큐브 중 최대 노드ID.
    target_id : 아그리게이터=0xFE, 멤버=가상ID(아그리게이터가 중계).
    """
    return build_frame(target_id, OpCode.SetNodeConfig,
                       bytes((D3_SUB_SET_NETCONF, node_id & 0xFF, cmf & 0xFF, term_id & 0xFF)))


def build_reboot_all(*, target_id: int = ADDR_BROADCAST) -> bytes:
    """SetNodeConfig/REBOOT. 기본 브로드캐스트: 연결된 큐브+멤버 전체 재부팅(기획서 7.2 step4)."""
    return build_frame(target_id, OpCode.SetNodeConfig, bytes((D3_SUB_REBOOT,)))


def build_get_node_config(*, target_id: int = ADDR_HUB) -> bytes:
    """GetNodeConfig(0xD4). 저장된 설정을 조회한다.

    회신 payload = [group_id, node_id, cmf, term_id]. 멤버(target=가상ID)에 보내면
    BLE 허브가 중계하고, 멤버의 회신 notify를 허브가 PC로 되돌려 준다.
    통신방식 세팅(7.2-2)이 각 큐브에 실제로 저장됐는지 확인하는 용도.
    """
    return build_frame(target_id, OpCode.GetNodeConfig, b"")


# ---- 센서 모니터링 (기획서 9장) — 펌웨어 rcube_sensor.h와 일치 ----
SENSOR_KIND_ACCEL = 0x00   # 가속도, 단위 mg
SENSOR_KIND_GYRO = 0x01    # 자이로, 단위 0.1°/s
SENSOR_PERIOD_DEFAULT_MS = 200


def build_get_sensors(*, target_id: int = ADDR_HUB) -> bytes:
    """GetSensors(0xB0). 지금 1회만 센서를 올리게 한다.

    회신 payload = [kind][x][y][z] (int16 big-endian ×3, 총 7B). 가속도·자이로
    2프레임으로 나뉘어 온다. 7바이트인 이유는 CAN 데이터필드(8B)에 멀티프레임 없이
    들어가야 하기 때문이다.
    """
    return build_frame(target_id, OpCode.GetSensors, b"")


def build_set_sensor_stream(on: bool, period_ms: int = SENSOR_PERIOD_DEFAULT_MS,
                            *, target_id: int = ADDR_BROADCAST) -> bytes:
    """SetSensorStream(0xB1). 주기 전송 시작/중지 (기획서 9장 "센서 전송 시작 명령").

    payload = [on, period_hi, period_lo]. 기본 브로드캐스트(0xFF): BLE 허브가 전
    멤버로 중계한 뒤 자신도 적용한다 — 허브도 유닛의 한 큐브이므로 자기 센서를 함께 올린다.
    """
    return build_frame(target_id, OpCode.SetSensorStream,
                       bytes((1 if on else 0, (period_ms >> 8) & 0xFF, period_ms & 0xFF)))


def parse_sensor_payload(payload: bytes):
    """센서 payload(7B)를 (kind, x, y, z)로. 형식이 아니면 None."""
    if len(payload) < 7:
        return None
    kind = payload[0]

    def i16(hi, lo):
        v = (hi << 8) | lo
        return v - 0x10000 if v & 0x8000 else v

    return kind, i16(payload[1], payload[2]), i16(payload[3], payload[4]), i16(payload[5], payload[6])


# ---- 멤버 맵 (기획서 7.3-3 ★보강) — 펌웨어 rcube_config.h와 일치 ----
MAX_NODES = 8
MEMBER_NONE = 0xFF   # 그 노드ID는 유닛에 없음
MEMBER_BLE = 0x00
MEMBER_CAN = 0x01


def build_member_map(cmf_by_node: dict) -> bytes:
    """{노드ID: CMF} → 8바이트 멤버 맵(인덱스 i = 노드ID i+1)."""
    m = bytearray([MEMBER_NONE] * MAX_NODES)
    for node_id, cmf in cmf_by_node.items():
        nid = int(node_id)
        if not 1 <= nid <= MAX_NODES:
            raise ValueError(f"노드ID 범위(1~{MAX_NODES}) 초과: {nid}")
        m[nid - 1] = MEMBER_CAN if int(cmf) else MEMBER_BLE
    return bytes(m)


def build_set_edge_central(ecf: int, unit_count: int, term_id: int, member_map: bytes,
                           *, target_id: int = ADDR_HUB) -> bytes:
    """SetEdgeCentralConfig(0xD5). 독립로봇유닛 전환/강등 (기획서 7.3-3).

    payload = [ecf, unit_n, term_id, map[8]]
    - ecf=1 : 이 큐브를 edge central(리드 큐브)로. 멤버 맵·N·종단노드ID를 함께 저장.
    - ecf=0 : 강등(일반 큐브로 복귀). 멤버 맵도 삭제된다.
    저장만 하고 재부팅하지 않는다 — 배선 정리를 위해 이어서 shutdown(E7)을 보낸다.
    """
    if len(member_map) != MAX_NODES:
        raise ValueError(f"member_map은 {MAX_NODES}바이트여야 합니다: {len(member_map)}")
    payload = bytes((ecf & 0xFF, unit_count & 0xFF, term_id & 0xFF)) + bytes(member_map)
    return build_frame(target_id, OpCode.SetEdgeCentralConfig, payload)


def build_get_edge_central(*, target_id: int = ADDR_HUB) -> bytes:
    """GetEdgeCentralConfig(0xD6). 회신 payload = [ecf, unit_n, term_id, map[8]]."""
    return build_frame(target_id, OpCode.GetEdgeCentralConfig, b"")


def build_shutdown(*, target_id: int = ADDR_BROADCAST) -> bytes:
    """SetPowerState(0xE7) payload=[0] = shut down. 기획서 7.3-4.

    기본 브로드캐스트: 허브가 전 멤버로 중계한 뒤 자신도 끈다. 개발보드에는 전원 차단
    회로가 없어 펌웨어가 딥슬립으로 대신하며, BOOT 버튼으로 다시 깨어난다.
    """
    return build_frame(target_id, OpCode.SetPowerState, bytes((0x00,)))


def build_reset_config(*, target_id: int = ADDR_BROADCAST) -> bytes:
    """ResetConfig(0xD7). 공장 초기화(노드ID=0, CMF=BLE, 종단ID=0) 후 재부팅.

    기획서 7.3 [노드ID/세팅 초기화 - 공통]. 노드ID=0이 되므로 비고정형으로 돌아간다.
    기본 브로드캐스트(0xFF): 허브가 전 멤버로 중계한 뒤 자신도 초기화한다.
    """
    return build_frame(target_id, OpCode.ResetConfig, b"")


def build_set_aggregator(
    connection_link_count: int,
    *,
    group_enabled: bool = False,
    virtual_ids=None,
    target_id: int = ADDR_HUB,
) -> bytes:
    """SetMultiroleAggregator(0xA0). 이 큐브를 BLE 허브로 승격.

    payload = [ConnectionLinkCount][GroupMode][Flags] (+ 고정형이면 VirtualCubeId 4B×n)
    - connection_link_count : 연결 대상 큐브 수(계약상 허브 포함 총 N)
    - group_enabled=False   : GROUP_DISABLED(0x0A) 그룹 무관 연결

    ※ 고정형/비고정형은 PC가 지시하지 않는다. 기획서 7.5 [새 방식]대로 허브 큐브가
      자기 저장 노드ID 유무로 스스로 판정한다(Flags는 0으로 예약).
    """
    mode = GROUP_ENABLED if group_enabled else GROUP_DISABLED
    payload = bytearray((connection_link_count & 0xFF, mode, 0x00))
    if virtual_ids:
        for vid in virtual_ids:
            payload += int(vid).to_bytes(4, "big")
    return build_frame(target_id, OpCode.SetMultiroleAggregator, bytes(payload))
