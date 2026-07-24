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


def build_set_node_config(group_id: int, *, target_id: int = ADDR_BROADCAST) -> bytes:
    """SetNodeConfig(0xD3). 노드 영구설정 갱신 — 지금은 그룹번호만.

    payload = [group_id]  (계약: 1바이트. 추후 node_id 등 확장)
    target_id 기본=브로드캐스트(0xFF): 연결된 큐브가 아그리게이터면 전 멤버로 중계된다.
    큐브는 그룹번호를 플래시에 저장한 뒤 스스로 재부팅한다.
    """
    if not 0 <= group_id <= 0xFF:
        raise ValueError(f"group_id 범위 초과: {group_id}")
    return build_frame(target_id, OpCode.SetNodeConfig, bytes((group_id & 0xFF,)))


def build_set_aggregator(
    connection_link_count: int,
    *,
    group_enabled: bool = False,
    virtual_ids=None,
    target_id: int = ADDR_HUB,
) -> bytes:
    """SetMultiroleAggregator(0xA0). 이 큐브를 BLE 허브(아그리게이터)로 승격.

    payload = [ConnectionLinkCount][GroupMode] (+ 고정형이면 VirtualCubeId 4B×n)
    - connection_link_count : 연결 대상 큐브 수(계약상 아그리게이터 포함 총 N)
    - group_enabled=False   : GROUP_DISABLED(0x0A) 비고정형(그룹 무관)
    - virtual_ids           : 고정형일 때만. 각 4바이트(big-endian).
    """
    mode = GROUP_ENABLED if group_enabled else GROUP_DISABLED
    payload = bytearray((connection_link_count & 0xFF, mode))
    if virtual_ids:
        for vid in virtual_ids:
            payload += int(vid).to_bytes(4, "big")
    return build_frame(target_id, OpCode.SetMultiroleAggregator, bytes(payload))
