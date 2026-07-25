"""
can.py — R큐브 CAN 전송 어댑터 (PC측, USB-CAN 어댑터). python-can 기반.

BLE(ble.py)와 같은 콜백 인터페이스(on_notify/on_log/on_state)를 제공해 GUI가
동일하게 다룰 수 있다. 다양한 USB-CAN 어댑터를 지원하도록 python-can의
interface/channel/bitrate를 그대로 노출한다(예: slcan+COM4, pcan+PCAN_USBBUS1).

와이어(shared-protocol CAN 시트):
- 29비트 확장 ID = [28:26]Priority [25:18]OpCode [17]MULTI [16]FLAG [15:8]Src [7:0]Dst
- 데이터필드(≤8B) = 표준프레임의 payload(Property+Value).
- PC(마스터)는 src=0xFE(CAN_SRC_MASTER)로 dst(노드ID/브로드캐스트)에 송신.

수신: 노드가 보낸 프레임을 표준프레임 [Src][OpCode][PacketSize BE][data]로 재구성해
on_notify로 올린다(app은 src로 어느 노드인지 식별). 부팅/하트비트(NodeAnnounce/
Heartbeat)를 수신하면 노드 목록에 등록 → discover()로 조회.
"""
from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import can  # python-can (site-packages)

from .protocol import OpCode, parse_frame, HEADER_LEN

# shared-protocol(단일 소스) — protocol.py가 이미 sys.path에 등록해 둠.
from rcube_protocol import can_id, CanPri, CAN_SRC_MASTER  # type: ignore

NotifyCb = Callable[[bytes], None]
LogCb = Callable[[str], None]
StateCb = Callable[[bool], None]

# 기본 비트레이트(펌웨어 can_transport.c 와 일치: 500kbit).
DEFAULT_BITRATE = 500000

# python-can 인터페이스 예시(어댑터에 맞게 선택).
KNOWN_INTERFACES = ["slcan", "pcan", "kvaser", "ixxat", "vector", "usb2can", "socketcan", "virtual"]


def default_priority(op: int) -> int:
    """OpCode → CAN 우선순위(낮을수록 우선). shared-protocol 우선순위 클래스와 정렬."""
    if op == int(OpCode.EmergencyStop):
        return int(CanPri.ESTOP)
    if op in (int(OpCode.Heartbeat), int(OpCode.TimeSync), int(OpCode.NodeAnnounce)):
        return int(CanPri.SAFETY_SYNC)
    if 0xC0 <= op <= 0xCF:
        return int(CanPri.MOTION)
    if op == int(OpCode.CmdAck) or 0xB0 <= op <= 0xB6:
        return int(CanPri.QUERY)
    if (0xE0 <= op <= 0xE7) or (0xEA <= op <= 0xED):
        return int(CanPri.PERIPHERAL)
    if 0xF0 <= op <= 0xF8:
        return int(CanPri.MISSION_OTA)
    if (0xD3 <= op <= 0xDB):
        return int(CanPri.CONFIG)
    return int(CanPri.QUERY)


def _can_id_fields(arb_id: int):
    """29비트 arbitration id 에서 (pri, op, multi, flag, src, dst) 추출."""
    return (
        (arb_id >> 26) & 0x7,
        (arb_id >> 18) & 0xFF,
        (arb_id >> 17) & 0x1,
        (arb_id >> 16) & 0x1,
        (arb_id >> 8) & 0xFF,
        arb_id & 0xFF,
    )


class RCubeCAN:
    """USB-CAN 어댑터를 통한 R큐브 CAN 버스 접속(1개)."""

    def __init__(
        self,
        on_notify: Optional[NotifyCb] = None,
        on_log: Optional[LogCb] = None,
        on_state: Optional[StateCb] = None,
    ) -> None:
        self._on_notify = on_notify
        self._on_log = on_log
        self._on_state = on_state
        self._bus: Optional[can.BusABC] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._nodes: set[int] = set()
        self._lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        return self._bus is not None

    def _log(self, msg: str) -> None:
        if self._on_log:
            self._on_log(msg)

    # ---- 버스 열기/닫기 ----
    def open(self, interface: str, channel: str, bitrate: int = DEFAULT_BITRATE) -> None:
        """USB-CAN 버스를 연다. interface/channel 은 python-can 규약."""
        if self._bus is not None:
            self.close()
        kwargs = {"interface": interface, "channel": channel}
        # virtual 은 bitrate 인자를 받지 않음.
        if interface != "virtual":
            kwargs["bitrate"] = bitrate
        self._bus = can.Bus(**kwargs)
        self._stop.clear()
        with self._lock:
            self._nodes.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        self._log(f"CAN 버스 열림: {interface}/{channel} @ {bitrate}bps")
        if self._on_state:
            self._on_state(True)

    def close(self) -> None:
        self._stop.set()
        t = self._rx_thread
        self._rx_thread = None
        if t is not None and t.is_alive():
            t.join(timeout=1.0)
        bus = self._bus
        self._bus = None
        if bus is not None:
            try:
                bus.shutdown()
            except Exception:
                pass
        self._log("CAN 버스 닫힘")
        if self._on_state:
            self._on_state(False)

    # ---- 송신 ----
    def send(self, frame: bytes, *, priority: Optional[int] = None) -> None:
        """표준 프레임(bytes)을 CAN으로 송신. target=dst, op/payload는 프레임에서 추출."""
        if self._bus is None:
            raise RuntimeError("CAN 버스가 열려 있지 않습니다.")
        fr = parse_frame(frame)
        pri = default_priority(fr.op_code) if priority is None else priority
        if len(fr.payload) > 8:
            # Classic CAN 데이터필드는 8바이트 — 초과분은 멀티프레임 필요(미구현).
            raise ValueError(f"payload {len(fr.payload)}B > 8B (CAN 멀티프레임 미지원)")
        arb = can_id(pri, fr.op_code, 0, 0, CAN_SRC_MASTER, fr.target_id)
        msg = can.Message(arbitration_id=arb, is_extended_id=True, data=bytes(fr.payload))
        self._bus.send(msg)
        self._log(f"TX  can id=0x{arb:08X} op={fr.op_name} dst=0x{fr.target_id:02X} "
                  f"[{fr.payload.hex(' ').upper() if fr.payload else '-'}]")

    # ---- 노드 검색(하트비트/부팅 수집) ----
    def discover(self, timeout: float = 2.0) -> list[int]:
        """timeout 동안 버스에서 하트비트/부팅 메시지를 수집해 노드ID 목록을 돌려준다."""
        with self._lock:
            self._nodes.clear()
        time.sleep(timeout)
        with self._lock:
            return sorted(self._nodes)

    # ---- 수신 루프 ----
    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                msg = self._bus.recv(timeout=0.5)
            except Exception:
                break
            if msg is None:
                continue
            if not msg.is_extended_id:
                continue
            pri, op, multi, flag, src, dst = _can_id_fields(msg.arbitration_id)
            data = bytes(msg.data)

            # 부팅/하트비트 → 노드 목록 등록.
            if op in (int(OpCode.Heartbeat), int(OpCode.NodeAnnounce)):
                with self._lock:
                    self._nodes.add(src)

            # 표준프레임 [Src][Op][PacketSize BE][data] 로 재구성해 상위로.
            total = HEADER_LEN + len(data)
            frame = bytes((src, op, (total >> 8) & 0xFF, total & 0xFF)) + data
            self._log(f"RX  can id=0x{msg.arbitration_id:08X} op=0x{op:02X} src=0x{src:02X} "
                      f"dst=0x{dst:02X} [{data.hex(' ').upper() if data else '-'}]")
            if self._on_notify:
                self._on_notify(frame)
