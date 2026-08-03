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
KNOWN_INTERFACES = ["pcan", "slcan", "kvaser", "ixxat", "vector", "usb2can", "socketcan", "virtual"]

# 기본 어댑터: PEAK PCAN-USB opto(IPEH-002022). 채널명은 PCAN-Basic 규약.
DEFAULT_INTERFACE = "pcan"
DEFAULT_CHANNEL = "PCAN_USBBUS1"


def detect_adapters(interfaces=("pcan",)) -> list[dict]:
    """꽂혀 있는 USB-CAN 어댑터를 찾아 [{interface, channel, ...}] 로 돌려준다.

    기본값을 pcan 하나로 좁혀 둔다. 다른 백엔드까지 훑으면 python-can이 각 벤더
    DLL을 import하면서 "Kvaser canlib is unavailable" 같은 경고를 쏟아내, 정작
    중요한 메시지가 묻힌다. 다른 어댑터를 쓸 때 인자로 넘기면 된다.

    python-can 백엔드는 드라이버 DLL이 없으면 조용히 빈 목록을 준다(PCAN 백엔드는
    PCANBasic() 생성 시 OSError를 잡아 [] 반환). 즉 "빈 목록"은 '어댑터 없음'과
    '드라이버 미설치'를 구분해 주지 않으므로, 호출부는 pcan_driver_installed()로
    둘을 나눠 안내해야 한다.
    """
    try:
        return list(can.detect_available_configs(interfaces=list(interfaces)))
    except Exception:
        return []


def pcan_driver_installed() -> bool:
    """PEAK PCAN-Basic API(PCANBasic.dll)가 설치되어 있는지.

    PEAK 드라이버 설치 시 'PCAN-Basic API' 항목을 함께 선택해야 깔린다. 드라이버만
    깔고 이 항목을 빼면 PCAN-View 같은 PEAK 도구는 되는데 python-can만 안 되는,
    원인을 찾기 어려운 상태가 된다.
    """
    try:
        from can.interfaces.pcan.pcan import PCANBasic  # type: ignore
        PCANBasic()
        return True
    except Exception:
        return False


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


# ---- 멀티프레임 (확장 규격 §5) — 펌웨어 can_transport.c와 동일 규격 ----
SEG_FIRST = 0x80
SEG_LAST = 0x40
SEG_INDEX_MASK = 0x3F
SEG_MAX_INDEX = 0x3F
SEG_FIRST_DATA = 5      # FIRST 세그먼트가 싣는 실데이터(헤더1 + 길이2 = 3바이트 소비)
SEG_DATA = 7
REASSEMBLY_MAX = SEG_FIRST_DATA + SEG_MAX_INDEX * SEG_DATA   # 446
REASSEMBLY_TIMEOUT = 0.2   # 초


def split_multiframe(payload: bytes) -> list[bytes]:
    """페이로드를 §5 세그먼트 목록으로 나눈다.

    FIRST : [hdr][전체길이 BE16][데이터 5B]
    이후   : [hdr][데이터 7B]
    hdr    : bit7=FIRST, bit6=LAST, bit5:0=순번
    """
    n = len(payload)
    if n > REASSEMBLY_MAX:
        raise ValueError(f"payload {n}B > 멀티프레임 최대 {REASSEMBLY_MAX}B")
    segs = []
    chunk = min(SEG_FIRST_DATA, n)
    last = chunk >= n
    hdr = SEG_FIRST | (SEG_LAST if last else 0) | 0
    segs.append(bytes((hdr, (n >> 8) & 0xFF, n & 0xFF)) + payload[:chunk])
    sent = chunk
    index = 0
    while sent < n:
        index += 1
        if index > SEG_MAX_INDEX:
            raise ValueError("세그먼트 순번 초과")
        chunk = min(SEG_DATA, n - sent)
        last = (sent + chunk) >= n
        segs.append(bytes(((SEG_LAST if last else 0) | index,)) + payload[sent:sent + chunk])
        sent += chunk
    return segs


class Reassembler:
    """(src, op)별 멀티프레임 재조립. 순번 불일치·타임아웃이면 통째로 버린다."""

    def __init__(self):
        self._st = {}   # (src, op) → dict

    def expire(self, now: Optional[float] = None) -> None:
        now = time.monotonic() if now is None else now
        for key in [k for k, v in self._st.items() if now - v["t"] > REASSEMBLY_TIMEOUT]:
            del self._st[key]

    def feed(self, src: int, op: int, data: bytes) -> Optional[bytes]:
        """세그먼트 1개 투입. 완성되면 페이로드, 아니면 None."""
        if not data:
            return None
        hdr = data[0]
        index = hdr & SEG_INDEX_MASK
        key = (src, op)

        if hdr & SEG_FIRST:
            if len(data) < 3:
                return None
            total = (data[1] << 8) | data[2]
            if total == 0 or total > REASSEMBLY_MAX:
                return None
            self._st[key] = {"total": total, "buf": bytearray(data[3:3 + total]),
                             "next": 1, "t": time.monotonic()}
        else:
            st = self._st.get(key)
            if st is None or index != st["next"]:
                self._st.pop(key, None)   # FIRST 누락 또는 순번 어긋남 → 폐기
                return None
            st["buf"] += data[1:1 + (st["total"] - len(st["buf"]))]
            st["next"] += 1

        if not (hdr & SEG_LAST):
            return None
        st = self._st.pop(key, None)
        if st is None or len(st["buf"]) != st["total"]:
            return None
        return bytes(st["buf"])


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
        self._reasm = Reassembler()   # 멀티프레임 재조립(확장 규격 §5)

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
        try:
            self._bus = can.Bus(**kwargs)
        except Exception as e:
            # python-can이 DLL 부재/장치 미연결에 대해 던지는 예외가 불친절해서
            # (OSError: [WinError 126] 등) 원인별로 다시 포장한다.
            if interface == "pcan":
                if not pcan_driver_installed():
                    raise RuntimeError(
                        "PEAK PCAN-Basic API(PCANBasic.dll)를 찾을 수 없습니다. "
                        "PEAK-System 'PCAN-Driver for Windows'를 설치하되 설치 옵션에서 "
                        "'PCAN-Basic API'를 반드시 함께 선택하세요."
                    ) from e
                # 드라이버는 있는데 채널이 없다 = 어댑터 미연결 또는 채널번호 불일치.
                # PCAN-Basic의 원문("The value of a handle ... is invalid")으로는
                # 무엇을 해야 하는지 알 수 없다.
                if "handle" in str(e).lower():
                    raise RuntimeError(
                        f"PCAN 채널 '{channel}'을 열 수 없습니다 — 어댑터가 USB에 연결되어 "
                        "있지 않거나 채널번호가 다릅니다. '장치검색'으로 실제 채널을 확인하세요."
                    ) from e
            raise
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

    # ---- 버스 상태 ----
    def state_text(self) -> str:
        """버스 상태를 사람이 읽는 문자열로. 배선 진단의 1차 지표다.

        CAN은 수신 확인(ACK)이 없으면 송신이 성립하지 않는다. 종단저항 누락·비트레이트
        불일치·CANH/CANL 뒤바뀜은 모두 '송신은 했는데 아무 일도 없음'으로 보이고,
        내부적으로는 에러카운터가 올라가다 error-passive → bus-off로 간다. 이 값을
        보면 "배선 문제"와 "펌웨어가 응답을 안 함"을 구분할 수 있다.
        """
        if self._bus is None:
            return "닫힘"
        try:
            st = self._bus.state
        except Exception:
            return "정상(상태조회 미지원 어댑터)"
        name = getattr(st, "name", str(st))
        return {
            "ACTIVE": "error-active(정상)",
            "PASSIVE": "error-passive — 에러 누적. 종단저항·비트레이트·결선 확인",
            "ERROR": "BUS-OFF — 버스 이상. 종단 120Ω 양끝, 500kbit 일치, CANH/CANL 확인",
        }.get(name, name)

    # ---- 송신 ----
    def send(self, frame: bytes, *, priority: Optional[int] = None) -> None:
        """표준 프레임(bytes)을 CAN으로 송신. target=dst, op/payload는 프레임에서 추출.

        payload가 8바이트를 넘으면 확장 규격 §5 멀티프레임(MULTI=1)으로 분할한다.
        """
        if self._bus is None:
            raise RuntimeError("CAN 버스가 열려 있지 않습니다.")
        fr = parse_frame(frame)
        pri = default_priority(fr.op_code) if priority is None else priority
        payload = bytes(fr.payload)

        if len(payload) <= 8:
            arb = can_id(pri, fr.op_code, 0, 0, CAN_SRC_MASTER, fr.target_id)
            self._bus.send(can.Message(arbitration_id=arb, is_extended_id=True, data=payload))
            self._log(f"TX  can id=0x{arb:08X} op={fr.op_name} dst=0x{fr.target_id:02X} "
                      f"[{payload.hex(' ').upper() if payload else '-'}]")
            return

        arb = can_id(pri, fr.op_code, 1, 0, CAN_SRC_MASTER, fr.target_id)
        for seg in split_multiframe(payload):
            self._bus.send(can.Message(arbitration_id=arb, is_extended_id=True, data=seg))
        self._log(f"TX  can id=0x{arb:08X} op={fr.op_name} dst=0x{fr.target_id:02X} "
                  f"멀티프레임 {len(payload)}B → {len(split_multiframe(payload))}세그먼트")

    # ---- 노드 검색(하트비트/부팅 수집) ----
    def discover(self, timeout: float = 2.0) -> list[int]:
        """timeout 동안 버스에서 하트비트/부팅 메시지를 수집해 노드ID 목록을 돌려준다."""
        with self._lock:
            self._nodes.clear()
        time.sleep(timeout)
        with self._lock:
            return sorted(self._nodes)

    # ---- 고정형 순서 연결 (기획서 7.2 [CAN 분기]) ----
    def connect_ordered(self, timeout: float = 3.0, per_node=None) -> list[int]:
        """CAN 버스의 고정형 큐브를 노드ID 오름차순으로 '연결'한다.

        CAN은 BLE 같은 연결 핸드셰이크가 없다. 각 CAN 큐브는 부팅 시 자기 노드ID
        색으로 자가점등하며 하트비트를 발행하므로, PC는 (1) 하트비트로 존재 노드를
        발견하고 (2) 노드ID 오름차순으로 순회하며 확인한다(기획서 7.2).

        per_node(node_id, index)를 노드ID 순서대로 호출(색 확인 명령 전송 등).
        발견·정렬된 노드ID 목록(오름차순)을 반환한다.
        """
        nodes = self.discover(timeout)   # discover()는 오름차순 정렬 반환
        for i, nid in enumerate(nodes):
            if per_node is not None:
                per_node(nid, i)
        return nodes

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

            if multi:
                # MULTI=1 → 세그먼트. 다 모여야 상위로 올린다(확장 규격 §5).
                self._reasm.expire()
                data = self._reasm.feed(src, op, data)
                if data is None:
                    continue

            # 표준프레임 [Src][Op][PacketSize BE][data] 로 재구성해 상위로.
            total = HEADER_LEN + len(data)
            frame = bytes((src, op, (total >> 8) & 0xFF, total & 0xFF)) + data
            self._log(f"RX  can id=0x{msg.arbitration_id:08X} op=0x{op:02X} src=0x{src:02X} "
                      f"dst=0x{dst:02X} [{data.hex(' ').upper() if data else '-'}]")
            if self._on_notify:
                self._on_notify(frame)
