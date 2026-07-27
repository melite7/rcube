"""
ble.py — R큐브 BLE Central (PC측) 전송 어댑터. bleak 기반, async.

펌웨어(esp32/main/ble_rcube.c)와 반드시 일치해야 하는 계약:
- 기기 이름: "RCUBE00.00" (광고 Complete Local Name). 접두어 "RCUBE" 로 스캔.
- 커스텀 GATT 서비스 : 52434245-0000-1000-8000-00805f9b34fb  (ASCII "RCBE")
- 명령/상태/알림 특성 : 52434245-0001-1000-8000-00805f9b34fb  (WRITE | READ | NOTIFY)
  → PC는 이 한 특성에 프레임을 write 하고, 같은 특성의 notify 로 회신을 받는다.
"""
from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Callable, Optional

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice

RCUBE_NAME_PREFIX = "RCUBE"
# 연결 대상 이름 접두어: 일반(비고정형/고정형) + 설정모드. 둘 다 스캔·연결한다.
#   RCUBEROBOT.GG.NN = 일반 동작,  RCUBECONFIG.GG.NN = 설정모드 진입 큐브.
RCUBE_NAME_PREFIXES = ("RCUBEROBOT", "RCUBECONFIG")

RCUBE_SVC_UUID = "52434245-0000-1000-8000-00805f9b34fb"
RCUBE_CHR_UUID = "52434245-0001-1000-8000-00805f9b34fb"


@dataclass
class ScanResult:
    name: str
    address: str
    rssi: Optional[int]

    def __str__(self) -> str:
        r = f"{self.rssi} dBm" if self.rssi is not None else "?"
        return f"{self.name}  [{self.address}]  {r}"


# 콜백 타입: (frame_bytes) -> None  / (message) -> None
NotifyCb = Callable[[bytes], None]
LogCb = Callable[[str], None]
StateCb = Callable[[bool], None]  # True=연결됨, False=끊김


class RCubeBLE:
    """단일 R큐브에 대한 BLE 연결 1개를 관리한다.

    사용:
        ble = RCubeBLE(on_notify=..., on_log=..., on_state=...)
        devs = await RCubeBLE.scan()
        await ble.connect(devs[0].address)   # 또는 address=None → 첫 R큐브 자동
        await ble.send(build_frame(...))
        await ble.disconnect()
    """

    def __init__(
        self,
        on_notify: Optional[NotifyCb] = None,
        on_log: Optional[LogCb] = None,
        on_state: Optional[StateCb] = None,
    ) -> None:
        self._on_notify = on_notify
        self._on_log = on_log
        self._on_state = on_state
        self._client: Optional[BleakClient] = None
        self._address: Optional[str] = None

    # ---- 정보 ----
    @property
    def is_connected(self) -> bool:
        return self._client is not None and self._client.is_connected

    @property
    def address(self) -> Optional[str]:
        return self._address

    def _log(self, msg: str) -> None:
        if self._on_log:
            self._on_log(msg)

    # ---- 스캔 ----
    @staticmethod
    async def scan(timeout: float = 5.0, prefix=RCUBE_NAME_PREFIXES) -> list[ScanResult]:
        """주변을 스캔해 이름이 prefix(문자열 또는 튜플)로 시작하는 기기 목록을 돌려준다.

        기본값=RCUBE_NAME_PREFIXES → RCUBEROBOT(일반) + RCUBECONFIG(설정모드) 모두 포함.
        """
        found: dict[str, ScanResult] = {}
        devices = await BleakScanner.discover(timeout=timeout, return_adv=True)
        for _, (dev, adv) in devices.items():  # {address: (BLEDevice, AdvertisementData)}
            name = adv.local_name or dev.name or ""
            if name.startswith(prefix):   # str.startswith 는 튜플도 허용
                found[dev.address] = ScanResult(
                    name=name, address=dev.address, rssi=adv.rssi
                )
        return sorted(found.values(), key=lambda s: (-(s.rssi or -999), s.name))

    # ---- 연결 ----
    async def connect(self, address: Optional[str] = None, timeout: float = 10.0) -> None:
        """address 로 연결한다. None 이면 스캔해서 첫 R큐브에 붙는다."""
        if self.is_connected:
            await self.disconnect()

        target: Optional[str] = address
        if target is None:
            self._log("R큐브 자동 검색 중…")
            results = await self.scan(timeout=5.0)
            if not results:
                raise RuntimeError("R큐브를 찾지 못했습니다. (기기 전원/BOOT버튼 광고 확인)")
            target = results[0].address
            self._log(f"자동 선택: {results[0]}")

        self._log(f"연결 시도: {target}")
        client = BleakClient(target, disconnected_callback=self._handle_disconnect, timeout=timeout)
        await client.connect()
        self._client = client
        self._address = target

        # 특성 존재 확인(펌웨어 계약)
        svc = client.services.get_service(RCUBE_SVC_UUID)
        if svc is None or svc.get_characteristic(RCUBE_CHR_UUID) is None:
            await client.disconnect()
            self._client = None
            raise RuntimeError(
                "연결됐지만 RCBE 서비스/특성이 없습니다. 펌웨어가 R큐브 GATT를 광고하는지 확인하세요."
            )

        await client.start_notify(RCUBE_CHR_UUID, self._handle_notify)
        self._log(f"연결됨: {target} (notify 구독 시작)")
        if self._on_state:
            self._on_state(True)

    async def disconnect(self) -> None:
        client = self._client
        self._client = None
        if client is not None and client.is_connected:
            try:
                await client.stop_notify(RCUBE_CHR_UUID)
            except Exception:
                pass
            await client.disconnect()
        # 상태 통지는 _handle_disconnect 에서 처리되지만, 수동 종료도 반영
        if self._on_state:
            self._on_state(False)

    # ---- 송신 ----
    async def send(self, frame: bytes, *, response: bool = True) -> None:
        """이미 만들어진 와이어 프레임(bytes)을 명령 특성에 write."""
        if not self.is_connected or self._client is None:
            raise RuntimeError("연결되어 있지 않습니다.")
        await self._client.write_gatt_char(RCUBE_CHR_UUID, bytes(frame), response=response)
        self._log(f"TX  {bytes(frame).hex(' ').upper()}")

    async def read(self) -> bytes:
        """상태 특성 READ (펌웨어는 현재 1바이트 상태 반환)."""
        if not self.is_connected or self._client is None:
            raise RuntimeError("연결되어 있지 않습니다.")
        data = await self._client.read_gatt_char(RCUBE_CHR_UUID)
        return bytes(data)

    # ---- 내부 콜백 ----
    def _handle_notify(self, _sender, data: bytearray) -> None:
        raw = bytes(data)
        self._log(f"RX  {raw.hex(' ').upper()}")
        if self._on_notify:
            self._on_notify(raw)

    def _handle_disconnect(self, _client: BleakClient) -> None:
        self._log("연결이 끊어졌습니다.")
        self._client = None
        if self._on_state:
            self._on_state(False)
