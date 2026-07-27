"""
gui.py — R큐브 BLE 제어 UI (tkinter, 추가 의존성 없음).

■ 주 기능: R1 / R2 / R3 / R4 시나리오 버튼
  R1: R큐브 1대 연결 → 빨강 LED. 완료 시 버튼이 검정. 검정 버튼 클릭 → 연결 해제.
  R2: 1대(=아그리게이터) 연결 → 빨강 LED → SetMultiroleAggregator(총 2대) 전송.
      아그리게이터가 멤버를 연결했다고 알려오면(0xA1) 그 멤버에 초록 LED.
      본인 포함 2대가 모두 연결되면 버튼 검정. 검정 클릭 → 전체 연결 해제.
  R3/R4: R2와 동일 규칙, 총 3대/4대.

■ 아래쪽 '디버그' 영역: 수동 스캔·연결·프레임 전송(개발용).

bleak(async)와 tkinter(동기 루프)는 백그라운드 이벤트 루프 스레드 + 큐로 잇는다.
시나리오 상태(연결 수·완료 여부)는 전부 UI 스레드에서만 다루고,
BLE I/O(connect/send/disconnect)만 코루틴으로 백그라운드 루프에 던진다.
"""
from __future__ import annotations

import asyncio
import json
import queue
import threading
import tkinter as tk
from pathlib import Path
from tkinter import ttk

# 네트워크 세팅 매핑(nodeID→CMF) 저장 파일 (기획서 7.2 PC 매핑 테이블).
NETMAP_PATH = Path(__file__).resolve().parent / "netmap.json"

from rcube import (
    OpCode,
    RCubeBLE,
    RCubeCAN,
    KNOWN_INTERFACES,
    DEFAULT_BITRATE,
    build_frame,
    build_set_led,
    build_set_led_solid,
    build_set_aggregator,
    build_set_group,
    build_fix_order,
    build_set_netconf,
    build_reboot_all,
    parse_frame,
    ADDR_HUB,
    ADDR_BROADCAST,
    RED,
    GREEN,
    OP_AGGREGATOR_EVENT,
)


# ----------------------------------------------------------------------------
# asyncio 이벤트 루프를 별도 스레드에서 구동
# ----------------------------------------------------------------------------
class AsyncLoop:
    def __init__(self) -> None:
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def submit(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    def stop(self) -> None:
        self._loop.call_soon_threadsafe(self._loop.stop)


# ----------------------------------------------------------------------------
# 시나리오 버튼 색상
# ----------------------------------------------------------------------------
def _parse_nn(name: str):
    """광고이름 'RCUBEROBOT.GG.NN'에서 노드번호(NN)를 정수로. 실패 시 None."""
    if not name:
        return None
    try:
        return int(name.rsplit(".", 1)[1])
    except (IndexError, ValueError):
        return None


# 큐브 가상ID(연결순서) → 노드ID 색 이름 (기획서 RGBCMYVO)
CUBE_COLORS = {1: "Red", 2: "Green", 3: "Blue", 4: "Cyan"}

# 노드ID(1~8) → RGB (기획서 RGBCMYVO). CAN 색 확인 명령용.
NODE_RGB = {
    1: (255, 0, 0), 2: (0, 255, 0), 3: (0, 0, 255), 4: (0, 255, 255),
    5: (255, 0, 255), 6: (255, 255, 0), 7: (148, 0, 211), 8: (255, 90, 0),
}


BTN_IDLE = {"bg": "#b8b8b8", "fg": "#000000", "activebackground": "#a8a8a8"}
BTN_BUSY = {"bg": "#e69500", "fg": "#ffffff", "activebackground": "#cf8600"}
BTN_DONE = {"bg": "#111111", "fg": "#ffffff", "activebackground": "#333333"}
BTN_DIS = {"bg": "#dddddd", "fg": "#999999"}


class RCubeApp:
    POLL_MS = 80

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.loop = AsyncLoop()
        self.ui_q: "queue.Queue[tuple[str, object]]" = queue.Queue()

        self.ble = RCubeBLE(
            on_notify=lambda raw: self.ui_q.put(("notify", raw)),
            on_log=lambda msg: self.ui_q.put(("log", msg)),
            on_state=lambda ok: self.ui_q.put(("state", ok)),
        )
        self.can = RCubeCAN(
            on_notify=lambda raw: self.ui_q.put(("notify", raw)),
            on_log=lambda msg: self.ui_q.put(("log", msg)),
            on_state=lambda ok: self.ui_q.put(("can_state", ok)),
        )
        self._scan_results = []

        # ---- 시나리오 상태(UI 스레드 전용) ----
        self.active = None        # 현재 진행 중인 시나리오 번호(1~4) 또는 None
        self.scn_total = 0        # 본인 포함 목표 큐브 수 N
        self.scn_members = 0      # 현재 연결된 멤버 수(아그리게이터 제외)
        self.scn_done = False     # 완료(검정) 여부
        self.scn_ordered = False  # 이번 시나리오가 순서고정 연결인지(시작 시 캡처)
        self.r_btns: dict[int, tk.Button] = {}
        self.net_combos: dict[int, ttk.Combobox] = {}  # 큐브 vid → 통신방식 콤보
        self._net_ui_n = 0        # 현재 네트워크 UI에 표시된 큐브 수
        self._mixed_active = False  # 고정형 혼합연결 진행 중(0xA1 멤버수 추적용)
        self._mixed_members = 0     # 혼합연결 중 연결된 BLE 멤버 수

        root.title("R-Cube 제어 (BLE / CAN)")
        root.geometry("680x780")
        root.minsize(580, 660)

        self._build_ui()
        self.root.after(self.POLL_MS, self._pump_ui)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # =====================================================================
    # UI 구성
    # =====================================================================
    def _build_ui(self) -> None:
        pad = dict(padx=6, pady=4)

        # 1) 시나리오 버튼
        scn = ttk.LabelFrame(self.root, text="시나리오 (R1=1대 · R2=2대 · R3=3대 · R4=4대)")
        scn.pack(fill="x", **pad)
        for n in range(1, 5):
            b = tk.Button(scn, text=f"R{n}", width=10, height=2,
                          font=("", 11, "bold"), relief="raised",
                          command=lambda k=n: self.on_r(k))
            b.grid(row=0, column=n - 1, padx=8, pady=8, sticky="we")
            b.configure(**BTN_IDLE)
            scn.columnconfigure(n - 1, weight=1)
            self.r_btns[n] = b

        self.status_var = tk.StringVar(value="● 대기")
        self.status_lbl = ttk.Label(scn, textvariable=self.status_var)
        self.status_lbl.grid(row=1, column=0, columnspan=4, sticky="w", padx=8, pady=(0, 6))

        # 순서고정 옵션 + 순서고정하기 버튼
        self.ordered_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(scn, text="순서고정으로 연결 (노드ID 순서대로)",
                        variable=self.ordered_var).grid(
            row=2, column=0, columnspan=2, sticky="w", padx=8, pady=(0, 6))
        self.fix_btn = tk.Button(scn, text="순서고정하기", command=self.on_fix_order,
                                 state="disabled")
        self.fix_btn.grid(row=2, column=2, columnspan=2, sticky="e", padx=8, pady=(0, 6))

        # 2) 그룹번호 설정
        grpf = ttk.LabelFrame(self.root, text="그룹번호 설정 (연결된 모든 큐브에 저장 후 재부팅)")
        grpf.pack(fill="x", **pad)
        ttk.Label(grpf, text="그룹번호 (0~99, 0=공장초기)").grid(row=0, column=0, sticky="e", padx=6, pady=6)
        self.group_var = tk.StringVar(value="1")
        ttk.Entry(grpf, textvariable=self.group_var, width=6).grid(row=0, column=1, sticky="w", padx=4)
        ttk.Button(grpf, text="그룹번호 적용", command=self.on_apply_group).grid(
            row=0, column=2, padx=8, pady=6)
        ttk.Label(grpf, text="※ 적용 시 큐브들이 재부팅되어 연결이 끊어집니다.").grid(
            row=0, column=3, sticky="w", padx=6)
        grpf.columnconfigure(3, weight=1)

        # 3) 네트워크(통신방식) 설정 — 다음 부팅부터 적용
        netf = ttk.LabelFrame(self.root, text="네트워크 설정 (큐브별 BLE/CAN · 다음 부팅부터 적용)")
        netf.pack(fill="x", **pad)
        self.net_rows = ttk.Frame(netf)
        self.net_rows.pack(side="left", fill="x", expand=True, padx=6, pady=4)
        self.net_hint = ttk.Label(self.net_rows,
                                  text="R1~R4로 큐브를 모두 연결하면 여기에 큐브별 통신방식 선택이 나타납니다.")
        self.net_hint.grid(row=0, column=0, sticky="w")
        self.net_save_btn = tk.Button(netf, text="저장(재부팅)", command=self.on_save_netconf,
                                      state="disabled")
        self.net_save_btn.pack(side="right", padx=8, pady=4)

        # 3b) 고정형 혼합 연결 (BLE+CAN, 네트워크 저장 후 재부팅한 유닛 재연결)
        mixf = ttk.LabelFrame(self.root, text="고정형 연결 (혼합 BLE+CAN · 네트워크 저장/재부팅 후)")
        mixf.pack(fill="x", **pad)
        ttk.Button(mixf, text="고정형 연결", command=self.on_fixed_connect).pack(side="left", padx=8, pady=4)
        self.mix_status_var = tk.StringVar(
            value="네트워크 설정 저장 시 만들어진 매핑으로 BLE·CAN 큐브를 노드ID 순으로 연결합니다.")
        ttk.Label(mixf, textvariable=self.mix_status_var).pack(side="left", padx=6)

        # 4) 로그
        logf = ttk.LabelFrame(self.root, text="로그")
        logf.pack(fill="both", expand=True, **pad)
        self.log = tk.Text(logf, height=14, wrap="none", state="disabled",
                           font=("Consolas", 9), background="#111", foreground="#ddd")
        self.log.pack(side="left", fill="both", expand=True, padx=(6, 0), pady=6)
        sb = ttk.Scrollbar(logf, command=self.log.yview)
        sb.pack(side="right", fill="y", pady=6)
        self.log.configure(yscrollcommand=sb.set)
        self.log.tag_config("tx", foreground="#6cf")
        self.log.tag_config("rx", foreground="#6f6")
        self.log.tag_config("scn", foreground="#fd6")
        self.log.tag_config("err", foreground="#f66")

        # 5) CAN (USB-CAN 어댑터)
        canf = ttk.LabelFrame(self.root, text="CAN (USB-CAN 어댑터) — CMF=CAN 큐브 제어")
        canf.pack(fill="x", **pad)
        ttk.Label(canf, text="interface").grid(row=0, column=0, sticky="e", padx=4, pady=4)
        self.can_if = ttk.Combobox(canf, state="readonly", width=10, values=KNOWN_INTERFACES)
        self.can_if.set("slcan")
        self.can_if.grid(row=0, column=1, sticky="w")
        ttk.Label(canf, text="channel").grid(row=0, column=2, sticky="e", padx=4)
        self.can_ch = tk.StringVar(value="COM4")
        ttk.Entry(canf, textvariable=self.can_ch, width=12).grid(row=0, column=3, sticky="w")
        ttk.Label(canf, text="bitrate").grid(row=0, column=4, sticky="e", padx=4)
        self.can_br = tk.StringVar(value=str(DEFAULT_BITRATE))
        ttk.Entry(canf, textvariable=self.can_br, width=8).grid(row=0, column=5, sticky="w")
        ttk.Button(canf, text="열기", command=self.on_can_open).grid(row=0, column=6, padx=3)
        ttk.Button(canf, text="닫기", command=self.on_can_close).grid(row=0, column=7, padx=3)
        ttk.Button(canf, text="노드검색", command=self.on_can_discover).grid(row=0, column=8, padx=3)
        ttk.Button(canf, text="순서연결", command=self.on_can_connect_ordered).grid(row=0, column=9, padx=3)
        self.can_status_var = tk.StringVar(value="● CAN 닫힘")
        ttk.Label(canf, textvariable=self.can_status_var).grid(
            row=1, column=0, columnspan=9, sticky="w", padx=6, pady=(0, 4))

        # 6) 디버그(수동 제어)
        dbg = ttk.LabelFrame(self.root, text="디버그 — 수동 스캔·전송")
        dbg.pack(fill="x", **pad)

        self.device_cb = ttk.Combobox(dbg, state="readonly", width=40, values=[])
        self.device_cb.grid(row=0, column=0, columnspan=2, sticky="we", padx=6, pady=4)
        ttk.Button(dbg, text="스캔", command=self.on_scan).grid(row=0, column=2, padx=4, pady=4)
        ttk.Button(dbg, text="연결", command=self.on_connect).grid(row=0, column=3, padx=4, pady=4)
        ttk.Button(dbg, text="끊기", command=self.on_disconnect).grid(row=0, column=4, padx=4, pady=4)

        ttk.Label(dbg, text="Target").grid(row=1, column=0, sticky="e", padx=4)
        self.target_var = tk.StringVar(value="FE")
        ttk.Entry(dbg, textvariable=self.target_var, width=6).grid(row=1, column=1, sticky="w")
        self._op_names = [f"{op.name} (0x{op.value:02X})" for op in OpCode]
        self.op_cb = ttk.Combobox(dbg, state="readonly", width=26, values=self._op_names)
        self.op_cb.grid(row=1, column=2, columnspan=2, sticky="we", padx=4, pady=4)
        self.op_cb.set(f"Heartbeat (0x{OpCode.Heartbeat.value:02X})")
        ttk.Button(dbg, text="BLE전송", command=self.on_send).grid(row=1, column=4, padx=4, pady=4)
        ttk.Button(dbg, text="CAN전송", command=self.on_can_send).grid(row=1, column=5, padx=4, pady=4)

        ttk.Label(dbg, text="payload(hex)").grid(row=2, column=0, sticky="e", padx=4)
        self.payload_var = tk.StringVar(value="")
        ttk.Entry(dbg, textvariable=self.payload_var).grid(
            row=2, column=1, columnspan=4, sticky="we", padx=4, pady=(0, 6))
        dbg.columnconfigure(3, weight=1)

    # =====================================================================
    # 로그 / 상태
    # =====================================================================
    def _log(self, msg: str, tag: str = "") -> None:
        self.log.configure(state="normal")
        self.log.insert("end", msg + "\n", tag)
        self.log.see("end")
        self.log.configure(state="disabled")

    def _set_status(self, text: str) -> None:
        self.status_var.set(text)

    # ---- 시나리오 버튼 모양 ----
    def _paint_buttons(self) -> None:
        """현재 시나리오 상태에 맞게 R1~R4 버튼 색/활성 갱신."""
        for n, b in self.r_btns.items():
            if self.active is None:
                b.configure(state="normal", **BTN_IDLE)
            elif n == self.active:
                b.configure(state="normal", **(BTN_DONE if self.scn_done else BTN_BUSY))
                b.configure(text=f"R{n} ■" if self.scn_done else f"R{n} …")
            else:
                b.configure(state="disabled", **BTN_DIS)
        if self.active is None:
            for n, b in self.r_btns.items():
                b.configure(text=f"R{n}")
        # 순서고정하기: R2~R4가 모두 연결(완료)됐을 때만 활성.
        fix_on = (self.active is not None and self.active >= 2 and self.scn_done)
        self.fix_btn.configure(state="normal" if fix_on else "disabled")
        # 네트워크 설정 UI: 시나리오 완료 시 큐브 수만큼 행 표시.
        self._set_net_ui(self.scn_total if (self.active is not None and self.scn_done) else 0)

    def _set_net_ui(self, n: int) -> None:
        """네트워크 설정 UI를 큐브 n개로 구성(0이면 안내문만). 안정 상태면 재구성 안 함."""
        if n == self._net_ui_n:
            return
        self._net_ui_n = n
        for w in self.net_rows.winfo_children():
            w.destroy()
        self.net_combos.clear()
        if n == 0:
            self.net_hint = ttk.Label(self.net_rows,
                text="R1~R4로 큐브를 모두 연결하면 여기에 큐브별 통신방식 선택이 나타납니다.")
            self.net_hint.grid(row=0, column=0, sticky="w")
            self.net_save_btn.configure(state="disabled")
            return
        for vid in range(1, n + 1):
            color = CUBE_COLORS.get(vid, f"vid{vid}")
            ttk.Label(self.net_rows, text=f"큐브 {vid} ({color})").grid(
                row=vid - 1, column=0, sticky="w", padx=(0, 8), pady=2)
            cb = ttk.Combobox(self.net_rows, state="readonly", width=6, values=["BLE", "CAN"])
            cb.set("BLE")   # 공장 기본
            cb.grid(row=vid - 1, column=1, sticky="w", pady=2)
            self.net_combos[vid] = cb
        self.net_save_btn.configure(state="normal")

    # =====================================================================
    # 코루틴 실행 헬퍼
    # =====================================================================
    def _run(self, coro) -> None:
        fut = self.loop.submit(coro)

        def _done(f):
            exc = f.exception()
            if exc is not None:
                self.ui_q.put(("error", repr(exc)))

        fut.add_done_callback(_done)

    def _run_scn(self, n: int, coro) -> None:
        """시나리오용: 실패 시 해당 시나리오를 초기화."""
        fut = self.loop.submit(coro)

        def _done(f):
            exc = f.exception()
            if exc is not None:
                self.ui_q.put(("scn_failed", (n, repr(exc))))

        fut.add_done_callback(_done)

    # =====================================================================
    # 시나리오 버튼 핸들러
    # =====================================================================
    def on_r(self, n: int) -> None:
        if self.active is None:
            self._start_scenario(n)
        elif self.active == n:
            self._teardown_scenario()
        # 다른 버튼은 비활성 상태라 여기 도달하지 않음

    def _start_scenario(self, n: int) -> None:
        self.active = n
        self.scn_total = n
        self.scn_members = 0
        self.scn_done = False
        self.scn_ordered = bool(self.ordered_var.get())   # UI 스레드에서 캡처
        self._paint_buttons()
        mode = "순서고정" if self.scn_ordered else "연결순"
        self._log(f"[R{n}] 시작 — 본인 포함 총 {n}대 목표 ({mode})", "scn")
        self._run_scn(n, self._scn_start(n))

    async def _scn_start(self, n: int) -> None:
        ordered = self.scn_ordered
        # 1) R큐브 1대(아그리게이터/단일) 연결. 순서고정이면 NN=01(노드1) 큐브를 고른다.
        if n >= 2 and ordered:
            results = await RCubeBLE.scan(timeout=5.0)
            addr = None
            for r in results:
                if _parse_nn(r.name) == 1:
                    addr = r.address
                    break
            if addr is None:
                raise RuntimeError("순서고정 연결: 노드ID 1(RCUBEROBOT.GG.01) 큐브를 찾지 못했습니다.")
            self.ui_q.put(("log", f"[scn] 순서고정: 노드1 큐브 연결 → {addr}"))
            await self.ble.connect(addr)
        else:
            await self.ble.connect(None)
        # 2) 그 큐브를 빨강으로
        await self.ble.send(build_set_led_solid(ADDR_HUB, RED))
        self.ui_q.put(("log", "[scn] 아그리게이터 후보 → 빨강 LED"))
        if n == 1:
            self.ui_q.put(("scn_done", n))
            return
        # 3) 아그리게이터로 승격 + 총 큐브 수 통지(비고정형=그룹무관, 순서고정 여부 전달)
        await self.ble.send(build_set_aggregator(n, group_enabled=False, ordered=ordered))
        self.ui_q.put(("log", f"[scn] SetMultiroleAggregator 전송(총 {n}대, {'NN순' if ordered else '연결순'}). 멤버 0/{n-1} 대기…"))

    def _teardown_scenario(self) -> None:
        n = self.active
        self._log(f"[R{n}] 해제 — 연결 종료", "scn")
        # 상태를 먼저 비워 두면, 뒤이어 오는 disconnect state 이벤트가 무시된다.
        self.active = None
        self.scn_done = False
        self.scn_members = 0
        self._paint_buttons()
        self._set_status("● 대기")
        self._run(self.ble.disconnect())

    # ---- 멤버 연결 알림(0xA1) 처리 ----
    def _on_aggregator_event(self, fr) -> None:
        if self.active is None or self.scn_done or self.scn_total <= 1:
            return
        # payload[0] = 현재 연결된 멤버 수(AggregatorLinkCount) 로 해석(계약)
        cur = fr.payload[0] if fr.payload else self.scn_members + 1
        cur = max(self.scn_members, min(cur, self.scn_total - 1))
        if cur <= self.scn_members:
            return
        # 새로 붙은 멤버(인덱스 prev+1..cur) 각각에 초록 LED.
        # 가상노드ID: 아그리게이터=1, 멤버 k → ID (k+1) = 2..N
        for member_idx in range(self.scn_members + 1, cur + 1):
            vid = member_idx + 1
            self._log(f"[scn] 멤버{member_idx} 연결(가상ID {vid}) → 초록 LED", "scn")
            self._run(self.ble.send(build_set_led_solid(vid, GREEN)))
        self.scn_members = cur
        if self.scn_members >= self.scn_total - 1:
            self.scn_done = True
            self._paint_buttons()
            self._log(f"[R{self.active}] 완료 — 총 {self.scn_total}대 연결", "scn")

    # =====================================================================
    # 큐 폴링: 코루틴/BLE 콜백 → UI
    # =====================================================================
    def _pump_ui(self) -> None:
        try:
            while True:
                kind, payload = self.ui_q.get_nowait()
                if kind == "log":
                    s = str(payload)
                    tag = "tx" if s.startswith("TX") else "rx" if s.startswith("RX") else \
                          "scn" if s.startswith("[scn]") else ""
                    self._log(s, tag)
                elif kind == "notify":
                    self._handle_notify(payload)
                elif kind == "state":
                    self._handle_state(bool(payload))
                elif kind == "scn_done":
                    if self.active == payload:
                        self.scn_done = True
                        self._paint_buttons()
                        self._log(f"[R{payload}] 완료", "scn")
                elif kind == "scn_failed":
                    n, err = payload
                    self._log(f"[R{n}] 실패: {err}", "err")
                    if self.active == n:
                        self.active = None
                        self.scn_done = False
                        self.scn_members = 0
                        self._paint_buttons()
                    self._run(self.ble.disconnect())
                elif kind == "error":
                    self._log(f"[오류] {payload}", "err")
                elif kind == "scan":
                    self._apply_scan(payload)
                elif kind == "can_state":
                    self.can_status_var.set("● CAN 열림" if payload else "● CAN 닫힘")
        except queue.Empty:
            pass
        self.root.after(self.POLL_MS, self._pump_ui)

    def _handle_notify(self, raw: bytes) -> None:
        try:
            fr = parse_frame(raw)
        except Exception:
            return
        self._log(f"    └ {fr}", "rx")
        if fr.op_code == OP_AGGREGATOR_EVENT:
            if self.active is not None:
                self._on_aggregator_event(fr)
            elif self._mixed_active:
                # 혼합 고정형 연결 중: 허브가 보고한 연결 멤버 수 추적.
                self._mixed_members = fr.payload[0] if fr.payload else self._mixed_members

    def _handle_state(self, ok: bool) -> None:
        self._set_status("● 연결됨" if ok else "● 대기")
        if not ok and self.active is not None:
            # 예기치 않은 연결 끊김(수동 teardown은 active를 먼저 비움)
            self._log(f"[R{self.active}] 연결이 끊어져 초기화", "err")
            self.active = None
            self.scn_done = False
            self.scn_members = 0
            self._paint_buttons()

    # =====================================================================
    # 디버그 핸들러
    # =====================================================================
    def on_scan(self) -> None:
        self._log("스캔 중… (약 5초)")
        self._run(self._scan_coro())

    async def _scan_coro(self) -> None:
        results = await RCubeBLE.scan(timeout=5.0)
        self.ui_q.put(("scan", results))

    def _apply_scan(self, results) -> None:
        self._scan_results = results
        self.device_cb["values"] = [str(r) for r in results]
        if results:
            self.device_cb.current(0)
            self._log(f"{len(results)}개 발견")
        else:
            self._log("R큐브를 찾지 못함 (기기 BOOT버튼으로 광고 시작했는지 확인)")

    def _selected_address(self):
        idx = self.device_cb.current()
        if 0 <= idx < len(self._scan_results):
            return self._scan_results[idx].address
        return None

    def on_connect(self) -> None:
        self._run(self.ble.connect(self._selected_address()))

    def on_disconnect(self) -> None:
        self._run(self.ble.disconnect())

    def on_apply_group(self) -> None:
        """그룹번호를 연결된 큐브(+아그리게이터 멤버)에 브로드캐스트로 저장시키고 재부팅."""
        if not self.ble.is_connected:
            self._log("[그룹] 먼저 큐브에 연결하세요.", "err")
            return
        txt = self.group_var.get().strip()
        try:
            group = int(txt, 10)
        except ValueError:
            self._log("[그룹] 그룹번호는 0~99 정수여야 합니다.", "err")
            return
        if not 0 <= group <= 99:
            self._log("[그룹] 그룹번호는 0~99 범위여야 합니다.", "err")
            return
        self._log(f"[그룹] 그룹번호 {group:02d} 적용 → 브로드캐스트 전송(저장 후 재부팅, 연결 끊김 예상)", "scn")
        frame = build_set_group(group, target_id=ADDR_BROADCAST)
        self._run(self.ble.send(frame))   # 큐브가 CmdAck 후 ~0.8s 뒤 재부팅

    def on_fix_order(self) -> None:
        """순서고정하기: 첫 큐브(아그리게이터)에 FIX_ORDER 전송 → 각 큐브가 순서를 노드ID로 저장 후 재부팅."""
        if self.active is None or not self.scn_done or self.scn_total < 2:
            self._log("[순서고정] R2~R4가 모두 연결된 상태에서만 가능합니다.", "err")
            return
        if not self.ble.is_connected:
            self._log("[순서고정] 연결이 없습니다.", "err")
            return
        self._log("[순서고정] FIX_ORDER 전송 → 각 큐브가 현재 순서를 노드ID로 저장 후 재부팅(연결 끊김)", "scn")
        self._run(self.ble.send(build_fix_order()))

    def on_save_netconf(self) -> None:
        """큐브별 통신방식(BLE/CAN)을 저장시키고 전체 재부팅(기획서 7.2). 다음 부팅부터 적용."""
        if self.active is None or not self.scn_done:
            self._log("[네트워크] 모든 큐브가 연결 완료된 뒤 저장하세요.", "err")
            return
        if not self.ble.is_connected:
            self._log("[네트워크] 연결이 없습니다.", "err")
            return
        n = self.scn_total
        choices: dict[int, int] = {}
        can_vids = []
        for vid in range(1, n + 1):
            cb = self.net_combos.get(vid)
            cmf = 1 if (cb is not None and cb.get() == "CAN") else 0
            choices[vid] = cmf
            if cmf == 1:
                can_vids.append(vid)
        term = max(can_vids) if can_vids else 0
        desc = ", ".join(f"{vid}:{'CAN' if choices[vid] else 'BLE'}" for vid in range(1, n + 1))
        # 기획서 7.2 step3: PC가 nodeID→CMF 매핑 테이블을 자기 메모리에 저장(고정형 재연결 근거).
        self._save_netmap(n, choices, term)
        self._log(f"[네트워크] 저장 [{desc}] 종단노드={term} → 각 큐브 저장 후 전체 재부팅(연결 끊김)", "scn")
        self._run(self._save_netconf_coro(n, choices, term))

    def _save_netmap(self, n: int, choices: dict, term: int) -> None:
        try:
            NETMAP_PATH.write_text(json.dumps(
                {"n": n, "term": term, "cmf": {str(v): c for v, c in choices.items()}},
                ensure_ascii=False, indent=2), encoding="utf-8")
            self._log(f"[네트워크] 매핑 저장: {NETMAP_PATH.name}")
        except Exception as e:
            self._log(f"[네트워크] 매핑 저장 실패: {e}", "err")

    def _load_netmap(self):
        try:
            if not NETMAP_PATH.exists():
                return None
            return json.loads(NETMAP_PATH.read_text(encoding="utf-8"))
        except Exception as e:
            self._log(f"[고정형] 매핑 읽기 실패: {e}", "err")
            return None

    # ---- 고정형 혼합 연결 (기획서 7.2 재연결) ----
    def on_fixed_connect(self) -> None:
        """저장된 매핑(nodeID→CMF)으로 BLE·CAN 큐브를 노드ID 순으로 재연결."""
        m = self._load_netmap()
        if not m:
            self._log("[고정형] 저장된 매핑이 없습니다. 먼저 '네트워크 설정 저장'을 하세요.", "err")
            return
        if self.active is not None:
            self._log("[고정형] 진행 중인 R1~R4 시나리오를 먼저 해제하세요.", "err")
            return
        cmf = m.get("cmf", {})
        ble = sorted(int(k) for k, v in cmf.items() if int(v) == 0)
        can = sorted(int(k) for k, v in cmf.items() if int(v) == 1)
        self._log(f"[고정형] 매핑 N={m.get('n')} · BLE={ble} · CAN={can}", "scn")

        # CAN 분기: 버스가 열려 있으면 노드ID 순으로 연결.
        if can:
            if self.can.is_connected:
                self._run_can_fixed(can)
            else:
                self._log("[고정형/CAN] CAN 큐브가 있으나 버스 미열림 — CAN 프레임에서 '열기' 후 다시 시도.", "err")

        # BLE 분기: 허브(최소 BLE 노드ID) 연결 후 나머지 BLE 멤버를 노드ID 순으로.
        if ble:
            self._run(self._fixed_ble_coro(ble))

    def _run_can_fixed(self, can_nodes: list) -> None:
        def per(nid, index):
            rgb = NODE_RGB.get(nid, (255, 255, 255))
            try:
                self.can.send(build_set_led(nid, [rgb]))
            except Exception:
                pass
            self.ui_q.put(("log", f"[고정형/CAN] 노드 0x{nid:02X} 연결(순서 {index + 1})"))

        def work():
            found = self.can.connect_ordered(3.0, per_node=per)
            missing = [n for n in can_nodes if n not in found]
            msg = f"[고정형/CAN] 발견 {[hex(n) for n in found]}"
            if missing:
                msg += f" · 미발견 {[hex(n) for n in missing]}(전원/버스 확인)"
            self.ui_q.put(("log", msg))
        threading.Thread(target=work, daemon=True).start()

    async def _fixed_ble_coro(self, ble: list) -> None:
        hub = ble[0]          # 최소 BLE 노드ID = BLE 허브 큐브
        count = len(ble)
        self.ui_q.put(("log", f"[고정형/BLE] 허브=노드{hub}, BLE {count}대 연결 시작"))
        # 허브(광고이름 NN==hub) 큐브를 찾아 연결.
        results = await RCubeBLE.scan(timeout=5.0)
        addr = next((r.address for r in results if _parse_nn(r.name) == hub), None)
        if addr is None:
            raise RuntimeError(f"BLE 허브(RCUBEROBOT.GG.{hub:02d})를 찾지 못했습니다.")
        await self.ble.connect(addr)
        # 허브는 자기 노드ID 색으로.
        await self.ble.send(build_set_led_solid(ADDR_HUB, NODE_RGB.get(hub, (255, 255, 255))))
        if count == 1:
            self.ui_q.put(("log", "[고정형/BLE] 단일 BLE 큐브 연결 완료"))
            return
        # 허브를 아그리게이터로 승격(고정형=NN 기준). 멤버 = 나머지 BLE 큐브.
        self._mixed_active = True
        self._mixed_members = 0
        await self.ble.send(build_set_aggregator(count, ordered=True))
        self.ui_q.put(("log", f"[고정형/BLE] 허브가 BLE 멤버 {count - 1}대 연결 대기…"))
        for _ in range(40):   # 최대 20초 대기
            if self._mixed_members >= count - 1:
                break
            await asyncio.sleep(0.5)
        # 각 BLE 멤버에 자기 노드ID 색 전송(허브가 target=노드ID로 중계, 고정형 vid=노드ID).
        for nid in ble[1:]:
            await self.ble.send(build_set_led_solid(nid, NODE_RGB.get(nid, (255, 255, 255))))
        self._mixed_active = False
        done = min(self._mixed_members + 1, count)
        self.ui_q.put(("log", f"[고정형/BLE] 연결 {done}/{count} — 노드ID {ble} 색 지정 완료", ))

    async def _save_netconf_coro(self, n: int, choices: dict, term: int) -> None:
        # 1) 각 큐브에 통신방식 세팅 저장(재부팅 없이). vid1=아그리게이터(0xFE), 그 외=멤버 중계.
        for vid in range(1, n + 1):
            target = ADDR_HUB if vid == 1 else vid
            await self.ble.send(build_set_netconf(vid, choices[vid], term, target_id=target))
        await asyncio.sleep(0.2)
        # 2) 전체 재부팅(브로드캐스트).
        await self.ble.send(build_reboot_all())

    def _debug_frame(self):
        """디버그 입력(Target/OpCode/payload)에서 표준 프레임을 만든다. 실패 시 None."""
        sel = self.op_cb.get()
        try:
            op = int(sel.rsplit("0x", 1)[1].rstrip(")"), 16)
        except (IndexError, ValueError):
            self._log("[오류] OpCode를 선택하세요.", "err")
            return None
        txt = self.target_var.get().strip().lower().replace("0x", "")
        target = int(txt, 16) if txt else ADDR_BROADCAST
        hexstr = self.payload_var.get().strip().replace(" ", "")
        try:
            payload = bytes.fromhex(hexstr) if hexstr else b""
        except ValueError:
            self._log("[오류] payload는 hex(예: E0 01 FF)여야 합니다.", "err")
            return None
        try:
            return build_frame(target, op, payload)
        except Exception as e:
            self._log(f"[프레임 오류] {e}", "err")
            return None

    def on_send(self) -> None:
        frame = self._debug_frame()
        if frame is not None:
            self._run(self.ble.send(frame))

    # ---- CAN(USB-CAN) 핸들러 ----
    def on_can_open(self) -> None:
        interface = self.can_if.get()
        channel = self.can_ch.get().strip()
        try:
            bitrate = int(self.can_br.get().strip())
        except ValueError:
            self._log("[CAN] bitrate는 정수여야 합니다.", "err")
            return
        self._log(f"[CAN] 버스 여는 중… ({interface}/{channel})")

        def work():
            try:
                self.can.open(interface, channel, bitrate)
            except Exception as e:
                self.ui_q.put(("error", f"CAN 열기 실패: {e!r}"))
        threading.Thread(target=work, daemon=True).start()

    def on_can_close(self) -> None:
        threading.Thread(target=self.can.close, daemon=True).start()

    def on_can_discover(self) -> None:
        if not self.can.is_connected:
            self._log("[CAN] 먼저 버스를 여세요.", "err")
            return
        self._log("[CAN] 노드 검색(2초, 하트비트 수집)…")

        def work():
            nodes = self.can.discover(2.0)
            names = ", ".join(f"0x{n:02X}" for n in nodes) if nodes else "없음"
            self.ui_q.put(("log", f"[CAN] 발견 노드ID: {names}"))
        threading.Thread(target=work, daemon=True).start()

    def on_can_connect_ordered(self) -> None:
        """고정형 CAN 연결(기획서 7.2 CAN 분기): 하트비트로 노드 발견 → 노드ID 순서대로 연결."""
        if not self.can.is_connected:
            self._log("[CAN] 먼저 버스를 여세요.", "err")
            return
        self._log("[CAN] 고정형 순서연결: 하트비트 수집 후 노드ID 오름차순 연결(3초)…", "scn")

        def per_node(nid, index):
            # 고정형 CAN 큐브는 자기 노드ID 색으로 자가점등. PC는 순서대로 색 확인만 보낸다.
            rgb = NODE_RGB.get(nid, (255, 255, 255))
            try:
                # CAN 데이터필드 ≤8B → LED 1개짜리 확인 색(payload 4B)만 전송.
                self.can.send(build_set_led(nid, [rgb]))
            except Exception as e:
                self.ui_q.put(("log", f"[CAN] 노드 0x{nid:02X} 색확인 전송 실패: {e}"))
            self.ui_q.put(("log", f"[CAN] 노드 0x{nid:02X} 연결(순서 {index + 1})"))

        def work():
            nodes = self.can.connect_ordered(3.0, per_node=per_node)
            if not nodes:
                self.ui_q.put(("log", "[CAN] 노드 없음(하트비트 미수신). 큐브 CMF=CAN·버스 배선·종단 확인.", ))
            else:
                order = ", ".join(f"0x{n:02X}" for n in nodes)
                self.ui_q.put(("log", f"[CAN] 순서연결 완료 — 노드ID 순: {order}"))
        threading.Thread(target=work, daemon=True).start()

    def on_can_send(self) -> None:
        if not self.can.is_connected:
            self._log("[CAN] 먼저 버스를 여세요.", "err")
            return
        frame = self._debug_frame()
        if frame is None:
            return
        try:
            self.can.send(frame)
        except Exception as e:
            self._log(f"[CAN 전송 오류] {e}", "err")

    # =====================================================================
    # 종료
    # =====================================================================
    def _on_close(self) -> None:
        try:
            fut = self.loop.submit(self.ble.disconnect())
            fut.result(timeout=2.0)
        except Exception:
            pass
        try:
            self.can.close()
        except Exception:
            pass
        self.loop.stop()
        self.root.destroy()


def main() -> int:
    root = tk.Tk()
    RCubeApp(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
