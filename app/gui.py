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
import queue
import threading
import tkinter as tk
from tkinter import ttk

from rcube import (
    OpCode,
    RCubeBLE,
    build_frame,
    build_set_led_solid,
    build_set_aggregator,
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
        self._scan_results = []

        # ---- 시나리오 상태(UI 스레드 전용) ----
        self.active = None        # 현재 진행 중인 시나리오 번호(1~4) 또는 None
        self.scn_total = 0        # 본인 포함 목표 큐브 수 N
        self.scn_members = 0      # 현재 연결된 멤버 수(아그리게이터 제외)
        self.scn_done = False     # 완료(검정) 여부
        self.r_btns: dict[int, tk.Button] = {}

        root.title("R-Cube BLE 제어")
        root.geometry("660x620")
        root.minsize(560, 520)

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

        # 2) 로그
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

        # 3) 디버그(수동 제어)
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
        ttk.Button(dbg, text="전송", command=self.on_send).grid(row=1, column=4, padx=4, pady=4)

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
        self._paint_buttons()
        self._log(f"[R{n}] 시작 — 본인 포함 총 {n}대 목표", "scn")
        self._run_scn(n, self._scn_start(n))

    async def _scn_start(self, n: int) -> None:
        # 1) R큐브 1대(아그리게이터/단일) 자동 연결
        await self.ble.connect(None)
        # 2) 그 큐브를 빨강으로
        await self.ble.send(build_set_led_solid(ADDR_HUB, RED))
        self.ui_q.put(("log", "[scn] 아그리게이터 후보 → 빨강 LED"))
        if n == 1:
            self.ui_q.put(("scn_done", n))
            return
        # 3) 아그리게이터로 승격 + 총 큐브 수 통지(비고정형=그룹무관)
        await self.ble.send(build_set_aggregator(n, group_enabled=False))
        self.ui_q.put(("log", f"[scn] SetMultiroleAggregator 전송(총 {n}대). 멤버 0/{n-1} 연결 대기…"))

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
            self._on_aggregator_event(fr)

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

    def on_send(self) -> None:
        sel = self.op_cb.get()
        try:
            op = int(sel.rsplit("0x", 1)[1].rstrip(")"), 16)
        except (IndexError, ValueError):
            self._log("[오류] OpCode를 선택하세요.", "err")
            return
        txt = self.target_var.get().strip().lower().replace("0x", "")
        target = int(txt, 16) if txt else ADDR_BROADCAST
        hexstr = self.payload_var.get().strip().replace(" ", "")
        try:
            payload = bytes.fromhex(hexstr) if hexstr else b""
        except ValueError:
            self._log("[오류] payload는 hex(예: E0 01 FF)여야 합니다.", "err")
            return
        try:
            frame = build_frame(target, op, payload)
        except Exception as e:
            self._log(f"[프레임 오류] {e}", "err")
            return
        self._run(self.ble.send(frame))

    # =====================================================================
    # 종료
    # =====================================================================
    def _on_close(self) -> None:
        try:
            fut = self.loop.submit(self.ble.disconnect())
            fut.result(timeout=2.0)
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
