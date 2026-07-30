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
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

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
    build_get_node_config,
    build_reset_config,
    build_member_map,
    build_set_edge_central,
    build_get_edge_central,
    build_shutdown,
    build_get_sensors,
    build_set_sensor_stream,
    parse_sensor_payload,
    SENSOR_KIND_ACCEL,
    SENSOR_KIND_GYRO,
    SENSOR_PERIOD_DEFAULT_MS,
    build_set_angle,
    build_move_to_origin,
    build_set_this_to_origin,
    build_set_drive_state,
    build_emergency_stop,
    build_get_motor_status,
    build_timesync,
    parse_timesync_reply,
    timesync_solve,
    parse_motor_status,
    parse_motion_complete,
    MOTION_REASON,
    DRIVE_DISABLE,
    DRIVE_ENABLE,
    MAX_NODES,
    MEMBER_NONE,
    MEMBER_CAN,
    build_set_netconf,
    build_reboot_all,
    parse_frame,
    ADDR_HUB,
    ADDR_BROADCAST,
    RED,
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

# 노드ID(1~8) → RGB (기획서 RGBCMYVO). 펌웨어 rcube_status.c PALETTE와 동일해야 한다.
NODE_RGB = {
    1: (255, 0, 0), 2: (0, 255, 0), 3: (0, 0, 255), 4: (0, 255, 255),
    5: (255, 0, 255), 6: (255, 255, 0), 7: (148, 0, 211), 8: (255, 90, 0),
}
WHITE = (255, 255, 255)   # 노드ID 범위 밖(미할당 등) 대체색

# TimeSync 왕복 측정 횟수. BLE는 connection event 대기로 지터가 커서 여러 번 재고
# 최소 RTT 표본을 쓴다(확장 규격 §3.2).
TIMESYNC_SAMPLES = 7


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
        self.r_btns: dict[int, tk.Button] = {}
        self.net_combos: dict[int, ttk.Combobox] = {}  # 큐브 vid → 통신방식 콤보
        self._net_ui_n = 0        # 현재 네트워크 UI에 표시된 큐브 수
        self.scn_fixed = False    # 이번 시나리오가 고정형 재연결인지(시작 시 캡처)
        self.scn_ble_nodes = []   # 고정형: BLE로 세팅된 노드ID 목록(오름차순)
        self.scn_ble_members = 0  # 허브가 취합해야 할 BLE 멤버 수(= BLE 큐브 수 - 1)
        self._ts_reply = None     # TimeSync 왕복 회신 (notify → _timesync_coro)

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

        # 연결 방식 선택 — R{N}이 "몇 대짜리 유닛"을, 이 라디오가 "어떤 절차"를 정한다.
        #   비고정형(7.1) : 노드ID 미할당 큐브를 켜는 순서대로 BLE로만 구성(초기 구성).
        #   고정형(7.2-7) : 저장된 노드ID·통신방식대로 BLE 분기/CAN 분기를 나눠 재연결.
        self.fixed_var = tk.BooleanVar(value=False)
        modef = ttk.Frame(scn)
        modef.grid(row=1, column=0, columnspan=4, sticky="w", padx=8, pady=(0, 2))
        ttk.Radiobutton(modef, text="비고정형 (7.1 초기 구성 · 켜는 순서 = 가상 노드ID)",
                        variable=self.fixed_var, value=False,
                        command=self._paint_mode_hint).pack(side="left", padx=(0, 12))
        ttk.Radiobutton(modef, text="고정형 (7.2 재연결 · 저장된 노드ID/통신방식)",
                        variable=self.fixed_var, value=True,
                        command=self._paint_mode_hint).pack(side="left")

        self.mode_hint_var = tk.StringVar()
        ttk.Label(scn, textvariable=self.mode_hint_var, wraplength=640, justify="left").grid(
            row=2, column=0, columnspan=4, sticky="w", padx=8, pady=(0, 4))

        self.status_var = tk.StringVar(value="● 대기")
        self.status_lbl = ttk.Label(scn, textvariable=self.status_var)
        self.status_lbl.grid(row=3, column=0, columnspan=4, sticky="w", padx=8, pady=(0, 6))
        self._paint_mode_hint()

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
        netright = ttk.Frame(netf)
        netright.pack(side="right", padx=8, pady=4)
        # 기획서 7.3-1: 체크하면 통신방식 세팅과 함께 리드 큐브(노드01)를 edge central로
        # 만든다(ECF=1 + 멤버 맵 + N 저장). 미션코드 업로드는 8장에서 붙인다.
        self.edge_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(netright, text="고정로봇유닛(독립)",
                        variable=self.edge_var, command=self._paint_net_save).pack(anchor="e")
        self.net_save_btn = tk.Button(netf, text="저장(재부팅)", command=self.on_save_netconf,
                                      state="disabled")
        self.net_save_btn.pack(side="right", padx=4, pady=4)

        # 3b) 큐브 설정 도구 (조회 / 초기화)
        # 두 명령 모두 "허브 + 전 멤버"를 대상으로 하므로, R1~R4로 유닛이 완전히 구성된
        # 뒤에만 의미가 있다. 그 전에는 버튼을 감추고 안내문만 보여준다.
        self.toolf = ttk.LabelFrame(self.root, text="큐브 설정")
        self.toolf.pack(fill="x", **pad)
        self.tool_btns = [
            ttk.Button(self.toolf, text="설정 조회", command=self.on_get_config),
            ttk.Button(self.toolf, text="독립 해제(강등)", command=self.on_demote_edge),
            ttk.Button(self.toolf, text="공장 초기화", command=self.on_reset_config),
        ]
        self.tool_desc = ttk.Label(
            self.toolf, text="조회=저장 설정 확인 · 강등=ECF 해제(설정모드로 연결한 큐브에) · "
                             "초기화=비고정형으로 복귀")
        self.tool_hint = ttk.Label(
            self.toolf, text="R1~R4로 큐브가 모두 연결되면 설정 조회·공장 초기화를 쓸 수 있습니다.")
        self._paint_tools()

        # 3c) 센서 모니터링 (기획서 9장)
        # 유닛이 완전히 구성된 뒤에만 의미가 있으므로 설정 도구와 같은 조건으로 노출한다.
        self.sensf = ttk.LabelFrame(self.root, text="센서 모니터링 (기획서 9장)")
        self.sensf.pack(fill="x", **pad)
        self.sens_bar = ttk.Frame(self.sensf)
        self.sens_start = ttk.Button(self.sens_bar, text="전송 시작",
                                     command=lambda: self.on_sensor_stream(True))
        self.sens_stop = ttk.Button(self.sens_bar, text="중지",
                                    command=lambda: self.on_sensor_stream(False))
        self.sens_once = ttk.Button(self.sens_bar, text="1회 조회", command=self.on_sensor_once)
        self.sens_period_var = tk.StringVar(value=str(SENSOR_PERIOD_DEFAULT_MS))
        self.sens_start.pack(side="left", padx=(8, 4), pady=4)
        self.sens_stop.pack(side="left", padx=4, pady=4)
        self.sens_once.pack(side="left", padx=4, pady=4)
        ttk.Label(self.sens_bar, text="주기(ms)").pack(side="left", padx=(12, 4))
        ttk.Entry(self.sens_bar, textvariable=self.sens_period_var, width=6).pack(side="left")
        self.sens_rows = ttk.Frame(self.sensf)
        self.sens_labels: dict[int, tk.StringVar] = {}
        self.sens_hint = ttk.Label(
            self.sensf, text="R1~R4로 큐브가 모두 연결되면 센서 전송을 시작할 수 있습니다.")
        self._paint_sensors()

        # 3d) 모터 제어 (확장 규격 §2.11)
        self.motf = ttk.LabelFrame(self.root, text="모터 제어")
        self.motf.pack(fill="x", **pad)
        self.mot_bar = ttk.Frame(self.motf)
        ttk.Label(self.mot_bar, text="대상").pack(side="left", padx=(8, 2))
        self.mot_target = ttk.Combobox(self.mot_bar, state="readonly", width=6)
        self.mot_target.pack(side="left")
        ttk.Label(self.mot_bar, text="각도(°)").pack(side="left", padx=(10, 2))
        self.mot_deg = tk.StringVar(value="90")
        ttk.Entry(self.mot_bar, textvariable=self.mot_deg, width=7).pack(side="left")
        ttk.Label(self.mot_bar, text="도달(ms)").pack(side="left", padx=(10, 2))
        self.mot_ms = tk.StringVar(value="1000")
        ttk.Entry(self.mot_bar, textvariable=self.mot_ms, width=7).pack(side="left")
        ttk.Button(self.mot_bar, text="이동", command=self.on_motor_move).pack(side="left", padx=6)
        ttk.Button(self.mot_bar, text="원점으로", command=self.on_motor_home).pack(side="left", padx=2)
        ttk.Button(self.mot_bar, text="여기를 원점", command=self.on_motor_set_origin).pack(side="left", padx=2)

        self.mot_bar2 = ttk.Frame(self.motf)
        ttk.Button(self.mot_bar2, text="Enable",
                   command=lambda: self.on_drive_state(DRIVE_ENABLE)).pack(side="left", padx=(8, 2))
        ttk.Button(self.mot_bar2, text="Disable",
                   command=lambda: self.on_drive_state(DRIVE_DISABLE)).pack(side="left", padx=2)
        ttk.Button(self.mot_bar2, text="상태 조회", command=self.on_motor_status).pack(side="left", padx=8)
        ttk.Button(self.mot_bar2, text="시계 동기", command=self.on_timesync).pack(side="left", padx=2)
        # 비상정지는 브로드캐스트라 대상 선택과 무관하다.
        tk.Button(self.mot_bar2, text="비상 정지(전체)", command=self.on_estop,
                  bg="#c0392b", fg="#ffffff", activebackground="#a93226").pack(side="left", padx=12)
        self.mot_hint = ttk.Label(
            self.motf, text="R1~R4로 큐브가 모두 연결되면 모터 제어를 쓸 수 있습니다.")
        self._paint_motor()

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

    # ---- 네트워크 저장 버튼 문구 ----
    def _paint_net_save(self) -> None:
        """독립 전환 여부에 따라 저장 버튼이 무엇을 하는지 문구로 드러낸다."""
        if not hasattr(self, "net_save_btn"):
            return
        self.net_save_btn.configure(
            text="저장(전원끄기)" if self.edge_var.get() else "저장(재부팅)")

    # ---- 큐브 설정 도구 노출 ----
    def _tools_ready(self) -> bool:
        """설정 조회·공장 초기화를 쓸 수 있는 상태인지(유닛 구성 완료 + 연결 유지)."""
        return (self.active is not None and self.scn_done and self.ble.is_connected)

    def _paint_tools(self) -> None:
        ready = self._tools_ready()
        for w in (*self.tool_btns, self.tool_desc, self.tool_hint):
            w.pack_forget()
        if ready:
            for b in self.tool_btns:
                b.pack(side="left", padx=(8, 4), pady=4)
            self.tool_desc.pack(side="left", padx=6)
        else:
            self.tool_hint.pack(side="left", padx=8, pady=4)

    # ---- 센서 모니터링 노출/행 구성 ----
    def _unit_nodes(self) -> list:
        """현재 유닛을 이루는 큐브의 주소 목록(허브 포함). 표시·조회 대상."""
        if not self.active:
            return []
        if self.scn_fixed:
            return list(self.scn_ble_nodes)
        return list(range(1, self.scn_total + 1))

    def _paint_sensors(self) -> None:
        ready = self._tools_ready()
        for w in (self.sens_bar, self.sens_rows, self.sens_hint):
            w.pack_forget()
        if not ready:
            self.sens_labels.clear()
            for w in self.sens_rows.winfo_children():
                w.destroy()
            self.sens_hint.pack(side="left", padx=8, pady=4)
            return
        self.sens_bar.pack(fill="x")
        nodes = self._unit_nodes()
        if set(nodes) != set(self.sens_labels.keys()):
            self.sens_labels.clear()
            for w in self.sens_rows.winfo_children():
                w.destroy()
            for i, nid in enumerate(nodes):
                label = "허브" if (not self.scn_fixed and nid == 1) else f"노드 {nid}"
                ttk.Label(self.sens_rows, text=f"{label} ({CUBE_COLORS.get(nid, nid)})",
                          width=16).grid(row=i, column=0, sticky="w", padx=(8, 6), pady=1)
                var = tk.StringVar(value="—")
                ttk.Label(self.sens_rows, textvariable=var,
                          font=("Consolas", 9)).grid(row=i, column=1, sticky="w")
                self.sens_labels[nid] = var
        self.sens_rows.pack(fill="x", pady=(0, 4))

    # ---- 모터 제어 노출 ----
    def _paint_motor(self) -> None:
        ready = self._tools_ready()
        for w in (self.mot_bar, self.mot_bar2, self.mot_hint):
            w.pack_forget()
        if not ready:
            self.mot_hint.pack(side="left", padx=8, pady=4)
            return
        nodes = self._unit_nodes()
        values = [str(n) for n in nodes]
        if list(self.mot_target["values"]) != values:
            self.mot_target["values"] = values
            if values:
                self.mot_target.current(0)
        self.mot_bar.pack(fill="x", pady=(2, 0))
        self.mot_bar2.pack(fill="x", pady=(0, 4))

    def _motor_target(self):
        """모터 명령의 TargetId. 유닛 첫 큐브는 허브(0xFE), 나머지는 그 주소."""
        try:
            nid = int(self.mot_target.get())
        except (ValueError, TypeError):
            return None
        nodes = self._unit_nodes()
        return ADDR_HUB if (nodes and nid == nodes[0]) else nid

    # ---- 연결 방식 안내문 ----
    def _paint_mode_hint(self) -> None:
        """선택한 연결 방식에 맞는 안내문. 고정형이면 저장된 매핑 구성을 보여준다."""
        if not self.fixed_var.get():
            self.mode_hint_var.set(
                "R{N} = 유닛 큐브 수. 정한 순서대로 큐브를 하나씩 켜면 연결 순서가 곧 "
                "가상 노드ID입니다 (1 Red, 2 Green, 3 Blue, 4 Cyan). "
                "고정형 전환은 아래 네트워크 설정 '저장'이 합니다.")
            return
        m = self._load_netmap(quiet=True)
        if not m:
            self.mode_hint_var.set(
                "저장된 매핑이 없습니다. 먼저 비고정형으로 구성한 뒤 네트워크 설정에서 "
                "'저장'을 해야 고정형으로 재연결할 수 있습니다.")
            return
        cmf = m.get("cmf", {})
        ble = sorted(int(k) for k, v in cmf.items() if int(v) == 0)
        can = sorted(int(k) for k, v in cmf.items() if int(v) == 1)
        self.mode_hint_var.set(
            f"저장된 매핑: 총 {m.get('n')}대 · BLE 노드 {ble or '없음'} · CAN 노드 {can or '없음'} "
            f"(종단 {m.get('term')}). R{m.get('n')}을 누르면 BLE 분기와 CAN 분기를 각각 "
            f"노드ID 순으로 연결합니다.")

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
        # 네트워크 설정 UI: 시나리오 완료 시 큐브 수만큼 행 표시.
        self._set_net_ui(self.scn_total if (self.active is not None and self.scn_done) else 0)
        # 설정 조회·공장 초기화·센서·모터: 유닛 구성이 끝난 뒤에만 노출.
        self._paint_tools()
        self._paint_sensors()
        self._paint_motor()

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
        fixed = bool(self.fixed_var.get())   # UI 스레드에서 캡처
        ble_nodes, can_nodes = [], []
        if fixed:
            m = self._load_netmap()
            if not m:
                self._log("[고정형] 저장된 매핑이 없습니다. 비고정형으로 구성 후 "
                          "네트워크 설정에서 '저장'을 먼저 하세요.", "err")
                return
            cmf = m.get("cmf", {})
            ble_nodes = sorted(int(k) for k, v in cmf.items() if int(v) == 0)
            can_nodes = sorted(int(k) for k, v in cmf.items() if int(v) == 1)
            saved_n = int(m.get("n", 0))
            if saved_n != n:
                self._log(f"[고정형] R{n}을 눌렀지만 저장된 매핑은 {saved_n}대 구성입니다. "
                          f"R{saved_n}을 누르세요.", "err")
                return

        self.active = n
        self.scn_total = n
        self.scn_members = 0
        self.scn_done = False
        self.scn_fixed = fixed
        # 고정형은 BLE 분기만 허브가 취합한다(CAN 분기는 PC가 직접). 완료 판정 기준이 다르다.
        self.scn_ble_nodes = ble_nodes
        self.scn_ble_members = max(0, len(ble_nodes) - 1) if fixed else max(0, n - 1)
        self._paint_buttons()

        if fixed:
            self._log(f"[R{n}] 시작 — 고정형 재연결 (7.2-7) · BLE {ble_nodes} · CAN {can_nodes}", "scn")
            if can_nodes:
                if self.can.is_connected:
                    self._run_can_fixed(can_nodes)
                else:
                    self._log("[고정형/CAN] CAN 큐브가 있으나 버스 미열림 — CAN 프레임에서 "
                              "'열기' 후 다시 시도하세요(BLE 분기는 계속 진행).", "err")
            if ble_nodes:
                self._run_scn(n, self._scn_start_fixed(ble_nodes))
            else:
                self._log("[고정형] BLE 큐브가 없습니다 — CAN 분기만 진행합니다.", "scn")
                self.ui_q.put(("scn_done", n))
        else:
            self._log(f"[R{n}] 시작 — 비고정형 초기 구성 (7.1) · 켜는 순서 = 가상 노드ID", "scn")
            self._run_scn(n, self._scn_start(n))

    async def _scn_start(self, n: int) -> None:
        """기획서 7.1 공통 1단계 — 비고정형 BLE로 전체 구성.

        사용자가 정한 순서대로 큐브를 켜면, PC에 처음 연결된 큐브가 가상1(Red)이 되어
        BLE 허브가 되고 나머지를 연결 순서대로 취합한다. 노드ID는 저장하지 않는다.
        """
        # 1) 가장 먼저 광고 중인 R큐브에 연결 = 사용자가 첫 번째로 켠 큐브 = 가상1.
        await self.ble.connect(None)
        # 2) 그 큐브를 가상1 색(Red)으로.
        await self.ble.send(build_set_led_solid(ADDR_HUB, RED))
        self.ui_q.put(("log", "[scn] 가상1 → 빨강 LED (BLE 허브 후보)"))
        if n == 1:
            self.ui_q.put(("scn_done", n))
            return
        # 3) BLE 허브로 승격 + 총 큐브 수 통지.
        #    ordered=False → 허브가 저장 노드ID를 무시하고 연결 순서로 가상ID를 배정한다.
        #    이미 노드ID가 저장된 큐브로 재구성할 때도 순서가 우선이어야 한다(§2.2).
        await self.ble.send(build_set_aggregator(n, group_enabled=False, ordered=False))
        self.ui_q.put(("log", f"[scn] SetMultiroleAggregator 전송(총 {n}대). 멤버 0/{n-1} 대기…"))

    async def _scn_start_fixed(self, ble: list) -> None:
        """기획서 7.2-7 [BLE 분기] — 저장된 노드ID로 BLE 큐브만 재연결.

        BLE로 세팅된 큐브 중 최소 노드ID가 허브가 되고, PC는 허브에게 "BLE로 세팅된
        큐브 수"만 알려준다. 중간 노드ID가 CAN이라 빠져 있어도(예: 1·3만 BLE) BLE
        분기는 그 2대로 완료된다. 가상ID는 허브가 광고 노드ID로 부여하므로 색이 어긋나지 않는다.
        """
        hub = ble[0]              # 최소 BLE 노드ID = BLE 허브 큐브
        count = len(ble)          # BLE 분기 큐브 수(허브 포함)
        self.ui_q.put(("log", f"[고정형/BLE] 허브=노드{hub}, BLE {count}대 연결 시작"))
        results = await RCubeBLE.scan(timeout=5.0)
        addr = next((r.address for r in results if _parse_nn(r.name) == hub), None)
        if addr is None:
            raise RuntimeError(f"BLE 허브(노드ID {hub})를 찾지 못했습니다. "
                               f"해당 큐브를 켜고 버튼으로 연결모드에 넣으세요.")
        await self.ble.connect(addr)
        # 7.2-7 ★: 허브는 무조건 Red가 아니라 자기 노드ID 색으로 켜진다.
        await self.ble.send(build_set_led_solid(ADDR_HUB, NODE_RGB.get(hub, WHITE)))
        if count == 1:
            self.ui_q.put(("log", "[고정형/BLE] 단일 BLE 큐브 연결 완료"))
            self.ui_q.put(("scn_done", self.scn_total))
            return
        # 허브에 "BLE로 세팅된 큐브 수"를 전달. ordered=True → 가상ID = 광고 노드ID(NN).
        await self.ble.send(build_set_aggregator(count, group_enabled=False, ordered=True))
        self.ui_q.put(("log", f"[고정형/BLE] 허브가 BLE 멤버 {count - 1}대 연결 대기… "
                              f"(대상 노드ID {ble[1:]})"))

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
        if self.active is None or self.scn_done or self.scn_ble_members <= 0:
            return
        # payload[0] = 현재 연결된 멤버 수(AggregatorLinkCount) 로 해석(계약)
        target = self.scn_ble_members
        cur = fr.payload[0] if fr.payload else self.scn_members + 1
        cur = max(self.scn_members, min(cur, target))
        if cur <= self.scn_members:
            return

        if self.scn_fixed:
            # 고정형: 허브가 광고 노드ID를 그대로 가상ID로 쓰므로 연결 순서로 추측할 수 없다.
            # 색은 전원 연결된 뒤 노드ID 기준으로 한 번에 지정한다.
            self._log(f"[고정형/BLE] 멤버 {cur}/{target} 연결", "scn")
        else:
            # 비고정형: 연결 순서 = 가상 노드ID (기획서 7.1-2). 허브=1, 멤버 k → k+1.
            for member_idx in range(self.scn_members + 1, cur + 1):
                vid = member_idx + 1
                self._log(f"[scn] 멤버{member_idx} 연결(가상ID {vid}) → "
                          f"{CUBE_COLORS.get(vid, vid)} LED", "scn")
                self._run(self.ble.send(build_set_led_solid(vid, NODE_RGB.get(vid, WHITE))))

        self.scn_members = cur
        if self.scn_members < target:
            return

        self.scn_done = True
        self._paint_buttons()
        if self.scn_fixed:
            # 각 BLE 멤버에 자기 노드ID 색 전송(허브가 target=노드ID로 중계).
            for nid in self.scn_ble_nodes[1:]:
                self._run(self.ble.send(build_set_led_solid(nid, NODE_RGB.get(nid, WHITE))))
            self._log(f"[R{self.active}] 고정형 완료 — BLE 노드 {self.scn_ble_nodes} 연결·색 지정", "scn")
        else:
            self._log(f"[R{self.active}] 완료 — 총 {self.scn_total}대 연결. "
                      f"다음: 네트워크 설정에서 큐브별 BLE/CAN 선택 후 저장(= 고정형 전환)", "scn")

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
        if fr.op_code == int(OpCode.GetSensors):
            # 센서 프레임은 주기적으로 쏟아지므로 로그를 채우지 않고 표에만 반영한다.
            self._on_sensor_frame(fr)
            return
        self._log(f"    └ {fr}", "rx")
        if fr.op_code == int(OpCode.TimeSync):
            # 왕복 측정 회신 — _timesync_coro가 기다리고 있다. 로그는 남기지 않는다
            # (수십 회 반복되므로 로그창을 덮는다).
            rep = parse_timesync_reply(fr.payload)
            if rep:
                self._ts_reply = rep
            return
        if fr.op_code == int(OpCode.GetMotorStatus):
            st = parse_motor_status(fr.payload)
            if st:
                err, pos, cur = st
                who = "허브" if fr.target_id == ADDR_HUB else f"노드 {fr.target_id}"
                self._log(f"    └ [모터] {who}: 위치 {pos:.2f}° 전류 {cur:.2f}A "
                          f"에러 0x{err:02X}", "scn")
            return
        if fr.op_code == int(OpCode.MotionComplete):
            mc = parse_motion_complete(fr.payload)
            if mc:
                reason, seq, pos, err = mc
                who = "허브" if fr.target_id == ADDR_HUB else f"노드 {fr.target_id}"
                tag = "err" if reason == 0x03 else "scn"
                self._log(f"    └ [모션완료] {who}: {MOTION_REASON.get(reason, reason)} "
                          f"seq={seq} pos={pos:.2f}° err=0x{err:02X}", tag)
            return
        if fr.op_code == int(OpCode.GetNodeConfig) and len(fr.payload) >= 4:
            # [group, node, cmf, term] — target은 허브가 재기입한 발신 큐브 가상ID.
            g, node, cmf, term = fr.payload[0], fr.payload[1], fr.payload[2], fr.payload[3]
            who = "허브(0xFE)" if fr.target_id == ADDR_HUB else f"큐브 {fr.target_id}"
            kind = "고정형" if node else "비고정형"
            self._log(f"    └ [설정] {who}: 그룹={g} 노드ID={node}({kind}) "
                      f"통신={'CAN' if cmf else 'BLE'} 종단노드={term}", "scn")
            return
        if fr.op_code == int(OpCode.GetEdgeCentralConfig) and len(fr.payload) >= 3 + MAX_NODES:
            ecf, unit_n, term = fr.payload[0], fr.payload[1], fr.payload[2]
            rows = []
            for i in range(MAX_NODES):
                v = fr.payload[3 + i]
                if v != MEMBER_NONE:
                    rows.append(f"{i + 1}:{'CAN' if v == MEMBER_CAN else 'BLE'}")
            self._log(f"    └ [독립] ECF={ecf}({'edge central' if ecf else '일반'}) "
                      f"유닛 {unit_n}대 종단노드={term} 멤버맵 [{', '.join(rows) or '없음'}]", "scn")
            return
        if fr.op_code == OP_AGGREGATOR_EVENT and self.active is not None:
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

    def on_get_config(self) -> None:
        """연결된 큐브들의 저장 설정을 조회한다(GetNodeConfig 0xD4).

        7.2-2 저장이 각 큐브에 실제로 반영됐는지 확인하는 용도. 멤버 회신은 BLE 허브가
        PC로 중계하므로, RX 로그에 큐브별 [group, node, cmf, term]이 찍힌다.
        """
        if not self.ble.is_connected:
            self._log("[설정조회] 연결이 없습니다.", "err")
            return
        # 멤버의 가상ID는 방식에 따라 다르다: 비고정형=연결순서(2..N), 고정형=저장 노드ID.
        if self.active and self.scn_fixed:
            vids = list(self.scn_ble_nodes[1:])
        elif self.active:
            vids = list(range(2, self.scn_total + 1))
        else:
            vids = []
        self._log(f"[설정조회] 허브 + 멤버 {len(vids)}대에 GetNodeConfig 전송", "scn")
        self._run(self.ble.send(build_get_node_config(target_id=ADDR_HUB)))
        for vid in vids:
            self._run(self.ble.send(build_get_node_config(target_id=vid)))

    # ---- 센서 모니터링 (기획서 9장) ----
    def on_sensor_stream(self, on: bool) -> None:
        """센서 전송 시작/중지. 브로드캐스트라 허브가 전 멤버로 중계한 뒤 자신도 적용한다."""
        if not self.ble.is_connected:
            self._log("[센서] 연결이 없습니다.", "err")
            return
        try:
            period = int(self.sens_period_var.get())
        except ValueError:
            self._log("[센서] 주기는 정수(ms)여야 합니다.", "err")
            return
        if on and not 20 <= period <= 10000:
            self._log("[센서] 주기는 20~10000 ms 범위여야 합니다.", "err")
            return
        self._log(f"[센서] 전송 {'시작' if on else '중지'} (주기 {period} ms) → 브로드캐스트", "scn")
        self._run(self.ble.send(build_set_sensor_stream(on, period)))

    def on_sensor_once(self) -> None:
        """유닛의 각 큐브에 1회 조회(GetSensors)."""
        if not self.ble.is_connected:
            self._log("[센서] 연결이 없습니다.", "err")
            return
        nodes = self._unit_nodes()
        self._log(f"[센서] 1회 조회 → 허브 + 멤버 {max(0, len(nodes) - 1)}대", "scn")
        self._run(self.ble.send(build_get_sensors(target_id=ADDR_HUB)))
        for nid in nodes[1:]:
            self._run(self.ble.send(build_get_sensors(target_id=nid)))

    def _on_sensor_frame(self, fr) -> None:
        """센서 프레임 수신 → 해당 노드 행 갱신. target은 발신 큐브(허브가 재기입)."""
        parsed = parse_sensor_payload(fr.payload)
        if parsed is None:
            return
        kind, x, y, z = parsed
        # 허브 자신이 보낸 프레임은 target=0xFE로 온다 → 유닛의 첫 큐브로 매핑.
        nodes = self._unit_nodes()
        nid = fr.target_id
        if nid == ADDR_HUB and nodes:
            nid = nodes[0]
        var = self.sens_labels.get(nid)
        if var is None:
            return
        prev = var.get()
        acc, gyr = "", ""
        if " | " in prev:
            acc, gyr = prev.split(" | ", 1)
        if kind == SENSOR_KIND_ACCEL:
            acc = f"acc {x:6d},{y:6d},{z:6d} mg"
        elif kind == SENSOR_KIND_GYRO:
            gyr = f"gyro {x / 10:7.1f},{y / 10:7.1f},{z / 10:7.1f} °/s"
        var.set(f"{acc or 'acc —':<28} | {gyr or 'gyro —'}")

    # ---- 모터 제어 (확장 규격 §2.11) ----
    def on_motor_move(self) -> None:
        t = self._motor_target()
        if t is None or not self.ble.is_connected:
            self._log("[모터] 대상/연결을 확인하세요.", "err")
            return
        try:
            deg = float(self.mot_deg.get())
            t_ms = int(self.mot_ms.get())
        except ValueError:
            self._log("[모터] 각도는 실수, 도달시간은 정수(ms)여야 합니다.", "err")
            return
        self._log(f"[모터] 노드 {self.mot_target.get()} → {deg}° / {t_ms}ms", "scn")
        self._run(self.ble.send(build_set_angle(deg, t_ms, target_id=t)))

    def on_motor_home(self) -> None:
        t = self._motor_target()
        if t is None or not self.ble.is_connected:
            return
        try:
            t_ms = int(self.mot_ms.get())
        except ValueError:
            t_ms = 0
        self._log(f"[모터] 노드 {self.mot_target.get()} → 원점(0°) / {t_ms}ms", "scn")
        self._run(self.ble.send(build_move_to_origin(t_ms, target_id=t)))

    def on_motor_set_origin(self) -> None:
        t = self._motor_target()
        if t is None or not self.ble.is_connected:
            return
        if not messagebox.askyesno("원점 설정",
                                   "현재 위치를 이 큐브의 원점으로 저장합니다. 진행할까요?"):
            return
        self._log(f"[모터] 노드 {self.mot_target.get()} 현재 위치를 원점으로 저장", "scn")
        self._run(self.ble.send(build_set_this_to_origin(target_id=t)))

    def on_drive_state(self, state: int) -> None:
        t = self._motor_target()
        if t is None or not self.ble.is_connected:
            return
        name = {DRIVE_ENABLE: "Enable", DRIVE_DISABLE: "Disable"}.get(state, str(state))
        self._log(f"[모터] 노드 {self.mot_target.get()} DriveState={name}", "scn")
        self._run(self.ble.send(build_set_drive_state(state, target_id=t)))

    def on_motor_status(self) -> None:
        t = self._motor_target()
        if t is None or not self.ble.is_connected:
            return
        self._run(self.ble.send(build_get_motor_status(target_id=t)))

    # ---- 시계 동기 (확장 규격 §3.2) ----
    def on_timesync(self) -> None:
        """유닛 전 큐브의 시계를 맞춘다. 다축 동기(T0)의 정확도가 여기서 결정된다."""
        if not self.ble.is_connected:
            self._log("[동기] 연결이 없습니다.", "err")
            return
        nodes = self._unit_nodes()
        if not nodes:
            return
        self._log(f"[동기] TimeSync 시작 — {len(nodes)}대, 각 {TIMESYNC_SAMPLES}회 왕복 측정", "scn")
        self._run(self._timesync_coro(nodes))

    async def _timesync_coro(self, nodes: list) -> None:
        for idx, nid in enumerate(nodes):
            target = ADDR_HUB if idx == 0 else nid
            best = None   # (rtt, offset)
            for _ in range(TIMESYNC_SAMPLES):
                self._ts_reply = None
                t1 = time.perf_counter_ns() // 1000
                await self.ble.send(build_timesync(t1, roundtrip=True, target_id=target))
                # 회신은 notify로 오므로 _handle_notify가 _ts_reply에 채워 준다.
                for _ in range(40):          # 최대 2초 대기
                    if self._ts_reply is not None:
                        break
                    await asyncio.sleep(0.05)
                if self._ts_reply is None:
                    continue
                t4 = time.perf_counter_ns() // 1000
                _echo, t_recv, t_send = self._ts_reply
                offset, rtt = timesync_solve(t1, t_recv, t_send, t4)
                # BLE는 connection event 대기로 지터가 크다 → 최소 RTT 표본이 가장 정확하다.
                if best is None or rtt < best[0]:
                    best = (rtt, offset)
                await asyncio.sleep(0.05)

            if best is None:
                self.ui_q.put(("log", f"[동기] 노드 {nid}: 회신 없음 — 건너뜀"))
                continue
            rtt, offset = best
            await self.ble.send(build_timesync(offset, offset=True, target_id=target))
            self.ui_q.put(("log", f"[동기] 노드 {nid}: offset {offset:+d} us "
                                  f"(최소 RTT {rtt} us) 적용"))
        self.ui_q.put(("log", "[동기] 완료 — 이제 ExecuteBuffer(Run, T0) 다축 동기가 유효합니다"))

    def on_estop(self) -> None:
        """비상 정지 — 브로드캐스트라 대상 선택과 무관하게 전 큐브가 멈춘다."""
        if not self.ble.is_connected:
            self._log("[모터] 연결이 없습니다.", "err")
            return
        self._log("[모터] ★ 비상 정지 브로드캐스트 — 전 큐브 큐 비움 + 게이트 차단", "err")
        self._run(self.ble.send(build_emergency_stop()))

    def on_demote_edge(self) -> None:
        """독립 해제(강등) — ECF=0 + 멤버 맵 삭제 (기획서 7.3 [독립→일반 되돌리기]).

        edge central은 연결모드에서 곧바로 central이 되어 PC에 붙지 않으므로, 되돌릴
        큐브는 설정모드(버튼 3초 롱프레스, RCUBECONFIG 광고)로 진입시켜 연결한 뒤 누른다.
        노드ID·통신방식은 유지된다(완전 초기화는 '공장 초기화').
        """
        if not self.ble.is_connected:
            self._log("[강등] 연결이 없습니다.", "err")
            return
        if not messagebox.askyesno(
                "독립 해제(강등)",
                "연결된 큐브의 ECF를 해제하고 멤버 맵을 삭제해 일반 큐브로 되돌립니다.\n"
                "노드ID·통신방식(CMF)은 그대로 유지됩니다.\n\n"
                "※ edge central은 설정모드(버튼 3초 롱프레스)로 연결한 상태여야 합니다.\n\n"
                "진행할까요?"):
            return
        self._log("[강등] SetEdgeCentralConfig(ECF=0) 전송 → 멤버 맵 삭제", "scn")
        empty = build_member_map({})
        self._run(self.ble.send(build_set_edge_central(0, 0, 0, empty, target_id=ADDR_HUB)))
        self._run(self.ble.send(build_get_edge_central(target_id=ADDR_HUB)))

    def on_reset_config(self) -> None:
        """공장 초기화(ResetConfig 0xD7) — 노드ID=0, CMF=BLE로 되돌리고 전체 재부팅."""
        if not self.ble.is_connected:
            self._log("[초기화] 연결이 없습니다.", "err")
            return
        if not messagebox.askyesno(
                "공장 초기화",
                "연결된 모든 큐브의 노드ID·통신방식(CMF)·종단ID를 공장값으로 되돌리고\n"
                "재부팅합니다(비고정형 상태로 복귀). 진행할까요?"):
            return
        self._log("[초기화] ResetConfig 브로드캐스트 → 전 큐브 공장 초기화 후 재부팅(연결 끊김)", "scn")
        self._run(self.ble.send(build_reset_config()))

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
        edge = bool(self.edge_var.get())

        if edge and not messagebox.askyesno(
                "고정로봇유닛(독립) 전환",
                f"노드01을 edge central(리드 큐브)로 만들고 멤버 맵을 저장합니다.\n\n"
                f"구성: [{desc}] · 종단노드 {term}\n\n"
                "저장 후 모든 큐브가 꺼집니다. 배선을 독립유닛 형태로 정리한 뒤\n"
                "(PC-리드 큐브 CAN 케이블 제거, 큐브끼리 노드ID 오름차순 연결)\n"
                "버튼으로 다시 켜면 리드 큐브가 스스로 멤버를 연결합니다.\n\n"
                "진행할까요?"):
            return

        # 기획서 7.2 step3: PC가 nodeID→CMF 매핑 테이블을 자기 메모리에 저장(고정형 재연결 근거).
        # 7.3에서는 이 표가 곧 edge central에 저장할 "멤버 맵"의 원본이다.
        self._save_netmap(n, choices, term)
        if edge:
            self._log(f"[독립] 저장 [{desc}] 종단노드={term} → 노드01에 ECF=1·멤버맵 저장 후 "
                      f"전체 전원끄기(7.3)", "scn")
        else:
            self._log(f"[네트워크] 저장 [{desc}] 종단노드={term} → 각 큐브 저장 후 전체 재부팅(연결 끊김)", "scn")
        self._run(self._save_netconf_coro(n, choices, term, edge))

    def _save_netmap(self, n: int, choices: dict, term: int) -> None:
        try:
            NETMAP_PATH.write_text(json.dumps(
                {"n": n, "term": term, "cmf": {str(v): c for v, c in choices.items()}},
                ensure_ascii=False, indent=2), encoding="utf-8")
            self._log(f"[네트워크] 매핑 저장: {NETMAP_PATH.name}")
            self._paint_mode_hint()   # 고정형 안내문에 새 구성 반영
        except Exception as e:
            self._log(f"[네트워크] 매핑 저장 실패: {e}", "err")

    def _load_netmap(self, quiet: bool = False):
        try:
            if not NETMAP_PATH.exists():
                return None
            return json.loads(NETMAP_PATH.read_text(encoding="utf-8"))
        except Exception as e:
            if not quiet:
                self._log(f"[고정형] 매핑 읽기 실패: {e}", "err")
            return None

    # ---- 고정형 CAN 분기 (기획서 7.2-7 [CAN 분기]) ----
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

    async def _save_netconf_coro(self, n: int, choices: dict, term: int, edge: bool) -> None:
        # 1) 각 큐브에 통신방식 세팅 저장(재부팅 없이). vid1=허브(0xFE), 그 외=멤버 중계.
        #    노드ID가 함께 저장되므로 이 시점에 고정형이 된다(기획서 7.5).
        for vid in range(1, n + 1):
            target = ADDR_HUB if vid == 1 else vid
            await self.ble.send(build_set_netconf(vid, choices[vid], term, target_id=target))
        await asyncio.sleep(0.2)

        if not edge:
            # 7.2-4: 저장이 끝나면 전체 재부팅. 다음 부팅부터 고정형으로 재연결한다.
            await self.ble.send(build_reboot_all())
            return

        # 7.3-3: 리드 큐브(노드01)에 ECF=1 + 전체 큐브 수 + 멤버 맵 + 종단노드ID를 저장.
        # (미션코드 업로드 F0~F2는 8장 확정 후 이 앞 단계에 들어간다.)
        member_map = build_member_map(choices)
        await self.ble.send(build_set_edge_central(1, n, term, member_map, target_id=ADDR_HUB))
        await asyncio.sleep(0.3)
        # 저장 확인(D6 회신은 로그에 해석되어 찍힌다).
        await self.ble.send(build_get_edge_central(target_id=ADDR_HUB))
        await asyncio.sleep(0.3)
        # 7.3-4: 모든 큐브에 shut down. 배선을 독립유닛 형태로 정리한 뒤 다시 켠다.
        await self.ble.send(build_shutdown())
        self.ui_q.put(("log", "[독립] 전원끄기 전송 — 배선 정리 후 리드 큐브부터 켜세요. "
                              "리드 큐브가 스스로 멤버를 연결합니다(7.4)."))

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
