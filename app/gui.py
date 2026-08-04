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
from tkinter import filedialog, messagebox, ttk

# 네트워크 세팅 매핑(nodeID→CMF) 저장 파일 (기획서 7.2 PC 매핑 테이블).
NETMAP_PATH = Path(__file__).resolve().parent / "netmap.json"

# 미션코드 소스(.py) 기본 폴더 — 기획서 7.3-2에서 사용자가 올리는 파일.
MISSION_DIR = Path(__file__).resolve().parent / "missions"

# 멜로디 테스트(임시) — 큐브가 아니라 PC 스피커로 재생한다.
from melody import MelodyPlayer, load_melodies

# 파이썬 미션 → 데이터 테이블(.rcm) 컴파일러 (기획서 8장 / 확장 규격 §2.5).
from rcube.mission import compile_file

from rcube import (
    OpCode,
    RCubeBLE,
    RCubeCAN,
    KNOWN_INTERFACES,
    DEFAULT_BITRATE,
    DEFAULT_INTERFACE,
    DEFAULT_CHANNEL,
    detect_adapters,
    pcan_driver_installed,
    build_frame,
    build_set_led,
    build_set_led_solid,
    build_set_aggregator,
    build_set_group,
    build_get_node_config,
    build_master_heartbeat,
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
    build_mission_upload_begin,
    build_mission_upload_chunk,
    build_mission_upload_commit,
    build_get_mission_info,
    parse_mission_info,
    mission_unit_sig,
    MISSION_STATE,
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

# 시나리오 시작 시 첫 큐브(비고정형) / BLE 허브(고정형)를 기다리는 시간.
# R버튼을 누른 뒤 큐브 버튼을 눌러 연결모드에 넣기까지 걸리는 시간을 감안한 값이다.
# STEP초 스캔을 TOTAL초까지 반복하되 발견 즉시 진행하므로, 이미 광고 중이면 지연이 없다.
SEARCH_TOTAL_S = 60.0
SEARCH_STEP_S = 5.0


BTN_IDLE = {"bg": "#b8b8b8", "fg": "#000000", "activebackground": "#a8a8a8"}
BTN_BUSY = {"bg": "#e69500", "fg": "#ffffff", "activebackground": "#cf8600"}
BTN_DONE = {"bg": "#111111", "fg": "#ffffff", "activebackground": "#333333"}
BTN_DIS = {"bg": "#dddddd", "fg": "#999999"}

# 큐브 연결 상태 박스: 대기=회색, 연결됨=검정
BOX_IDLE = {"bg": "#b8b8b8", "fg": "#555555"}
BOX_DONE = {"bg": "#111111", "fg": "#ffffff"}


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
        self.scn_branch_ok = {"can": True, "ble": True}   # 고정형 분기별 완료 여부
        self._ts_reply = None     # TimeSync 왕복 회신 (notify → _timesync_coro)

        root.title("R-Cube 제어 (BLE / CAN)")
        root.geometry("680x780")
        root.minsize(580, 660)

        # 멜로디 테스트(임시): 큐브와 무관하게 PC 스피커로만 재생한다.
        # 창은 도구 메뉴에서 열고, 엑셀은 그때 처음 읽는다(앱 시작을 늦추지 않는다).
        self.melodies: dict = {}
        self.mel_win = None
        self.melody_player = MelodyPlayer(on_log=lambda m: self.ui_q.put(("log", m)))

        self._build_ui()
        self.root.after(self.POLL_MS, self._pump_ui)
        self.root.after(1000, self._can_keepalive)   # CAN 연결 유지 하트비트
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # =====================================================================
    # UI 구성
    # =====================================================================
    def _build_ui(self) -> None:
        pad = dict(padx=6, pady=4)

        # 메뉴 — 창 크기와 무관하게 늘 보이는 자리. 임시 도구는 여기에 둔다
        # (본문 프레임에 넣으면 로그 패널에 밀려 화면 밖으로 잘린다).
        menubar = tk.Menu(self.root)
        tools = tk.Menu(menubar, tearoff=0)
        tools.add_command(label="멜로디 테스트 (PC 스피커)…", command=self.open_melody_window)
        menubar.add_cascade(label="도구", menu=tools)
        self.root.config(menu=menubar)

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
        # 기본값은 저장된 매핑 유무로 정한다(기획서 7.5). 통신방식 세팅에서 저장을 누른
        # 순간 각 큐브에 노드ID가 기록돼 고정형이 되므로, 그 뒤로는 앱을 다시 켜도 고정형이
        # 기본이어야 한다. 공장 초기화(D7)로 노드ID가 0이 되면 매핑도 지워져 비고정형으로
        # 돌아온다. 물론 사용자가 라디오로 비고정형을 다시 고를 수는 있다
        # (그때는 각 큐브를 설정모드로 BLE 연결해 재구성).
        self.fixed_var = tk.BooleanVar(value=bool(self._load_netmap(quiet=True)))
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

        # 큐브별 연결 상태 박스 — R{N} 진행 중 큐브 수만큼 표시한다.
        # 회색=대기, 검정=연결됨. 박스 아래 작은 글씨는 저장된 통신방식(BLE/CAN).
        self.cube_rowf = ttk.Frame(scn)
        self.cube_rowf.grid(row=3, column=0, columnspan=4, sticky="w", padx=8, pady=(2, 4))
        self.cube_boxes: dict[int, tk.Label] = {}
        self.cube_cmf_labels: dict[int, ttk.Label] = {}   # 박스 아래 통신방식 글씨
        self.cube_ids: list[int] = []                     # 이번 시나리오의 큐브 ID 목록
        self.cube_cmf: dict[int, int] = {}                # 큐브ID → 저장 CMF(0=BLE/1=CAN)
        # ★ 큐브ID → "지금 실제로 붙어 있는 경로"("ble"/"can"). 저장 CMF와 다를 수 있다:
        #   설정모드(RCUBECONFIG)에서는 CMF=CAN 큐브도 BLE로 붙고, 비고정형 초기구성도
        #   전부 BLE다. 명령을 저장 CMF로 보내면 그런 상황에서 아무 데도 닿지 않는다.
        self.cube_route: dict[int, str] = {}

        self.status_var = tk.StringVar(value="● 대기")
        self.status_lbl = ttk.Label(scn, textvariable=self.status_var)
        self.status_lbl.grid(row=4, column=0, columnspan=4, sticky="w", padx=8, pady=(0, 6))
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
        # 만든다(ECF=1 + 멤버 맵 + N 저장). 7.3-2: 체크하면 PC가 미션코드를 요청해
        # 리드 큐브에 업로드한다(아래 미션코드 줄).
        self.edge_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(netright, text="독립로봇유닛",
                        variable=self.edge_var, command=self._paint_net_save).pack(anchor="e")
        missf = ttk.Frame(netright)
        missf.pack(anchor="e", pady=(2, 0))
        self.mission_path: Path | None = None
        self._mission_info = None      # 마지막 F3(GetMissionInfo) 회신 — 업로드 확인용
        self.mission_var = tk.StringVar(value="미션코드: (없음)")
        ttk.Label(missf, textvariable=self.mission_var).pack(side="left", padx=(0, 4))
        ttk.Button(missf, text="선택…", width=6, command=self.on_pick_mission).pack(side="left")
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
        self.can_if.set(DEFAULT_INTERFACE)
        self.can_if.grid(row=0, column=1, sticky="w")
        ttk.Label(canf, text="channel").grid(row=0, column=2, sticky="e", padx=4)
        self.can_ch = tk.StringVar(value=DEFAULT_CHANNEL)
        self.can_ch_box = ttk.Combobox(canf, textvariable=self.can_ch, width=14)
        self.can_ch_box.grid(row=0, column=3, sticky="w")
        ttk.Label(canf, text="bitrate").grid(row=0, column=4, sticky="e", padx=4)
        self.can_br = tk.StringVar(value=str(DEFAULT_BITRATE))
        ttk.Entry(canf, textvariable=self.can_br, width=8).grid(row=0, column=5, sticky="w")
        ttk.Button(canf, text="장치검색", command=self.on_can_detect).grid(row=0, column=6, padx=3)
        ttk.Button(canf, text="열기", command=self.on_can_open).grid(row=0, column=7, padx=3)
        ttk.Button(canf, text="닫기", command=self.on_can_close).grid(row=0, column=8, padx=3)
        ttk.Button(canf, text="노드검색", command=self.on_can_discover).grid(row=0, column=9, padx=3)
        ttk.Button(canf, text="순서연결", command=self.on_can_connect_ordered).grid(row=0, column=10, padx=3)
        self.can_status_var = tk.StringVar(value="● CAN 닫힘")
        ttk.Label(canf, textvariable=self.can_status_var).grid(
            row=1, column=0, columnspan=11, sticky="w", padx=6, pady=(0, 4))

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

    # ---- 큐브 연결 상태 박스 ----
    def _set_cube_boxes(self, entries) -> None:
        """entries = [(큐브ID, "BLE"/"CAN"), …]. 빈 목록이면 박스를 지운다.

        ID는 고정형이면 저장 노드ID, 비고정형이면 연결 순서로 받을 가상ID다.
        """
        for w in self.cube_rowf.winfo_children():
            w.destroy()
        self.cube_boxes.clear()
        self.cube_cmf_labels.clear()
        for col, (cube_id, cmf) in enumerate(entries):
            cell = ttk.Frame(self.cube_rowf)
            cell.grid(row=0, column=col, padx=(0, 8))
            box = tk.Label(cell, text=f"{cube_id:02d}", width=4, height=2,
                           font=("", 10, "bold"), relief="raised", **BOX_IDLE)
            box.grid(row=0, column=0)
            sub = ttk.Label(cell, text=cmf, font=("", 7))
            sub.grid(row=1, column=0)
            self.cube_boxes[cube_id] = box
            self.cube_cmf_labels[cube_id] = sub

    def _mark_cube(self, cube_id: int, route: str = None) -> None:
        """해당 큐브를 '연결됨'(검정)으로 바꾼다. route를 주면 실제 연결 경로도 기록한다."""
        box = self.cube_boxes.get(cube_id)
        if box is not None:
            box.configure(**BOX_DONE)
        if route:
            self.cube_route[cube_id] = route

    def _route_of(self, cube_id: int) -> str:
        """이 큐브에 지금 명령을 보낼 경로. 연결 때 기록한 값이 우선이고, 없으면 저장 CMF.

        저장 CMF로 되돌아가는 것은 마지막 수단이다 — 설정모드처럼 "저장은 CAN인데 붙은
        건 BLE"인 상황에서 저장 CMF를 믿으면 명령이 어디에도 닿지 않는다.
        """
        r = self.cube_route.get(cube_id)
        if r:
            return r
        return "can" if self.cube_cmf.get(cube_id, 0) == 1 else "ble"

    def _apply_cube_cmf(self, cube_id: int, cmf: int) -> None:
        """큐브가 알려온 저장 통신방식을 상태 박스와 네트워크 설정 콤보에 반영한다."""
        self.cube_cmf[cube_id] = cmf
        text = "CAN" if cmf else "BLE"
        sub = self.cube_cmf_labels.get(cube_id)
        if sub is not None:
            sub.configure(text=text)
        cb = self.net_combos.get(cube_id)
        if cb is not None:
            cb.set(text)

    def _direct_ble_cube(self) -> int:
        """PC와 BLE로 직접 붙어 있는 큐브 ID — 그 큐브만 0xFE로 주고받는다.

        혼합 구성에서는 유닛의 첫 큐브(cube_ids[0])가 아니라 BLE 분기의 최소 노드ID다.
        예: 1·3=CAN, 2=BLE → PC에 붙은 것은 노드2이므로 0xFE는 노드2를 뜻한다.
        """
        if self.scn_fixed and self.scn_ble_nodes:
            return self.scn_ble_nodes[0]
        return self.cube_ids[0] if self.cube_ids else 1

    def _query_cube_configs(self) -> None:
        """각 큐브에 GetNodeConfig(D4)를 보내 저장된 통신방식을 읽어 온다.

        네트워크 설정 콤보를 "지금 큐브에 저장된 값"으로 미리 선택해 두기 위한 것이다.
        고정형은 netmap으로 이미 알지만 실제 큐브 값과 어긋났을 때(수동 변경·초기화)
        조회 결과가 우선한다. 비고정형은 가상ID와 큐브의 대응을 앱이 모르므로 이 조회가
        유일한 근거다. 회신은 _handle_notify의 GetNodeConfig 분기가 _apply_cube_cmf로
        넘긴다.
        """
        for cid in self.cube_ids:
            # 경로는 연결 때 기록한 실제 경로를 쓴다(설정모드에서는 CAN 큐브도 BLE로 붙는다).
            if self._route_of(cid) == "can" and self.can.is_connected:
                # CAN에서 0xFE는 "마스터에게"라는 뜻이라 큐브가 받지 않는다.
                # 첫 큐브라도 실제 노드ID로 지목해야 한다.
                try:
                    self.can.send(build_get_node_config(target_id=cid))
                except Exception as e:
                    self._log(f"[설정조회/CAN] 큐브 {cid} 전송 실패: {e!r}", "err")
            elif self.ble.is_connected:
                direct = self._direct_ble_cube()
                self._run(self.ble.send(
                    build_get_node_config(target_id=ADDR_HUB if cid == direct else cid)))

    # ---- 멜로디 테스트 (임시) — 도구 메뉴에서 여는 별도 창 ----
    def open_melody_window(self) -> None:
        """멜로디 테스트 창을 연다. 이미 열려 있으면 앞으로 가져온다."""
        if self.mel_win is not None and self.mel_win.winfo_exists():
            self.mel_win.lift()
            self.mel_win.focus_force()
            return
        if not self.melodies:
            try:
                self.melodies = load_melodies()
            except Exception as e:
                self._log(f"[멜로디] 로드 실패: {e!r} — openpyxl과 docs 엑셀을 확인하세요.", "err")
                messagebox.showerror("멜로디 테스트",
                                     f"멜로디 데이터를 읽지 못했습니다.\n\n{e!r}\n\n"
                                     f"app\\.venv\\Scripts\\python.exe 로 실행했는지 확인하세요.")
                return

        win = tk.Toplevel(self.root)
        win.title("멜로디 테스트 (PC 스피커 · 임시)")
        win.transient(self.root)
        win.resizable(False, False)
        self.mel_win = win

        names = list(self.melodies)
        ttk.Label(win, text="곡").grid(row=0, column=0, sticky="e", padx=(10, 4), pady=10)
        self.mel_cb = ttk.Combobox(win, state="readonly", width=34, values=names)
        self.mel_cb.grid(row=0, column=1, sticky="w", pady=10)
        self.mel_cb.bind("<<ComboboxSelected>>", lambda _e: self._show_melody_info())
        ttk.Button(win, text="재생", command=self.on_melody_play).grid(row=0, column=2, padx=4)
        ttk.Button(win, text="정지", command=self.on_melody_stop).grid(row=0, column=3, padx=(0, 10))

        self.mel_info_var = tk.StringVar()
        ttk.Label(win, textvariable=self.mel_info_var).grid(
            row=1, column=0, columnspan=4, sticky="w", padx=10, pady=(0, 4))
        ttk.Label(win, text="※ 큐브에는 아무것도 보내지 않습니다. PC 스피커로만 재생합니다.",
                  foreground="#888").grid(row=2, column=0, columnspan=4, sticky="w",
                                          padx=10, pady=(0, 10))
        if names:
            self.mel_cb.set(names[0])
            self._show_melody_info()

    def _show_melody_info(self) -> None:
        notes = self.melodies.get(self.mel_cb.get(), [])
        if notes:
            self.mel_info_var.set(
                f"{len(notes)}음 · {sum(d for _, d in notes):.2f}초 · 전체 {len(self.melodies)}곡")

    def on_melody_play(self) -> None:
        name = self.mel_cb.get()
        notes = self.melodies.get(name)
        if not notes:
            self._log("[멜로디] 재생할 곡을 고르세요.", "err")
            return
        self.melody_player.play(name, notes)

    def on_melody_stop(self) -> None:
        self.melody_player.stop()

    def _can_keepalive(self) -> None:
        """시나리오가 살아 있는 동안 1초마다 마스터 하트비트를 CAN에 뿌린다.

        CAN에는 BLE의 disconnect 이벤트가 없다. 이 신호가 끊기면(앱 종료·버스 분리)
        큐브가 상위는 사라졌다고 보고 지정색을 버린 뒤 미연결 점멸로 돌아간다
        (펌웨어 can_transport의 master_watch). 즉 이 주기 송신이 곧 '연결 유지'다.
        """
        if (self.active is not None and self.can.is_connected
                and any(r == "can" for r in self.cube_route.values())):
            try:
                self.can.send(build_master_heartbeat())
            except Exception:
                pass   # 버스가 막 닫혔을 수 있다 — 다음 주기에 조건이 걸러 준다
        self.root.after(1000, self._can_keepalive)

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
        # 행 순서는 상태 박스와 같은 큐브 ID 목록을 따른다(고정형=저장 노드ID,
        # 비고정형=가상ID). 목록이 없으면 1..n으로 둔다.
        ids = self.cube_ids if len(self.cube_ids) == n else list(range(1, n + 1))
        for row, cid in enumerate(ids):
            color = CUBE_COLORS.get(cid, f"vid{cid}")
            ttk.Label(self.net_rows, text=f"큐브 {cid} ({color})").grid(
                row=row, column=0, sticky="w", padx=(0, 8), pady=2)
            cb = ttk.Combobox(self.net_rows, state="readonly", width=6, values=["BLE", "CAN"])
            # 현재 큐브에 저장된 통신방식을 미리 선택해 둔다. 아직 모르면 공장 기본(BLE)로
            # 두고, 아래 조회 회신이 오는 대로 덮어쓴다.
            cb.set("CAN" if self.cube_cmf.get(cid) == 1 else "BLE")
            cb.grid(row=row, column=1, sticky="w", pady=2)
            self.net_combos[cid] = cb
        self.net_save_btn.configure(state="normal")
        self._query_cube_configs()   # 실제 큐브 저장값으로 맞춘다

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
            # CAN 분기는 PC가 직접 붙으므로 어댑터가 열려 있어야 한다. 사용자가 '열기'를
            # 따로 누르지 않아도 되도록 설정값을 여기서 캡처해 두고, CAN 작업 스레드가
            # 필요하면 스스로 연다(UI 변수는 UI 스레드에서만 읽는다).
            can_cfg = None
            if can_nodes:
                try:
                    can_cfg = (self.can_if.get(), self.can_ch.get().strip(),
                               int(self.can_br.get().strip()))
                except ValueError:
                    self._log("[CAN] bitrate는 정수여야 합니다. CAN 프레임 설정을 확인하세요.", "err")
                    return

        self.active = n
        self.scn_total = n
        self.scn_members = 0
        self.scn_done = False
        self.scn_fixed = fixed
        # 고정형은 BLE 분기만 허브가 취합한다(CAN 분기는 PC가 직접). 완료 판정 기준이 다르다.
        self.scn_ble_nodes = ble_nodes
        self.scn_ble_members = max(0, len(ble_nodes) - 1) if fixed else max(0, n - 1)
        # 고정형 완료 판정: 두 분기가 각각 끝났는지. 없는 분기는 처음부터 완료로 둔다.
        self.scn_branch_ok = {"can": not can_nodes, "ble": not ble_nodes}
        # 상태 박스: 고정형은 저장 노드ID와 저장된 통신방식, 비고정형은 가상ID(연결 순서).
        # 비고정형은 초기 구성 단계라 항상 BLE로 붙는다(기획서 7.1).
        # 실제 연결 경로는 이번 시나리오에서 다시 채운다(연결될 때마다 _mark_cube가 기록).
        self.cube_route = {}
        if fixed:
            self.cube_ids = sorted(ble_nodes + can_nodes)
            # 저장된 매핑을 그대로 안다 — 네트워크 설정 콤보의 초기 선택값이 된다.
            self.cube_cmf = {nid: (1 if nid in can_nodes else 0) for nid in self.cube_ids}
        else:
            # 비고정형은 가상ID와 큐브 저장값의 대응을 아직 모른다. 연결이 끝나면
            # D4(GetNodeConfig)로 각 큐브에 직접 물어 채운다(_query_cube_configs).
            self.cube_ids = list(range(1, n + 1))
            self.cube_cmf = {}
        boxes = [(cid, "CAN" if self.cube_cmf.get(cid) == 1 else
                       "BLE" if cid in self.cube_cmf else "…") for cid in self.cube_ids]
        self._set_cube_boxes(boxes)
        self._paint_buttons()

        if fixed:
            self._log(f"[R{n}] 시작 — 고정형 재연결 (7.2-7) · BLE {ble_nodes} · CAN {can_nodes}", "scn")
            can_only = bool(can_nodes) and not ble_nodes
            if can_nodes:
                # CAN 분기만인 구성에서는 CAN 발견 결과가 곧 완료 판정이다.
                self._run_can_fixed(can_nodes, complete_scenario=can_only, can_cfg=can_cfg)
            if ble_nodes:
                self._run_scn(n, self._scn_start_fixed(ble_nodes))
            elif can_only:
                self._log("[고정형] BLE 큐브가 없습니다 — CAN 분기만 진행합니다.", "scn")
        else:
            self._log(f"[R{n}] 시작 — 비고정형 초기 구성 (7.1) · 켜는 순서 = 가상 노드ID", "scn")
            self._run_scn(n, self._scn_start(n))

    async def _find_cube(self, what: str, match=None):
        """조건에 맞는 큐브가 광고할 때까지 최대 SEARCH_TOTAL_S초 기다린다(발견 즉시 반환).

        R버튼을 누른 뒤 사용자가 큐브 버튼을 눌러 연결모드에 넣기까지 시간이 걸리므로,
        5초 스캔 한 번으로 끊지 않는다. R버튼을 다시 눌러 시나리오를 해제하면 중단된다.
        """
        def progress(elapsed: float, total: float) -> None:
            self.ui_q.put(("log", f"[검색] {what} 광고 대기… ({int(elapsed)}/{int(total)}초) "
                                  f"— 큐브 버튼을 눌러 연결모드에 두세요. R버튼 재클릭=취소"))

        hits = await RCubeBLE.scan_for(
            match, total=SEARCH_TOTAL_S, step=SEARCH_STEP_S,
            on_progress=progress, should_stop=lambda: self.active is None)
        if not hits:
            if self.active is None:
                raise RuntimeError(f"{what} 검색이 취소되었습니다.")
            raise RuntimeError(f"{int(SEARCH_TOTAL_S)}초 안에 {what}를 찾지 못했습니다. "
                               f"전원과 연결모드(버튼)를 확인하세요.")
        return hits[0]

    async def _scn_start(self, n: int) -> None:
        """기획서 7.1 공통 1단계 — 비고정형 BLE로 전체 구성.

        사용자가 정한 순서대로 큐브를 켜면, PC에 처음 연결된 큐브가 가상1(Red)이 되어
        BLE 허브가 되고 나머지를 연결 순서대로 취합한다. 노드ID는 저장하지 않는다.
        """
        # 1) 가장 먼저 광고 중인 R큐브에 연결 = 사용자가 첫 번째로 켠 큐브 = 가상1.
        first = await self._find_cube("첫 큐브(가상1)")
        self.ui_q.put(("log", f"[검색] 발견: {first}"))
        await self.ble.connect(first.address)
        self.ui_q.put(("cube_connected", (1, "ble")))   # 가상1 = 허브
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
        found = await self._find_cube(f"BLE 허브(노드{hub})",
                                      match=lambda r: _parse_nn(r.name) == hub)
        self.ui_q.put(("log", f"[검색] 발견: {found}"))
        await self.ble.connect(found.address)
        self.ui_q.put(("cube_connected", (hub, "ble")))
        # 7.2-7 ★: 허브는 무조건 Red가 아니라 자기 노드ID 색으로 켜진다.
        await self.ble.send(build_set_led_solid(ADDR_HUB, NODE_RGB.get(hub, WHITE)))
        if count == 1:
            self.ui_q.put(("log", "[고정형/BLE] 단일 BLE 큐브 연결 완료"))
            self.ui_q.put(("branch_done", "ble"))
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
        self.cube_ids = []
        self.cube_cmf = {}
        self.cube_route = {}
        self._set_cube_boxes([])
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
            # 상태 박스는 허브가 노드ID 순으로 붙이는 순서를 따라 채운다(허브=index 0).
            for member_idx in range(self.scn_members + 1, cur + 1):
                if member_idx < len(self.scn_ble_nodes):
                    self._mark_cube(self.scn_ble_nodes[member_idx], "ble")
        else:
            # 비고정형: 연결 순서 = 가상 노드ID (기획서 7.1-2). 허브=1, 멤버 k → k+1.
            for member_idx in range(self.scn_members + 1, cur + 1):
                vid = member_idx + 1
                self._log(f"[scn] 멤버{member_idx} 연결(가상ID {vid}) → "
                          f"{CUBE_COLORS.get(vid, vid)} LED", "scn")
                self._mark_cube(vid, "ble")
                self._run(self.ble.send(build_set_led_solid(vid, NODE_RGB.get(vid, WHITE))))

        self.scn_members = cur
        if self.scn_members < target:
            return

        if self.scn_fixed:
            # 혼합 구성이면 CAN 분기도 끝나야 완료다 — join은 _pump_ui가 판정한다.
            self.ui_q.put(("branch_done", "ble"))
        else:
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
                elif kind == "cube_connected":
                    # payload = (큐브ID, 경로) — 예전 형태(정수)도 받아 준다.
                    cid, route = payload if isinstance(payload, tuple) else (payload, None)
                    self._mark_cube(int(cid), route)
                elif kind == "scn_reset":
                    # 저장(재부팅)·전원끄기처럼 전 큐브가 사라지는 동작 뒤에는 시나리오를
                    # 접어야 한다. CAN에는 BLE 같은 disconnect 이벤트가 없어서, 이 신호가
                    # 없으면 R버튼이 계속 진행중으로 남고 네트워크 설정도 열린 채 있는다.
                    if self.active is not None:
                        self._log(f"[R{self.active}] {payload} — 초기 상태로", "scn")
                        self.active = None
                        self.scn_done = False
                        self.scn_members = 0
                        self.cube_ids = []
                        self.cube_cmf = {}
                        self.cube_route = {}
                        self._set_cube_boxes([])
                        self._paint_buttons()
                        self._set_status("● 대기")
                        if self.ble.is_connected:
                            self._run(self.ble.disconnect())
                elif kind == "netmap_save":
                    # 저장 명령이 실제로 나간 뒤에 앱의 매핑을 갱신한다(코루틴 → UI 스레드).
                    self._save_netmap(*payload)
                elif kind == "branch_done":
                    # 고정형은 CAN 분기와 BLE 분기가 따로 진행된다. 둘 다 끝나야 완료다
                    # (한쪽만 보고 완료로 두면 나머지가 빠진 채 네트워크 설정이 열린다).
                    self.scn_branch_ok[str(payload)] = True
                    self._log(f"[고정형] {str(payload).upper()} 분기 완료", "scn")
                    if self.active is not None and not self.scn_done \
                            and all(self.scn_branch_ok.values()):
                        self.scn_done = True
                        self._paint_buttons()
                        self._log(f"[R{self.active}] 완료 — 전 분기 연결", "scn")
                elif kind == "scn_failed":
                    n, err = payload
                    self._log(f"[R{n}] 실패: {err}", "err")
                    if self.active == n:
                        self.active = None
                        self.scn_done = False
                        self.scn_members = 0
                        self.cube_ids = []
                        self.cube_cmf = {}
                        self.cube_route = {}
                        self._set_cube_boxes([])
                        self._paint_buttons()
                    self._run(self.ble.disconnect())
                elif kind == "error":
                    self._log(f"[오류] {payload}", "err")
                elif kind == "scan":
                    self._apply_scan(payload)
                elif kind == "can_state":
                    if payload:
                        self.can_status_var.set(f"● CAN 열림 — 버스 {self.can.state_text()}")
                    else:
                        self.can_status_var.set("● CAN 닫힘")
                elif kind == "can_channels":
                    chans, iface = payload
                    self.can_ch_box["values"] = chans
                    if chans:
                        self.can_ch.set(chans[0])
                    if iface in KNOWN_INTERFACES:
                        self.can_if.set(iface)
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
            # 상태 박스·네트워크 설정 콤보를 실제 저장값으로 맞춘다.
            # 어느 큐브의 회신인지 고르는 순서:
            #   1) 고정형이면 회신에 실린 노드ID가 가장 확실하다(박스도 노드ID로 매긴다)
            #   2) 0xFE로 온 것은 PC와 BLE로 직접 붙은 큐브의 회신
            #   3) 그 외에는 허브가 재기입한 발신 큐브 ID
            if self.scn_fixed and node:
                cid = node
            elif fr.target_id == ADDR_HUB:
                cid = self._direct_ble_cube()
            else:
                cid = fr.target_id
            self._apply_cube_cmf(cid, 1 if cmf else 0)
            return
        if fr.op_code == int(OpCode.GetMissionInfo):
            info = parse_mission_info(fr.payload)
            self._mission_info = info      # 업로드 확인(_upload_mission)이 기다린다
            if info is None:
                self._log("    └ [미션] 회신 형식이 아닙니다", "err")
            elif not info.get("valid"):
                self._log(f"    └ [미션] 상태={MISSION_STATE.get(info['state'], info['state'])} "
                          f"— 적재된 미션 없음", "err")
            else:
                self._log(f"    └ [미션] 상태={MISSION_STATE.get(info['state'], info['state'])} "
                          f"'{info['name']}' type={info['type']} "
                          f"{info['body_len']}B({info['body_len'] // 8}키프레임) "
                          f"crc=0x{info['body_crc']:08X} unit_sig=0x{info['unit_sig']:08X}", "scn")
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
            self.cube_ids = []
            self.cube_cmf = {}
            self.cube_route = {}
            self._set_cube_boxes([])
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
        # 노드ID=0(미할당)로 돌아갔으므로 고정형 재연결 근거가 사라진다 → 매핑 폐기 +
        # 라디오도 비고정형으로 되돌린다(기획서 7.3 [노드ID/세팅 초기화 - 공통]).
        try:
            NETMAP_PATH.unlink(missing_ok=True)
            self._log("[초기화] 저장된 매핑 폐기 → 비고정형으로 복귀")
        except OSError as e:
            self._log(f"[초기화] 매핑 파일 삭제 실패: {e}", "err")
        self.fixed_var.set(False)
        self._paint_mode_hint()

    # ---- 미션코드 (기획서 7.3-2 · 8장) ----
    def on_pick_mission(self) -> None:
        """사용자에게 미션코드를 받는다. 파이썬 소스(.py)면 저장 시 컴파일해 올린다."""
        path = filedialog.askopenfilename(
            title="미션코드 선택 — 파이썬 소스(.py) 또는 컴파일된 미션(.rcm)",
            initialdir=str(MISSION_DIR if MISSION_DIR.is_dir() else Path.cwd()),
            filetypes=[("미션 소스/코드", "*.py *.rcm"), ("모든 파일", "*.*")])
        if not path:
            return
        self.mission_path = Path(path)
        self.mission_var.set(f"미션코드: {self.mission_path.name}")
        self._log(f"[미션] 선택: {self.mission_path}")

    def _compile_mission(self, ids: list, choices: dict):
        """선택한 미션을 '지금 저장할 구성'으로 컴파일한다. 실패하면 None.

        노드 목록과 유닛 서명(unit_sig)을 이 저장의 멤버 맵에서 뽑는 것이 핵심이다 —
        다른 구성에 올라간 미션은 큐브가 실행을 거부한다(확장 규격 §2.5.1 안전핀).
        """
        if self.mission_path is None:
            return None
        try:
            sig = mission_unit_sig(len(ids), build_member_map(choices))
            m = compile_file(self.mission_path, nodes=list(ids), cmf=dict(choices),
                             unit_sig=sig)
        except Exception as e:
            self._log(f"[미션] 컴파일 실패: {e}", "err")
            messagebox.showerror("미션 컴파일 실패", str(e))
            return None
        self._log(f"[미션] {m.summary()}", "scn")
        for line in m.timeline:
            self._log(line)
        return m

    def on_save_netconf(self) -> None:
        """큐브별 통신방식(BLE/CAN)을 저장시키고 전체 재부팅(기획서 7.2). 다음 부팅부터 적용."""
        if self.active is None or not self.scn_done:
            self._log("[네트워크] 모든 큐브가 연결 완료된 뒤 저장하세요.", "err")
            return
        # ※ 여기서 BLE 연결을 일괄로 요구하면 안 된다 — CAN 전용 구성(전 큐브 CMF=CAN)은
        #   BLE 연결이 아예 없다. 필요한 전송로는 아래에서 큐브별 현재 CMF로 따진다.
        n = self.scn_total
        # 대상 ID는 네트워크 설정 행과 같은 목록을 쓴다(고정형=저장 노드ID, 비고정형=가상ID).
        ids = self.cube_ids if len(self.cube_ids) == n else list(range(1, n + 1))
        choices: dict[int, int] = {}
        can_vids = []
        for cid in ids:
            cb = self.net_combos.get(cid)
            cmf = 1 if (cb is not None and cb.get() == "CAN") else 0
            choices[cid] = cmf
            if cmf == 1:
                can_vids.append(cid)
        term = max(can_vids) if can_vids else 0
        desc = ", ".join(f"{cid}:{'CAN' if choices[cid] else 'BLE'}" for cid in ids)
        edge = bool(self.edge_var.get())

        # 7.3-2: "체크하면 PC가 사용자에게 미션코드를 요청한다." 아직 안 골랐으면 지금 묻고,
        # 그래도 없으면 미션 없이 갈 것인지 확인한다(연결만 하고 아무 동작도 하지 않는 유닛).
        mission = None
        if edge:
            if self.mission_path is None:
                self.on_pick_mission()
            if self.mission_path is None:
                if not messagebox.askyesno(
                        "미션코드 없음",
                        "미션코드 없이 독립로봇유닛으로 전환할까요?\n\n"
                        "미션이 없으면 모든 큐브가 연결돼도 유닛은 아무 동작도 하지 않습니다."):
                    return
            else:
                mission = self._compile_mission(ids, choices)
                if mission is None:
                    return   # 컴파일 실패 — 전환 자체를 진행하지 않는다

        if edge and not messagebox.askyesno(
                "독립로봇유닛 전환",
                f"노드01을 edge central(리드 큐브)로 만들고 멤버 맵을 저장합니다.\n\n"
                f"구성: [{desc}] · 종단노드 {term}\n"
                f"미션코드: {mission.summary() if mission else '없음'}\n\n"
                "저장 후 모든 큐브가 꺼집니다. 배선을 독립유닛 형태로 정리한 뒤\n"
                "(PC-리드 큐브 CAN 케이블 제거, 큐브끼리 노드ID 오름차순 연결)\n"
                "버튼으로 다시 켜면 리드 큐브가 스스로 멤버를 연결하고,\n"
                "모두 연결되면 이 미션이 자동으로 실행됩니다(7.4-6).\n\n"
                "진행할까요?"):
            return

        # ★저장 명령은 "지금 실제로 붙어 있는 경로"로 나가야 한다. 새 통신방식은 재부팅
        #   후에나 유효하고, 저장된 CMF도 지금 경로와 다를 수 있다 — 설정모드
        #   (RCUBECONFIG)에서는 CMF=CAN 큐브도 BLE로 붙는다. 저장 CMF로 경로를 고르면
        #   그 상황에서 "CAN 버스가 없다"며 아무것도 못 보내고 멈춘다(2026-08-04 실사례).
        #   매핑 파일 갱신은 전송이 끝난 뒤에 한다(실패했는데 앱만 새 구성을 기억하면
        #   다음 고정형 연결이 어긋난다).
        routes = {cid: self._route_of(cid) for cid in ids}
        direct_ble = self._direct_ble_cube()
        need_ble = any(r == "ble" for r in routes.values())
        need_can = any(r == "can" for r in routes.values())
        rdesc = ", ".join(f"{cid}:{routes[cid].upper()}" for cid in ids)
        if need_ble and not self.ble.is_connected:
            msg = (f"BLE로 붙어 있는 큐브가 있는데 BLE 연결이 없습니다.\n현재 경로: [{rdesc}]")
            self._log(f"[네트워크] {msg}", "err")
            messagebox.showwarning("저장할 수 없습니다", msg)
            return
        if need_can and not self.can.is_connected:
            msg = (f"CAN으로 붙어 있는 큐브가 있는데 CAN 버스가 열려 있지 않습니다.\n"
                   f"현재 경로: [{rdesc}]\n\nCAN 프레임에서 '열기'를 누른 뒤 다시 저장하세요.")
            self._log(f"[네트워크] {msg}", "err")
            messagebox.showwarning("저장할 수 없습니다", msg)
            return
        self._log(f"[네트워크] 전송 경로 [{rdesc}] (저장된 CMF가 아니라 지금 붙어 있는 경로)")

        if edge:
            self._log(f"[독립] 저장 [{desc}] 종단노드={term} → 노드01에 ECF=1·멤버맵 저장 후 "
                      f"전체 전원끄기(7.3)", "scn")
        else:
            self._log(f"[네트워크] 저장 [{desc}] 종단노드={term} → 각 큐브 저장 후 전체 재부팅(연결 끊김)", "scn")
        self._run(self._save_netconf_coro(ids, choices, term, edge, routes, direct_ble,
                                          mission))

    def _save_netmap(self, n: int, choices: dict, term: int) -> None:
        try:
            NETMAP_PATH.write_text(json.dumps(
                {"n": n, "term": term, "cmf": {str(v): c for v, c in choices.items()}},
                ensure_ascii=False, indent=2), encoding="utf-8")
            self._log(f"[네트워크] 매핑 저장: {NETMAP_PATH.name}")
            # 저장 = 노드ID/CMF 기록 = 고정형 전환(기획서 7.5). 다음 연결부터는 고정형이
            # 기본이므로 라디오도 여기서 고정형으로 옮긴다(앱 재시작 시엔 매핑 파일이 근거).
            self.fixed_var.set(True)
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
    def _run_can_fixed(self, can_nodes: list, complete_scenario: bool = False,
                       can_cfg=None) -> None:
        def per(nid, index):
            rgb = NODE_RGB.get(nid, (255, 255, 255))
            try:
                self.can.send(build_set_led(nid, [rgb]))
            except Exception:
                pass
            self.ui_q.put(("log", f"[고정형/CAN] 노드 0x{nid:02X} 연결(순서 {index + 1})"))
            self.ui_q.put(("cube_connected", (nid, "can")))

        def work():
            # 버스가 닫혀 있으면 알아서 연다 — CAN 분기는 어댑터 없이는 시작조차 못 하므로
            # 사용자가 'CAN 열기'를 따로 누르게 할 이유가 없다(기획서 7.2-7은 PC가 CAN
            # 분기를 직접 취합한다고만 정한다).
            if not self.can.is_connected:
                if can_cfg is None:
                    self.ui_q.put(("log", "[CAN] 버스가 닫혀 있습니다 — CAN 프레임에서 '열기'를 "
                                          "누르세요."))
                    return
                iface, chan, br = can_cfg
                self.ui_q.put(("log", f"[CAN] 버스 자동 열기… ({iface}/{chan} @{br})"))
                try:
                    self.can.open(iface, chan, br)
                except Exception as e:
                    self.ui_q.put(("log", f"[CAN] 자동 열기 실패: {e!r} — 어댑터 연결과 채널을 "
                                          f"확인하세요. PCAN-View가 떠 있으면 채널을 점유하므로 "
                                          f"먼저 종료해야 합니다."))
                    if complete_scenario:
                        self.ui_q.put(("scn_failed", (scn_n, "CAN 버스를 열지 못했습니다")))
                    return

            found = self.can.connect_ordered(3.0, per_node=per)
            missing = [n for n in can_nodes if n not in found]
            msg = f"[고정형/CAN] 발견 {[hex(n) for n in found]}"
            if missing:
                msg += f" · 미발견 {[hex(n) for n in missing]}(전원/버스 확인)"
            self.ui_q.put(("log", msg))
            # 완료는 담당 노드를 전부 찾았을 때만. 못 찾았는데 완료로 두면 이어지는
            # 네트워크 설정·센서·모터가 없는 큐브를 대상으로 열린다.
            # 혼합 구성에서는 BLE 분기까지 끝나야 시나리오가 완료된다(_pump_ui의 join).
            if not missing:
                self.ui_q.put(("branch_done", "can"))
        threading.Thread(target=work, daemon=True).start()

    async def _save_netconf_coro(self, ids: list, choices: dict, term: int, edge: bool,
                                 routes: dict, direct_ble: int, mission=None) -> None:
        n = len(ids)

        async def send_to(cid: int, build):
            """★지금 실제로 붙어 있는 경로로 그 큐브에 보낸다(routes).

            새 통신방식은 재부팅 후에나 유효하므로 저장 명령 자체는 기존 경로로 가야 한다.
            그 "기존 경로"는 저장된 CMF가 아니라 **연결 때 기록한 실제 경로**다 —
            설정모드에서는 CMF=CAN 큐브도 BLE로 붙어 있다.
            CAN에서는 0xFE가 "마스터에게"라 큐브가 받지 않으므로 항상 실제 노드ID로 지목하고,
            BLE에서는 PC에 직접 붙은 큐브만 0xFE로 보낸다(나머지는 허브가 중계).
            """
            if routes.get(cid) == "can":
                self.can.send(build(cid))
            else:
                await self.ble.send(build(ADDR_HUB if cid == direct_ble else cid))

        async def broadcast(build):
            """재부팅·전원끄기처럼 전 큐브 대상 명령은 살아 있는 두 버스 모두로 보낸다."""
            if self.ble.is_connected:
                await self.ble.send(build())
            if self.can.is_connected:
                self.can.send(build())

        # 0) 먼저 전 큐브의 ECF를 해제한다(멤버 맵도 삭제).
        #    이전 구성에서 edge central이었던 큐브가 섞여 있으면, 이번에 서브로봇유닛으로
        #    저장해도 그 큐브는 다음 부팅에서 다시 혼자 central이 되어 PC에 붙지 않는다.
        #    독립로봇유닛으로 저장하는 경우엔 아래 7.3-3 단계에서 리드 큐브만 ECF=1로 올린다.
        #    ※ D5는 term_id도 함께 쓰므로 1)의 SET_NETCONF보다 먼저 보낸다(term 보존).
        empty_map = build_member_map({})
        for cid in ids:
            await send_to(cid, lambda t: build_set_edge_central(0, 0, term, empty_map,
                                                                target_id=t))
        await asyncio.sleep(0.2)
        self.ui_q.put(("log", f"[네트워크] 전 큐브 ECF 해제(ECF=0 · 멤버맵 삭제) {n}대"))

        # 1) 각 큐브에 통신방식 세팅 저장(재부팅 없이).
        #    노드ID가 함께 저장되므로 이 시점에 고정형이 된다(기획서 7.5).
        for cid in ids:
            await send_to(cid, lambda t, c=cid: build_set_netconf(c, choices[c], term,
                                                                  target_id=t))
        await asyncio.sleep(0.3)

        # 2) 전송이 끝난 뒤에야 앱의 매핑을 새 구성으로 갱신한다 — 다음 고정형 연결의 근거.
        self.ui_q.put(("netmap_save", (n, choices, term)))

        if not edge:
            # 7.2-4: 저장이 끝나면 전체 재부팅. 다음 부팅부터 새 통신방식으로 재연결한다.
            await broadcast(build_reboot_all)
            self.ui_q.put(("scn_reset", "전체 재부팅 — 연결 해제"))
            return

        # 7.3-2: 리드 큐브(노드01)에 미션코드를 업로드한다. ECF 저장(7.3-3)보다 먼저 하는
        # 이유는 기획서 순서(요청 → 업로드 → 저장) 그대로이기도 하고, 업로드가 실패하면
        # 독립 전환을 멈출 수 있어야 하기 때문이다.
        lead = ids[0]
        if mission is not None:
            if not await self._upload_mission(lead, mission, send_to):
                self.ui_q.put(("log", "[미션] 업로드 실패 — 독립 전환을 중단합니다. "
                                      "큐브 로그(F0/F1/F2 CmdAck)를 확인하세요."))
                return

        # 7.3-3: 리드 큐브(노드01)에 ECF=1 + 전체 큐브 수 + 멤버 맵 + 종단노드ID를 저장.
        member_map = build_member_map(choices)
        await send_to(lead, lambda t: build_set_edge_central(1, n, term, member_map, target_id=t))
        await asyncio.sleep(0.3)
        # 저장 확인(D6 회신은 로그에 해석되어 찍힌다).
        await send_to(lead, lambda t: build_get_edge_central(target_id=t))
        await asyncio.sleep(0.3)
        # 7.3-4: 모든 큐브에 shut down. 배선을 독립유닛 형태로 정리한 뒤 다시 켠다.
        await broadcast(build_shutdown)
        self.ui_q.put(("log", "[독립] 전원끄기 전송 — 배선 정리 후 리드 큐브부터 켜세요. "
                              "리드 큐브가 스스로 멤버를 연결합니다(7.4)."))
        self.ui_q.put(("scn_reset", "전체 전원끄기 — 연결 해제"))

    # 한 번에 보낼 본문 조각 크기. CAN 멀티프레임(최대 446B)과 BLE MTU 양쪽에 안전하고,
    # 8바이트 레코드 경계에 딱 맞는 값으로 잡는다(레코드가 조각을 걸치지 않아 디버깅이 쉽다).
    MISSION_CHUNK = 64

    async def _upload_mission(self, lead: int, mission, send_to) -> bool:
        """리드 큐브에 미션 본문을 올린다(F0 → F1×n → F2 → F3 확인). 성공하면 True.

        조각마다 CmdAck을 기다리지는 않고 간격을 두고 밀어 넣되(큐브가 F0에서 플래시
        슬롯을 소거하므로 첫 대기만 넉넉히 준다), **F3 회신으로 실제로 적재됐는지 확인**
        한다. 길이·CRC가 우리가 보낸 것과 같아야 통과다 — 조각 하나가 유실되면 큐브가
        업로드를 중단하므로, 확인 없이 넘어가면 미션 없는 유닛이 조용히 완성된다.
        """
        import zlib

        body = mission.body
        crc = zlib.crc32(body) & 0xFFFFFFFF
        self.ui_q.put(("log", f"[미션] 노드{lead}에 업로드: {mission.name} "
                              f"{len(body)}B / {mission.record_count}키프레임 "
                              f"crc=0x{crc:08X} unit_sig=0x{mission.unit_sig:08X}"))
        self._mission_info = None
        try:
            # 업로드 전에 센서 주기전송을 끈다. 켜져 있으면 큐브들이 링크를 계속 채워
            # 조각 전송이 밀리고, 로그도 파묻혀 실패 원인을 볼 수 없다.
            await send_to(lead, lambda t: build_set_sensor_stream(False,
                                                                 target_id=ADDR_BROADCAST))
            await asyncio.sleep(0.3)
            await send_to(lead, lambda t: build_mission_upload_begin(
                len(body), crc, mission.unit_sig, target_id=t))
            await asyncio.sleep(0.6)   # 슬롯 소거 시간

            seq = 0
            for off in range(0, len(body), self.MISSION_CHUNK):
                chunk = body[off:off + self.MISSION_CHUNK]
                await send_to(lead, lambda t, s=seq, c=chunk:
                              build_mission_upload_chunk(s, c, target_id=t))
                seq += 1
                await asyncio.sleep(0.15)

            await send_to(lead, lambda t: build_mission_upload_commit(target_id=t))
            await asyncio.sleep(0.5)
            await send_to(lead, lambda t: build_get_mission_info(target_id=t))
        except Exception as e:
            self.ui_q.put(("log", f"[미션] 전송 오류: {e!r}"))
            return False

        for _ in range(20):            # F3 회신 최대 2초 대기
            if self._mission_info is not None:
                break
            await asyncio.sleep(0.1)
        info = self._mission_info
        if info is None:
            self.ui_q.put(("log", "[미션] F3 회신이 없습니다 — 리드 큐브와의 경로를 확인하세요."))
            return False
        if not info.get("valid") or info["body_len"] != len(body) or info["body_crc"] != crc:
            self.ui_q.put(("log", f"[미션] 적재 확인 실패 — 큐브가 보고한 내용이 "
                                  f"보낸 것과 다릅니다({info})"))
            return False
        self.ui_q.put(("log", f"[미션] 업로드 완료({seq}조각) — 리드 큐브에 '{info['name']}' 적재됨"))
        return True

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
    def on_can_detect(self) -> None:
        """꽂혀 있는 USB-CAN 어댑터를 찾아 채널 목록을 채운다."""
        self._log("[CAN] USB-CAN 어댑터 검색 중…")

        def work():
            cfgs = detect_adapters()
            if cfgs:
                chans = [str(c.get("channel")) for c in cfgs]
                self.ui_q.put(("can_channels", (chans, cfgs[0].get("interface", ""))))
                for c in cfgs:
                    self.ui_q.put(("log", f"[CAN] 발견: {c.get('interface')}/{c.get('channel')}"
                                          f" {c.get('device_id', '')}"))
            elif not pcan_driver_installed():
                self.ui_q.put(("error", "PCANBasic.dll 없음 — PEAK 'PCAN-Driver for Windows'를 "
                                        "설치하고 옵션에서 'PCAN-Basic API'를 함께 선택하세요."))
            else:
                self.ui_q.put(("error", "USB-CAN 어댑터를 찾지 못했습니다 — USB 연결을 확인하세요."))
        threading.Thread(target=work, daemon=True).start()

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
        """고정형 CAN 연결(기획서 7.2 CAN 분기): 하트비트로 노드 발견 → 노드ID 순서대로 연결.

        can_cfg=(interface, channel, bitrate)를 주면 버스가 닫혀 있을 때 스스로 연다.
        """
        scn_n = self.scn_total
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
