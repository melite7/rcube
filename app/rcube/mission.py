"""
mission.py — 파이썬 미션 소스를 R큐브 미션코드(.rcm)로 컴파일한다.

기획서 8장은 미션코드의 형태를 두 가지로 둔다.
  (1) 데이터 테이블  : edge central이 표를 보고 각 큐브를 중앙 제어한다. C 구현이라
                       파이썬 없이 항상 동작한다(12.4).
  (2) 마이크로파이썬 : 큐브 안의 MicroPython VM이 사용자 코드를 직접 실행한다.

(2)는 로드맵 Phase 11(MicroPython 임베딩)이라 아직 큐브에 VM이 없다. 그래서 여기서는
**사용자가 파이썬으로 미션을 쓰고, PC가 그것을 실행해 (1)의 데이터 테이블로 컴파일**한다.
사용자 입장의 저작 언어는 파이썬이고, 큐브에 올라가는 것은 "컴파일된" 이진 미션코드
(.rcm 컨테이너 = 헤더 32B + 8바이트 키프레임 배열)다. Phase 11이 끝나면 같은 소스를
mpy-cross로 굽는 경로(TYPE=3)를 이 옆에 붙이면 된다 — 컨테이너 type만 달라진다.

미션 소스에서 쓸 수 있는 것(아래 _NAMESPACE):
    NAME = "DOREMI"          # 미션 이름(12자, 표시용). 없으면 파일 이름을 쓴다.
    wait(3.0)                # 3초 쉼 (커서만 진행)
    play("도", 0.25)         # 모든 큐브가 동시에 도 0.25초 → 커서도 0.25초 진행
    play("미", 0.25, nodes=[2])   # 특정 큐브만
    chord({1: "도", 2: "미", 3: "솔"}, 1.0)   # 큐브마다 다른 음을 같은 시각에(화음)
    at(5.0)                  # 커서를 절대시각(초)으로 이동
    angle(90, nodes=[2])     # 관절각 키프레임(KIND=0) — 모터가 붙어 있을 때
    NODES                    # 이 유닛의 노드ID 목록(예: [1, 2, 3])

음이름은 계이름(도레미파솔라시, 옥타브 숫자 선택)과 음명(C4·A3…) 둘 다 받는다.
주파수 표는 큐브 펌웨어의 piano_scale과 같은 값이다(docs/큐브_멜로디_데이터.xlsx).

단독 실행:
    python -m rcube.mission missions/melody_do_re_mi.py            # 타임라인만 출력
    python -m rcube.mission missions/melody_do_re_mi.py -o out.rcm # 컴파일해 저장
    python -m rcube.mission missions/melody_do_re_mi.py --nodes 3  # 큐브 수 지정
기본 큐브 수·통신방식은 app/netmap.json(마지막 네트워크 설정)에서 읽는다.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from .protocol import (
    MAX_NODES,
    MISSION_FLAG_REPEAT,
    MISSION_KIND_ANGLE,
    MISSION_KIND_TONE,
    MISSION_REC_LEN,
    build_member_map,
    build_mission_container,
    build_mission_record,
    build_mission_tone_record,
    mission_unit_sig,
    parse_mission_container,
)

NETMAP_PATH = Path(__file__).resolve().parents[1] / "netmap.json"

# ---- 음이름 → 주파수(Hz) ------------------------------------------------
# 펌웨어 rcube_melody.c의 piano_scale[17] 그대로. A3(라)~C6(도).
PIANO_SCALE = (220, 247, 262, 294, 330, 349, 392, 440, 494,
               523, 587, 659, 698, 784, 880, 988, 1046)
_SCALE_NAMES = ("A3", "B3", "C4", "D4", "E4", "F4", "G4",
                "A4", "B4", "C5", "D5", "E5", "F5", "G5", "A5", "B5", "C6")
_SOLFA = {"C": "도", "D": "레", "E": "미", "F": "파", "G": "솔", "A": "라", "B": "시"}

NOTE_HZ: "dict[str, int]" = {}
for _i, _n in enumerate(_SCALE_NAMES):
    _hz = PIANO_SCALE[_i]
    NOTE_HZ[_n] = _hz                                  # "C4"
    NOTE_HZ[_SOLFA[_n[0]] + _n[1]] = _hz               # "도4"
# 옥타브를 안 쓴 이름은 기본 옥타브(도=C4 … 시=B4)로 본다.
for _n in ("C4", "D4", "E4", "F4", "G4", "A4", "B4"):
    NOTE_HZ[_n[0]] = NOTE_HZ[_n]
    NOTE_HZ[_SOLFA[_n[0]]] = NOTE_HZ[_n]

_REST_NAMES = {"쉼", "쉼표", "rest", "R", "-", ""}


def note_hz(note) -> int:
    """음이름/주파수/None → 주파수(Hz). 쉼표는 0."""
    if note is None:
        return 0
    if isinstance(note, (int, float)):
        hz = int(note)
        if hz and not 20 <= hz <= 20000:
            raise ValueError(f"주파수 범위(20~20000Hz)를 벗어났습니다: {note}")
        return hz
    key = str(note).strip()
    if key in _REST_NAMES:
        return 0
    hz = NOTE_HZ.get(key) or NOTE_HZ.get(key.upper())
    if hz is None:
        raise ValueError(f"모르는 음이름입니다: {note!r} "
                         f"(쓸 수 있는 이름: {', '.join(sorted(set(_SCALE_NAMES)))} "
                         f"또는 도·레·미…)")
    return hz


# ---- 미션 빌더(소스가 호출하는 DSL) -------------------------------------
@dataclass
class _Rec:
    t_ms: int
    node: int
    kind: int
    payload: bytes          # 직렬화된 8바이트 레코드
    desc: str               # 사람이 읽는 설명(타임라인 출력용)


@dataclass
class MissionBuilder:
    """미션 소스를 실행하며 키프레임을 모은다. 커서(t)는 초 단위로 흐른다."""

    nodes: "list[int]"
    name: str = ""
    t: float = 0.0
    records: "list[_Rec]" = field(default_factory=list)

    # ---- 시간 ----
    def wait(self, sec: float) -> None:
        """지정 시간만큼 아무것도 하지 않고 커서를 진행한다(쉼)."""
        if sec < 0:
            raise ValueError(f"wait 시간은 음수일 수 없습니다: {sec}")
        self.t += float(sec)

    def at(self, sec: float) -> None:
        """커서를 절대시각(미션 시작 기준 초)으로 옮긴다."""
        if sec < 0:
            raise ValueError(f"at 시각은 음수일 수 없습니다: {sec}")
        self.t = float(sec)

    def now(self) -> float:
        return self.t

    # ---- 동작 ----
    def _targets(self, nodes) -> "list[int]":
        if nodes is None:
            return list(self.nodes)
        if isinstance(nodes, int):
            nodes = [nodes]
        out = [int(n) for n in nodes]
        for n in out:
            if n not in self.nodes:
                raise ValueError(f"노드 {n}은 이 유닛(={self.nodes})에 없습니다")
        return out

    def _t_ms(self) -> int:
        t_ms = int(round(self.t * 1000))
        if t_ms > 0xFFFF:
            raise ValueError(f"미션 한 묶음은 65.535초까지입니다(현재 {self.t:.3f}s). "
                             "더 길면 미션을 나누세요(확장 규격 §2.5.2).")
        return t_ms

    def play(self, note, sec: float, nodes=None, *, advance: bool = True) -> None:
        """지정 큐브(기본: 전체)가 같은 시각에 note를 sec초 동안 낸다.

        기본값이 "모든 큐브 동시"인 이유는, 유닛 전체가 한 미션으로 함께 움직이는 것이
        미션코드의 기본형이기 때문이다(기획서 8장 중앙 제어). advance=False면 커서를
        진행시키지 않아 다른 큐브에 다른 음을 겹쳐 쌓을 수 있다(화음).
        """
        dur_ms = int(round(float(sec) * 1000))
        if dur_ms <= 0:
            raise ValueError(f"음 길이는 0보다 커야 합니다: {sec}")
        hz = note_hz(note)
        t_ms = self._t_ms()
        for n in self._targets(nodes):
            self.records.append(_Rec(
                t_ms, n, MISSION_KIND_TONE,
                build_mission_tone_record(t_ms, n, hz, dur_ms),
                f"{note if note is not None else '쉼'} {hz}Hz {dur_ms}ms"))
        if advance:
            self.t += float(sec)

    def chord(self, notes_by_node: dict, sec: float) -> None:
        """큐브마다 다른 음을 같은 시각에 sec초 동안 낸다(화음).

        예: chord({1: "도", 2: "미", 3: "솔"}, 1.0)
        play(..., advance=False)를 여러 번 쓰는 것과 같지만, "이 큐브는 이 음"이라는
        의도가 한 줄에 드러난다. 커서는 마지막에 한 번만 sec만큼 진행한다.
        """
        if sec <= 0:
            raise ValueError(f"화음 길이는 0보다 커야 합니다: {sec}")
        for node, note in notes_by_node.items():
            self.play(note, sec, nodes=[int(node)], advance=False)
        self.t += float(sec)

    def angle(self, deg: float, nodes=None) -> None:
        """관절각 키프레임(KIND=0). 모터가 붙어 있을 때만 의미가 있다.

        커서는 진행시키지 않는다 — 각도 키프레임의 t_ms는 "도달 시각"이라, 다음 목표를
        언제 줄지는 wait()으로 직접 정하는 편이 오해가 없다.
        """
        t_ms = self._t_ms()
        for n in self._targets(nodes):
            self.records.append(_Rec(
                t_ms, n, MISSION_KIND_ANGLE,
                build_mission_record(t_ms, n, deg), f"{deg:.2f}°"))

    # ---- 직렬화 ----
    def body(self) -> bytes:
        """키프레임을 (시각, 노드) 순으로 정렬해 본문 바이트로 만든다.

        정렬이 규약이다: 큐브 시퀀서는 같은 t_ms의 레코드를 한 묶음으로 처리해 원격
        멤버부터 보내므로, 시각 순으로 늘어서 있어야 한 번의 훑기로 끝난다.
        """
        recs = sorted(self.records, key=lambda r: (r.t_ms, r.node))
        return b"".join(r.payload for r in recs)

    def timeline(self) -> "list[str]":
        lines = []
        for r in sorted(self.records, key=lambda r: (r.t_ms, r.node)):
            kind = "톤 " if r.kind == MISSION_KIND_TONE else "각도"
            lines.append(f"  {r.t_ms / 1000:7.3f}s  노드{r.node}  {kind}  {r.desc}")
        return lines

    @property
    def duration(self) -> float:
        return self.t


@dataclass
class CompiledMission:
    """컴파일 결과 — 파일로 저장하거나 그대로 업로드할 수 있다."""

    name: str
    nodes: "list[int]"
    body: bytes
    container: bytes
    unit_sig: int
    timeline: "list[str]"
    duration: float

    @property
    def record_count(self) -> int:
        return len(self.body) // MISSION_REC_LEN

    def summary(self) -> str:
        return (f"{self.name}: {self.record_count}개 키프레임 / {len(self.body)}B "
                f"/ {self.duration:.2f}초 / 노드 {self.nodes} "
                f"/ unit_sig=0x{self.unit_sig:08X}")


def load_netmap(path: Path | None = None):
    """app/netmap.json(마지막 네트워크 설정)에서 (노드수, {노드ID: CMF}, 종단ID)를 읽는다.

    없거나 깨졌으면 None. 미션의 노드 목록·유닛 서명 기본값의 근거가 된다.
    """
    src = Path(path) if path else NETMAP_PATH
    try:
        data = json.loads(src.read_text(encoding="utf-8"))
        n = int(data["n"])
        cmf = {int(k): int(v) for k, v in data.get("cmf", {}).items()}
        return n, cmf, int(data.get("term", 0))
    except Exception:
        return None


def compile_source(source: str, *, nodes: "list[int]", name: str = "",
                   unit_sig: int = 0, repeat: bool = False,
                   filename: str = "<mission>") -> CompiledMission:
    """미션 소스(파이썬)를 실행해 데이터 테이블로 컴파일한다.

    ※ 샌드박스가 아니다 — 사용자가 직접 쓴 미션 코드를 PC에서 그대로 실행한다.
      큐브에 올라가는 것은 실행 결과(키프레임 표)뿐이고 소스는 올라가지 않는다.
    """
    if not nodes:
        raise ValueError("노드 목록이 비었습니다 — 유닛 구성(netmap)을 먼저 정하세요")
    b = MissionBuilder(nodes=list(nodes), name=name)
    ns = {
        "NODES": list(nodes),
        "NAME": name,
        "wait": b.wait, "rest": b.wait, "at": b.at, "now": b.now,
        "play": b.play, "tone": b.play, "chord": b.chord, "angle": b.angle,
        "note_hz": note_hz, "NOTE_HZ": NOTE_HZ,
    }
    exec(compile(source, filename, "exec"), ns)   # noqa: S102 — 사용자 미션 코드

    final_name = (str(ns.get("NAME") or name or "MISSION"))[:12]
    body = b.body()
    if not body:
        raise ValueError("키프레임이 하나도 없습니다 — play()/angle()을 호출했는지 확인하세요")
    container = build_mission_container(body, name=final_name, unit_sig=unit_sig,
                                        n_nodes=len(nodes), repeat=repeat)
    return CompiledMission(name=final_name, nodes=list(nodes), body=body,
                           container=container, unit_sig=unit_sig,
                           timeline=b.timeline(), duration=b.duration)


def decode_body(body: bytes) -> "tuple[list[str], float]":
    """본문(키프레임 배열)을 사람이 읽는 타임라인과 총 길이(초)로 푼다.

    이미 컴파일된 .rcm을 눈으로 확인할 때 쓴다 — 소스가 없어도 무엇이 언제 울리는지
    보여야 잘못된 파일을 올리는 것을 막을 수 있다.
    """
    lines, end = [], 0.0
    for off in range(0, len(body) - MISSION_REC_LEN + 1, MISSION_REC_LEN):
        r = body[off:off + MISSION_REC_LEN]
        t_ms = int.from_bytes(r[0:2], "little")
        node, kind = r[2], r[3]
        raw = int.from_bytes(r[4:8], "little")
        if kind == MISSION_KIND_TONE:
            hz, dur = (raw >> 16) & 0xFFFF, raw & 0xFFFF
            lines.append(f"  {t_ms / 1000:7.3f}s  노드{node}  톤    {hz}Hz {dur}ms")
            end = max(end, (t_ms + dur) / 1000)
        else:
            deg = int.from_bytes(r[4:8], "little", signed=True) / 100.0
            lines.append(f"  {t_ms / 1000:7.3f}s  노드{node}  각도  {deg:.2f}°")
            end = max(end, t_ms / 1000)
    return lines, end


def compile_file(path, *, nodes: "list[int]" = None, cmf: dict = None,
                 unit_sig: int = None, repeat: bool = False) -> CompiledMission:
    """미션 소스 파일(.py)을 컴파일한다. 이미 컴파일된 .rcm이면 그대로 싣는다.

    nodes/cmf를 주지 않으면 netmap.json(마지막 네트워크 설정)을 쓴다. unit_sig도 그
    구성에서 계산한다 — 다른 구성의 큐브에 올리면 실행이 거부되도록 하는 안전핀이다.

    ★ .rcm을 읽을 때도 unit_sig는 "지금 구성"으로 다시 찍는다. 파일에 박힌 서명은
      그 파일을 만들던 때의 구성이라, 구성이 바뀐 뒤 그대로 올리면 큐브가 실행을
      거부한다. 서명은 업로드(F0)에 따로 실려 큐브가 헤더에 쓰므로 재계산이 가능하다.
    """
    p = Path(path)
    if nodes is None or cmf is None:
        nm = load_netmap()
        if nm is not None:
            n, map_cmf, _term = nm
            nodes = nodes or sorted(map_cmf) or list(range(1, n + 1))
            cmf = cmf or map_cmf
    if unit_sig is None:
        unit_sig = mission_unit_sig(len(nodes), build_member_map(cmf)) if (nodes and cmf) else 0

    if p.suffix.lower() == ".rcm":
        info = parse_mission_container(p.read_bytes())
        node_list = list(nodes) if nodes else list(range(1, (info["n_nodes"] or 1) + 1))
        timeline, duration = decode_body(info["body"])
        container = build_mission_container(
            info["body"], name=info["name"], unit_sig=unit_sig, n_nodes=len(node_list),
            repeat=repeat or bool(info["flags"] & MISSION_FLAG_REPEAT))
        return CompiledMission(name=info["name"], nodes=node_list, body=info["body"],
                               container=container, unit_sig=unit_sig,
                               timeline=timeline, duration=duration)

    if not nodes:
        raise ValueError("netmap.json이 없습니다 — 노드 목록을 직접 지정하세요(--nodes)")
    return compile_source(p.read_text(encoding="utf-8"), nodes=list(nodes),
                          name=p.stem[:12], unit_sig=unit_sig, repeat=repeat,
                          filename=str(p))


# ---- 단독 실행 ---------------------------------------------------------
def _main(argv: "list[str]") -> int:
    import argparse

    ap = argparse.ArgumentParser(description="파이썬 미션 → R큐브 미션코드(.rcm) 컴파일")
    ap.add_argument("source", help="미션 소스(.py) 또는 이미 컴파일된 .rcm")
    ap.add_argument("-o", "--out", help="저장할 .rcm 경로(없으면 출력만)")
    ap.add_argument("--nodes", type=int, default=0,
                    help="유닛 큐브 수(노드ID 1..N). 없으면 netmap.json")
    ap.add_argument("--cmf", choices=("can", "ble"),
                    help="모든 큐브의 통신방식을 이 값으로 두고 유닛 서명을 계산한다"
                         "(예: 3대 모두 CAN). 기본은 netmap.json의 구성")
    ap.add_argument("--repeat", action="store_true", help="반복 재생 플래그")
    ap.add_argument("--no-sig", action="store_true",
                    help="유닛 서명을 0으로(어떤 구성에서도 실행 — 시험용)")
    a = ap.parse_args(argv)

    nodes = list(range(1, a.nodes + 1)) if a.nodes else None
    cmf = None
    if a.cmf:
        nm = load_netmap()
        nodes = nodes or (sorted(nm[1]) if nm else None)
        if not nodes:
            print("--cmf를 쓰려면 --nodes로 큐브 수를 함께 지정하세요.")
            return 1
        cmf = {n: (1 if a.cmf == "can" else 0) for n in nodes}
    elif nodes is not None:
        nm = load_netmap()
        cmf = nm[1] if nm and len(nm[1]) == len(nodes) else None
    try:
        m = compile_file(a.source, nodes=nodes, cmf=cmf,
                         unit_sig=0 if a.no_sig else None, repeat=a.repeat)
    except Exception as e:
        print(f"컴파일 실패: {e}")
        return 1

    print(m.summary())
    if m.unit_sig == 0:
        print("  ※ unit_sig=0 — 유닛 구성 검사를 하지 않는다(어느 유닛에서도 실행됨).")
    for line in m.timeline:
        print(line)
    if a.out:
        Path(a.out).write_bytes(m.container)
        print(f"저장: {a.out} ({len(m.container)}B = 헤더32 + 본문{len(m.body)})")
    return 0


if __name__ == "__main__":
    import sys
    raise SystemExit(_main(sys.argv[1:]))
