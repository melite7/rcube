"""
melody.py — 큐브 멜로디를 PC 스피커로 재생한다. ★임시 테스트용

docs/큐브_멜로디_데이터.xlsx의 "멜로디 데이터" 시트를 그대로 읽어, 곡별 음 목록
[(주파수Hz, 길이초), …]으로 만든다. 큐브에 아무것도 보내지 않는다 — 어떤 멜로디인지
귀로 확인하려는 용도다.

시트 레이아웃(2026-06-13판):
    [멜로디][순번][NoteIdx][음이름][계이름][주파수(Hz)][Duration(초)][Amplitude]
    곡 사이에 "■ 곡명 (트리거상수)" 구분행이 들어간다(순번이 비어 있어 걸러진다).

재생 방식: 곡 전체를 16비트 PCM 사각파로 렌더링해 winsound.PlaySound(SND_MEMORY)로
한 번에 낸다.
  · winsound.Beep은 쓰지 않는다 — 호출마다 수십 ms의 오버헤드가 있어 LINK(음당
    0.05~0.08초) 같은 짧은 곡이 거의 들리지 않고, 음 사이가 벌어져 리듬이 무너진다.
  · 큐브 부저는 LEDC PWM 50% 듀티 = 사각파다. 사인파 대신 사각파로 만들어 음색을
    맞췄다. 음 경계에는 2ms 페이드를 넣어 딸깍거림(클릭)을 없앤다.
"""
from __future__ import annotations

import io
import struct
import threading
import time
import wave
from pathlib import Path

XLSX_PATH = Path(__file__).resolve().parent.parent / "docs" / "큐브_멜로디_데이터.xlsx"
SHEET_NAME = "멜로디 데이터"

SAMPLE_RATE = 44100
AMPLITUDE = 0.35        # 0~1 (풀스케일 대비). 사각파는 체감 음량이 커서 낮게 잡는다
FADE_SEC = 0.002        # 음 앞뒤 페이드 — 사각파 경계의 클릭 제거
MIN_HZ = 20             # 이보다 낮으면 쉼표로 본다


def load_melodies(path: Path | None = None) -> "dict[str, list[tuple[int, float]]]":
    """xlsx에서 {곡명: [(주파수Hz, 길이초), …]}를 읽는다. 시트 순서를 유지한다."""
    import openpyxl   # 무거워서 필요할 때만 부른다

    src = Path(path) if path else XLSX_PATH
    wb = openpyxl.load_workbook(src, data_only=True, read_only=True)
    ws = wb[SHEET_NAME]

    out: "dict[str, list[tuple[int, float]]]" = {}
    for row in ws.iter_rows(min_row=2, values_only=True):
        if not row or not row[0]:
            continue
        name, seq, freq, dur = row[0], row[1], row[5], row[6]
        if seq in (None, ""):
            continue          # "■ …" 구분행
        try:
            hz = int(float(freq))
            sec = float(dur)
        except (TypeError, ValueError):
            continue          # 값이 비었거나 숫자가 아님 → 그 음은 건너뛴다
        out.setdefault(str(name).strip(), []).append((hz, sec))
    wb.close()
    return out


def render_wav(notes: "list[tuple[int, float]]",
               sample_rate: int = SAMPLE_RATE, amplitude: float = AMPLITUDE) -> bytes:
    """음 목록을 16비트 모노 WAV 바이트로 만든다(사각파, 음 경계 2ms 페이드)."""
    fade = max(1, int(FADE_SEC * sample_rate))
    frames = bytearray()
    for hz, sec in notes:
        count = max(1, int(sec * sample_rate))
        if hz < MIN_HZ:
            frames += b"\x00\x00" * count            # 쉼표
            continue
        period = sample_rate / hz
        peak = int(amplitude * 32767)
        for i in range(count):
            v = peak if (i % period) < (period / 2.0) else -peak
            if i < fade:                              # 앞 페이드
                v = int(v * i / fade)
            elif i >= count - fade:                   # 뒤 페이드
                v = int(v * (count - i) / fade)
            frames += struct.pack("<h", v)
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sample_rate)
        w.writeframes(bytes(frames))
    return buf.getvalue()


class MelodyPlayer:
    """한 번에 한 곡만 재생한다. 재생 중 다른 곡을 요청하면 앞의 것을 끊는다."""

    def __init__(self, on_log=None) -> None:
        self._on_log = on_log
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._wav: bytes = b""   # 재생 중인 버퍼(SND_ASYNC 동안 살아 있어야 한다)

    @property
    def is_playing(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _log(self, msg: str) -> None:
        if self._on_log:
            self._on_log(msg)

    def play(self, name: str, notes: "list[tuple[int, float]]") -> None:
        self.stop()
        total = sum(d for _, d in notes)
        try:
            import winsound
        except ImportError:
            self._log("[멜로디] winsound를 쓸 수 없습니다(Windows 전용).")
            return
        self._wav = render_wav(notes)
        self._log(f"[멜로디] ▶ {name} — {len(notes)}음 / {total:.2f}초")
        # winsound는 SND_MEMORY와 SND_ASYNC를 함께 쓰지 못한다("Cannot play
        # asynchronously from memory"). 그래서 워커 스레드에서 동기로 재생한다 —
        # UI 스레드는 막히지 않고, 중단은 UI 쪽에서 SND_PURGE로 건다.
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, args=(name, total), daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """재생을 즉시 중단한다."""
        try:
            import winsound
            winsound.PlaySound(None, winsound.SND_PURGE)
        except (ImportError, RuntimeError):
            pass
        if self.is_playing:
            self._stop.set()
            self._thread.join(timeout=1.0)

    def _run(self, name: str, total: float) -> None:
        import winsound
        try:
            winsound.PlaySound(self._wav, winsound.SND_MEMORY)   # 끝날 때까지 블록
        except RuntimeError as e:
            self._log(f"[멜로디] 재생 실패: {e}")
            return
        self._log(f"[멜로디] ■ {name} 중지" if self._stop.is_set()
                  else f"[멜로디] ◼ {name} 재생 완료")


# ---- 단독 실행(앱 없이 들어보기) --------------------------------------
#   python melody.py            → 곡 목록
#   python melody.py 3          → 3번 곡 재생
#   python melody.py START      → 이름에 START가 들어가는 곡 재생
#   python melody.py all        → 전곡을 차례로 재생
def _main(argv: "list[str]") -> int:
    try:
        mels = load_melodies()
    except Exception as e:
        print(f"멜로디를 읽지 못했습니다: {e!r}")
        print(f"  파일: {XLSX_PATH}")
        print("  openpyxl이 필요합니다 — app\\.venv\\Scripts\\python.exe 로 실행하세요.")
        return 1

    names = list(mels)
    if not argv:
        print(f"{XLSX_PATH.name} — {len(names)}곡\n")
        for i, n in enumerate(names, 1):
            notes = mels[n]
            print(f"  {i:2d}. {n:34s} {len(notes):3d}음 {sum(d for _, d in notes):5.2f}초")
        print("\n재생: python melody.py <번호|이름 일부|all>")
        return 0

    arg = argv[0]
    if arg == "all":
        targets = names
    elif arg.isdigit() and 1 <= int(arg) <= len(names):
        targets = [names[int(arg) - 1]]
    else:
        targets = [n for n in names if arg.lower() in n.lower()]
        if not targets:
            print(f"'{arg}'에 해당하는 곡이 없습니다. 목록은 인자 없이 실행하세요.")
            return 1

    player = MelodyPlayer(on_log=print)
    for n in targets:
        player.play(n, mels[n])
        while player.is_playing:      # 단독 실행에서는 끝날 때까지 기다린다
            time.sleep(0.05)
        time.sleep(0.3)
    return 0


if __name__ == "__main__":
    import sys
    raise SystemExit(_main(sys.argv[1:]))
