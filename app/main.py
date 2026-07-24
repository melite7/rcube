"""
R큐브 PC 제어 프로그램 — 진입점.

기본 실행하면 BLE 제어 GUI(gui.py)를 연다.

실행:
    python main.py            # BLE 제어 GUI 열기
    python main.py --scan     # (GUI 없이) 콘솔에서 R큐브 스캔만
    python main.py --check    # 실행환경(bleak 등) 설치 확인
"""
import argparse
import asyncio
import sys


def _check_env() -> int:
    print("==== R-Cube PC tool — 환경 확인 ====")
    print(f"python: {sys.version.split()[0]}")
    for mod, hint in (("bleak", "BLE"), ("can", "USB-CAN"), ("tkinter", "GUI")):
        try:
            __import__(mod)
            print(f"{mod}: OK ({hint})")
        except ImportError:
            print(f"{mod}: 미설치 — 'pip install -r requirements.txt' 필요")
    return 0


def _scan_console() -> int:
    from rcube import RCubeBLE

    async def run():
        print("스캔 중… (약 5초)")
        results = await RCubeBLE.scan(timeout=5.0)
        if not results:
            print("R큐브를 찾지 못했습니다. (기기 BOOT버튼으로 광고 시작했는지 확인)")
            return
        for i, r in enumerate(results):
            print(f"  [{i}] {r}")

    asyncio.run(run())
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="R-Cube PC control tool")
    parser.add_argument("--scan", action="store_true", help="GUI 없이 콘솔에서 BLE 스캔")
    parser.add_argument("--check", action="store_true", help="실행환경(bleak/can/tkinter) 확인")
    args = parser.parse_args()

    if args.check:
        return _check_env()
    if args.scan:
        return _scan_console()

    # 기본: GUI
    from gui import main as gui_main
    return gui_main()


if __name__ == "__main__":
    raise SystemExit(main())
