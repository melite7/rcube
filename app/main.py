"""
R큐브 PC 제어 프로그램 — 진입점 (Phase 0 스켈레톤)

이 파일은 "환경이 도는지" 확인용 최소 골격이다. 실제 BLE/CAN 제어는
Phase 4부터 채운다(shared-protocol 정의 사용).

실행:
    python main.py            # 환경 확인
    python main.py --scan     # (예정) BLE 스캔
"""
import argparse
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description="R-Cube PC control tool")
    parser.add_argument("--scan", action="store_true",
                        help="(예정) BLE로 R큐브 스캔")
    args = parser.parse_args()

    print("==== R-Cube PC tool (Phase 0 skeleton) ====")
    print(f"python: {sys.version.split()[0]}")

    try:
        import bleak  # noqa: F401
        print("bleak: OK")
    except ImportError:
        print("bleak: 미설치 — 'pip install -r requirements.txt' 필요")

    try:
        import can  # noqa: F401
        print("python-can: OK")
    except ImportError:
        print("python-can: 미설치 — 'pip install -r requirements.txt' 필요")

    if args.scan:
        print("TODO(Phase4): BLE 스캔은 아직 구현 전입니다.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
