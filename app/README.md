# app — PC 제어 프로그램

BLE / USB-CAN 으로 R큐브 유닛을 제어·모니터링하는 PC측 프로그램 (Python).

- 비고정형 BLE 구성, 통신방식 세팅(고정형 전환), 멤버맵/독립전환
- 센서 스트림 표시, 미션(.mpy/데이터테이블) 업로드
- `shared-protocol` 정의를 그대로 사용

## 요구 환경

- Python **3.10+**

## 환경 설정 (venv)

**Windows (PowerShell)**
```powershell
cd app
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python main.py            # 환경 확인 (bleak / python-can OK 확인)
```

**Windows (cmd)**
```
cd app
python -m venv .venv
.venv\Scripts\activate.bat
pip install -r requirements.txt
python main.py
```

## 현재 상태 (Phase 0)

`main.py` 는 Python·bleak·python-can 설치 여부만 확인하는 **스켈레톤**이다.
BLE 스캔·제어(Phase 4)부터 실제 기능이 붙는다.

## 의존성

- `bleak` : BLE Central (스캔/연결/특성 R/W/notify)
- `python-can` : USB-CAN 어댑터 제어 (Phase 7)
- `pyserial`, `rich` : 시리얼·CLI 표시
