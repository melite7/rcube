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

## 현재 상태 (Phase 4 착수 — BLE 제어 UI)

```
python main.py            # BLE 제어 GUI 열기 (기본)
python main.py --scan     # GUI 없이 콘솔에서 R큐브 스캔만
python main.py --check    # bleak/python-can/tkinter 설치 확인
```

**GUI 사용법**
1. R큐브 전원 ON → **BOOT 버튼**을 눌러 `RCUBE00.00` 광고 시작(LED 청록).
2. GUI에서 **[스캔]** → 목록에서 기기 선택 → **[연결]** (LED 초록).
3. **명령 전송**: TargetId(hex) + OpCode(드롭다운) + payload(hex) → **[전송]**.
   또는 **빠른 명령** 버튼(Heartbeat / GetNodeState / LED …).
4. 로그창에 TX(파랑) / RX·notify(초록) 가 실시간 표시된다.

> 참고: 현재 펌웨어의 GATT write 핸들러는 **수신 로깅만** 한다(명령 파싱은
> ESP32 Phase 2/3에서 붙음). 그래서 지금은 `idf.py -p COM3 monitor` 로그에서
> `GATT write N bytes` 를 확인하는 것이 왕복 검증 방법이다. LED/명령별 payload
> 바이트 레이아웃은 xlsx에 아직 코드화 전이라 `gui.py`의 `on_led`는 잠정값이다.

### 코드 구조

| 파일 | 역할 |
|---|---|
| `rcube/protocol.py` | 표준 프레임 조립/파싱 (`build_frame`/`parse_frame`). OpCode/주소는 `shared-protocol`에서 로드 |
| `rcube/ble.py` | BLE Central 전송 어댑터(`RCubeBLE`): 스캔/연결/write/notify (bleak, async) |
| `gui.py` | tkinter UI. async 루프를 백그라운드 스레드에서 돌리고 큐로 UI 마셜링 |
| `main.py` | 진입점(GUI / `--scan` / `--check`) |

펌웨어 계약(`esp32/main/ble_rcube.c`와 일치): 이름 `RCUBE00.00`,
서비스 `52434245-0000-…`, 특성 `52434245-0001-…`(W/R/Notify).

## 의존성

- `bleak` : BLE Central (스캔/연결/특성 R/W/notify)
- `python-can` : USB-CAN 어댑터 제어 (Phase 7)
- `pyserial`, `rich` : 시리얼·CLI 표시
