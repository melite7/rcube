# R큐브 코드 개발 로드맵 (전체 시스템)

*기준 문서: R큐브_기획서_20260709 / R큐브_프로토콜_BLE_CAN_20260703 · 작성일 2026-07-21*

> 참고: 모터제어보드(STM32G4) 펌웨어는 외부 업체가 개발한다. 본 저장소는 그 UART 프로토콜로 위치/속도를 제어하고 상태를 수신하는 ESP32 쪽 어댑터만 구현한다. (기존 "트랙 B(STM32G4 서보 펌웨어)"는 개발 범위에서 제외되며, 대신 벤더 UART 프로토콜 확보가 최우선 외부 의존성이 된다.)

---

## 0. 개발 철학 3원칙

이 시스템은 "역할이 많아 보이지만 실제로는 한 개의 펌웨어가 부팅 분기로 모든 역할을 수행"하는 구조입니다(12장). 그래서 개발 순서도 **역할별로 나누지 말고, 한 큐브를 세로로 관통시킨 뒤 대수를 늘리는** 방식이 맞습니다.

1. **수직 슬라이스 우선(vertical slice).** "PC → BLE → ESP32 → UART → (벤더)모터보드 → 모터 1회전"이 한 줄로 끝까지 도는 최소 경로를 가장 먼저 완성합니다. 이게 되면 나머지는 이 파이프에 기능을 얹는 일입니다.
2. **전송계층 무관 설계(transport-agnostic).** 응용계층(OpCode·필드·ACK)은 BLE/CAN이 완전히 공유합니다(CAN 시트 A절). 그러니 **명령 파서·상태머신·모션 큐를 먼저 만들고, BLE/CAN은 그 아래 얇은 송수신 어댑터로** 붙입니다. 통신방식별 코드 분기를 최소화하는 게 핵심(12.9).
3. **역할은 부팅 분기, 스택은 플래그로.** ECF/CMF/멤버맵을 읽어 스택을 선택하는 부팅 시퀀스(12.6)를 골격으로 먼저 세우면, 멤버·edge central·BLE 허브·설정모드가 같은 뼈대 위 레이어로 붙습니다.

**두 개의 트랙 + 하나의 외부 의존성**
- **트랙 A (ESP32-S3 펌웨어)** — 임계경로. 전체 일정의 중심.
- **트랙 C (PC 제어 프로그램)** — 트랙 A가 BLE 최소기능을 여는 순간부터 따라붙어 서로를 검증하는 페어로 진행.
- **외부 의존성 (벤더 모터보드 UART 프로토콜)** — 내가 개발하지 않고 업체로부터 스펙을 받아 고정. 늦어지면 Phase 1이 막히므로, 세팅과 병행해 지금 확보에 착수. 실보드가 없어도 개발이 막히지 않도록 `tools/motor-sim`(UART 시뮬레이터)을 초반에 만든다.

가장 위험한 부분: **① ESP32 듀얼코어 격리(Core0 통신 폭주가 Core1 모션 주기를 밀지 않는가, 12.7), ② BLE 다중연결 시 지터/부하(edge central이 BLE 멤버 직접 다중연결, 11장), ③ CAN 자가종단 배선 규칙(13장).** 각 해당 단계에서 "실측 검증"을 마일스톤에 넣습니다.

---

## 1. 단계별 개발 순서

각 단계는 **끝나는 조건(Exit)** 을 명확히 두고, 그게 통과돼야 다음으로 갑니다.

### Phase 0 — 기반 정리 (선행)

코드를 짜기 전에 "말이 통하는 규칙"과 개발 토대부터 못 박습니다.

- **저장소·Git·GitHub 세팅.** 모노레포 구조(esp32 / app / shared-protocol / vendor / tools / docs), .gitignore, README, git init, GitHub 연동. *(현재 진행 중)*
- **개발환경(본인 PC).** ESP-IDF(ESP32-S3, hello-world 빌드/플래시 확인), PC측 Python venv(BLE·USB-CAN 라이브러리). 각각 "빈 프로젝트가 실제로 동작"까지가 검증 조건.
- **프로토콜을 코드로 고정.** xlsx OpCode 표(59 + CmdAck)를 단일 헤더/스키마로 전환. `opcodes.h`, `resultcodes.h`(NACK 코드표 F절), 패킷 구조체, TargetId 규약. **esp32·app이 공유하는 단일 소스.**
- **★벤더 UART 프로토콜 확보.** 모터보드 업체로부터 UART 프로토콜 스펙 수령 → `vendor/motor-board`에 고정. 키프레임(목표위치+도달시간/속도) 송신, 모터상태(추종오차·전류·플래그·폴트·온도) 수신, 하트비트/무신호 자가정지 규약(11장/12.9).
- **Exit:** 빈 ESP-IDF 바이너리 부팅 로그 + PC가 공유 프로토콜 헤더로 패킷 인코딩/디코딩 왕복 테스트 통과 + 벤더 UART 스펙 문서 확보.

### Phase 1 — 단일 큐브 브링업 (하드웨어 관통)

역할·통신 다 빼고, **한 대의 큐브가 물리적으로 살아있음**을 증명합니다.

- 부팅 후 플래시(NVS) 읽기/쓰기: ECF/CMF/노드ID/그룹번호(출하 기본값 0,0,0,0).
- 주변장치 드라이버: SK6812 칼라LED(E0, 색상표 I-3), 부저(E6/E3), 버튼(전원/연결/설정모드 3초 롱프레스, 5장), IMU(SPI), 외부확장포트 자동탐지(I2C→UART→Analog, 2장).
- **Core 1 모션 태스크 뼈대:** 하드웨어 타이머 고정주기(20~50Hz)로 벤더 UART 프로토콜을 통해 모터보드에 키프레임 송출 + 상태 수신. 실보드가 없으면 `tools/motor-sim`으로 대체.
- **★코어 핀닝 못 박기:** 모션=Core1, (후속)MicroPython=Core0(MP_TASK_COREID=0), 통신=Core0. 기본값 충돌 주의(12.2).
- **Exit:** 전원 ON → 멜로디 → LED 점등 → 버튼 상태 전환 → UART 명령 한 줄로 모터 1회전 + 모터상태 회신 수신(실보드 또는 시뮬레이터).

### Phase 2 — 프로토콜 코어 + 상태머신 (전송계층 무관)

통신 스택을 붙이기 **전에**, 통신과 무관한 "명령 처리 두뇌"를 만듭니다.

- **명령 파서/디스패처:** 바이트[4...] 페이로드 → OpCode별 핸들러. 입력은 "어디서 왔는지 모르는" 버퍼(12.3 무지 원칙).
- **CmdAck(0xAF) 통합 응답 엔진** + NACK ResultCode 전 코드 처리(APPENDIX F).
- **명시적 상태머신(12.7):** Boot→ConnWait→ConnDone→Armed→Run→Stop + Config(별도). armed/실행 중 설정·업로드 거부. GetNodeState(B3) 노출.
- **모션 최소셋:** SetSingleSpeed(C0)/SetSingleAngle(C1)/MoveToOrigin(C8)/SetThisToOrigin(C9)/SetDriveState(CB). Immediate부터.
- **센서 조회:** GetSensors(B0, 20B 예외구조)/GetMotorStatus(B2)/GetNodeState(B3).
- **Exit:** 가짜 전송계층(시리얼 콘솔)으로 위 명령 → 모터 동작 + CmdAck·센서 규격대로 회신. **이 코어는 그대로 BLE·CAN 양쪽에 재사용.**

### Phase 3 — 통신 전송계층 (BLE 먼저, CAN 그 다음)

- **BLE peripheral(NimBLE):** 광고 `RCUBEROBOT.GG.NN`, 명령 수신 characteristic / 센서·ACK notification, 광고에 노드ID(APPENDIX I-4).
- **CAN(TWAI):** 29비트 확장 ID(CAN B절), acceptance filter(D절), 분할전송(E절), C5 CAN 변형(F절), D9 NodeAnnounce(G절), TWAI ISR IRAM 배치.
- **송신 추상화:** 상위는 노드ID로만 주소지정, 하위가 노드ID→BLE handle 또는 노드ID→CAN ID 변환(12.4/12.6).
- **Exit:** 동일 명령 시퀀스가 BLE로도 CAN(USB-CAN 직결)으로도 동일하게 동작.

### Phase 4 — PC 제어 프로그램 최소기능

- BLE 스캔·연결, 명령 송신, 센서 스트림 수신·표시(SetSensorStream B1), USB-CAN 직접 제어.
- **Exit:** PC에서 단일 큐브를 BLE/CAN 각각으로 붙여 실시간 제어. 이후 모든 단계는 이 PC 툴로 검증.

### Phase 5 — 비고정형 BLE 구성 + BLE 허브(Aggregator)

- 비고정형: 노드ID 미할당 → 흰색 점멸 → 연결순서로 가상 노드ID(1..N) 배정·점등(첫=가상1=Red).
- **BLE 허브 add-on:** NimBLE multirole. A0/A2/A3. PC↔허브(peripheral) + 허브↔멤버(central) 취합·분배(12.4, APPENDIX I).
- **Exit:** 3~5대를 켜는 순서로 붙여 가상 노드ID/색 배정 + 허브 경유 일괄 제어.

### Phase 6 — 통신방식 세팅 + 고정형 전환

- **SetNodeConfig(D3):** 노드ID+CMF+종단노드ID 플래시 저장 후 재부팅. D4/DA read-back.
- PC 매핑테이블(노드ID↔통신방식) — 멤버맵 원본.
- 고정형 재연결: 광고에 저장 노드ID 실어 켜는 순서 무관 노드ID 순 자동 연결. ResetConfig(D7) 초기화.
- **Exit:** 세팅→저장→reset→고정형 자동 재연결·점등.

### Phase 7 — CAN 서브로봇유닛 + 자가종단 (★리스크 검증)

- **부팅 시 자가종단(13.4):** CMF=1 & 노드ID==종단노드ID → ON. CAN 통신 이전 결정.
- **CAN 배선 규칙(13장):** CAN 큐브만 노드ID 오름차순 데이지체인, 최대 CAN 노드ID를 바깥 끝, PC USB-CAN 어댑터가 반대쪽 끝.
- 혼합 서브: PC가 CAN(직접) + BLE(허브 경유) 동시 제어(3장).
- **Exit:** 혼합 R3(노드01·02 CAN, 노드03 BLE) 서브 동작. **종단 2개만 켜지는지 물리 실측.**

### Phase 8 — edge central / 독립로봇유닛 (핵심 전환) (★리스크 검증)

- **SetEdgeCentralConfig(D5):** ECF=1 + N + 멤버맵{노드ID,CMF} 저장, D6 정합.
- **부팅 서버 선택(12.6):** 멤버맵 보고 TWAI만/NimBLE central만/둘 다. BLE 멤버 허브 없이 직접 다중연결, 노드ID→conn_handle 바인딩(12.4).
- **미션 데이터테이블 엔진(C):** 시간축 스케줄을 각 노드(자신 포함)에 분배. 파이썬 없이 항상 동작. GetUnitRoster(B4) 진단.
- **Exit:** 서브→D5 독립 전환 → 배선 정리 → 혼합 멤버 노드ID 순 연결 → 데이터테이블 미션 자율 수행. **BLE 멤버 4~5대 직접연결 시 Core0 워스트케이스 실측(12.7).**

### Phase 9 — 모션 고도화 (버퍼·T0 동기·키프레임 스트림)

- **버퍼 실행 모델:** Mode{Immediate/Buffered} + ExecuteBuffer(C7, Run/Cancel/Flush + StartTime=T0).
- **TimeSync(D2):** 공통시각 → 전 축 T0 일제 출발.
- **SetKeyframeStream(C5, PVT):** 실시간 스트리밍, CAN 변형, BUFFER_FULL/UNDERRUN 크레딧.
- **look-ahead 버퍼:** BLE 멤버 2~3키프레임 선큐잉, connection interval ≤25ms. C2/C3, C4(허브 집계).
- **Exit:** 2축+ T0 동기 출발 협조동작. CAN 10~20ms 고속협조 / BLE look-ahead 안정성 실측.

### Phase 10 — 태스크공간 IK/FK (산업용)

- SetKinematicModel(EA)/SetWorkOrigin(EB)/MoveToPosition(EC, IK→C5 Buffered→C7 T0)/SetScheduledPositions(ED)/GetPosition(B6, FK).
- 3단 안전: 각도 소프트리밋(D8) + IK 해 배제 + 폴트임계(CE). 산업용 모드: CA/CC/CF/CD/CE/DB.
- **Exit:** SCARA(R4)에 "TCP를 (x,y)로" → IK 분해 → 동기 실행. 도달불가 IK_FAIL(0x10).

### Phase 11 — 미션코드 MicroPython 임베딩

- MicroPython embed 포트 컴포넌트화, 큐브 API frozen module(목표 적재만, 12.5).
- 업로드: MissionUploadBegin/Chunk/Commit(F0~F2), CRC/버전 검증, LittleFS 원자적 커밋(12.8). .mpy 버전 고정 + mpy-cross 동반.
- **Exit:** PC에서 .mpy 업로드 → 독립 전환 → 파이썬 미션 실행.

### Phase 12 — 안전·설정모드·OTA·보안

- **Fail-safe(10장):** 충격/모터오류 → 정지 전파, 멤버 무신호 자가정지(D1/D9 watchdog), EmergencyStop(D0), ESTOP_ACTIVE(0x0E).
- **설정모드(5장/12.6):** 버튼 3초 롱프레스 → advertising-only(RCUBECONFIG.GG.NN), 모터 중립, 종단 OFF. ECF 해제/초기화/멤버맵 재설정.
- **OTA(F5~F8):** 2-OTA 파티션, 롤백/안티롤백. **보안(부록 F):** PEM 서명 OTA + 챌린지-응답 3계층.
- **Exit:** 임의 큐브 폴트 → 전 유닛 안전정지. 설정모드 강등/복구. 서명검증 OTA.

### Phase 13 — 통합 검증 · 부하 · 신뢰성

- 코어 격리(Core1 jitter), CAN 무중단(OTA 중 TWAI ISR), watchdog 3중 정합, R5 실부하(8대/5대), 다중 서브유닛 라인 E-stop(부록 D).

---

## 2. 의존 관계 한눈에

```
트랙 A (ESP32) : P0 → P1 → P2 → P3 → P5 → P6 → P7 → P8 → P9 → P10 → P11 → P12 → P13
                          └(핵심 코어)      └(멀티큐브)   └(독립)        (고도화)   (안전/양산)
트랙 C (PC)     : P0(프로토콜) → P4부터 A와 페어 → P5·P6 세팅화면 → P7 USB-CAN → P11 업로드툴
외부 의존성     : 벤더 UART 프로토콜 확보(P0) → motor-sim 제작 → P1 합류 / 실보드 입고 시 교체
```

핵심 임계경로: **P0→P1→P2→P3**(단일 큐브 완전 관통)이 전체의 절반. 여기가 탄탄하면 P5 이후는 "같은 코어에 레이어 얹기"라 속도가 붙습니다.

## 3. 실무 권고 요약

- **P2를 서두르지 말 것.** 파서·상태머신·CmdAck·모션 큐는 모든 역할이 공유하는 뼈대. 대충 하면 P8·P9에서 전면 재작업.
- **BLE를 CAN보다 먼저.** 출하 기본 BLE·비고정형(7.1)이 표준 진입점이라 PC 검증 루프가 빨리 돎. CAN은 P3에서 같은 코어에 얹기.
- **리스크 3종(코어격리/BLE다중연결/CAN자가종단)은 해당 단계 Exit에 "실측"으로.**
- **공유 프로토콜 헤더는 단일 소스.** 표(59+1)를 손으로 옮기지 말고 스크립트로 생성.
- **벤더 UART 스펙은 지금 확보 착수.** 늦으면 P1이 막힘. motor-sim으로 실보드 의존을 끊어 병렬 진행.
- **설정모드/OTA/보안(P12)은 뒤에 두되 게이팅(armed 중 재설정 거부)은 P2 상태머신에 미리 반영.**
