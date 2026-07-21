# esp32 — R큐브 메인보드 펌웨어 (ESP32-S3)

ESP-IDF(FreeRTOS SMP) 기반 단일 바이너리. 부팅 시 ECF/CMF/멤버맵으로 역할·통신 스택 분기.

- **Core 1**: 실시간 모션(고정주기 키프레임 송출 + 센서 수집)
- **Core 0**: NimBLE/TWAI, 역할 레이어, 데이터테이블 시퀀서, MicroPython VM
- `components/motor_uart/`: 모터제어보드(STM32G4) UART 어댑터 — `vendor/motor-board` 스펙 기준 *(Phase 1 예정)*

## 요구 환경

- ESP-IDF **v5.x** (권장: VS Code + Espressif ESP-IDF 확장)
- 보드: ESP32-S3-WROOM-1-N16R2 (16MB Flash / 2MB PSRAM)

## 빌드 · 플래시 · 모니터

**VS Code (ESP-IDF 확장)**
1. `File > Open Folder` 로 이 `esp32/` 폴더를 연다.
2. 하단 상태바에서 타겟을 **esp32s3**, 포트를 보드가 연결된 COM 포트로 설정.
3. 🔨(Build) → ⚡(Flash) → 🖥(Monitor). 또는 "ESP-IDF: Build, Flash and Monitor".

**CLI (ESP-IDF Command Prompt)**
```
cd esp32
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

## 현재 상태 (Phase 0)

`main/main.c` 는 부팅 로그 + Core 1 핀닝된 모션 태스크 placeholder만 있는 **빌드 검증용 스켈레톤**이다.
정상이면 모니터에 `==== R-Cube firmware boot ====` 와 주기적 `motion tick` 로그가 찍힌다.

## 구성 파일

- `CMakeLists.txt` / `main/CMakeLists.txt` : ESP-IDF 프로젝트 정의
- `sdkconfig.defaults` : 타겟/플래시/PSRAM/NimBLE/TWAI-IRAM/파티션 기본값
- `partitions.csv` : 2-OTA 앱 + LittleFS 미션 저장 파티션
