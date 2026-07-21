# esp32 — R큐브 메인보드 펌웨어 (ESP32-S3)

ESP-IDF(FreeRTOS SMP) 기반 단일 바이너리. 부팅 시 ECF/CMF/멤버맵으로 역할·통신 스택 분기.
- Core 1: 실시간 모션(고정주기 키프레임 송출 + 센서 수집)
- Core 0: NimBLE/TWAI, 역할 레이어, 데이터테이블 시퀀서, MicroPython VM

`components/motor_uart/` : 모터제어보드(STM32G4) UART 어댑터 (vendor/motor-board 스펙 기준)
