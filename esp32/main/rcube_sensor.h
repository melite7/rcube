/*
 * rcube_sensor — 센서 모니터링 (기획서 9장)
 * ------------------------------------------------------------------
 * 상위가 "센서 전송 시작" 명령을 내리면 각 큐브가 주기적으로 자기 센서를 올린다.
 * 어디로 올라가는지는 이 모듈이 신경 쓰지 않는다 — rcube_cmd의 현재 응답 경로를
 * 그대로 타므로 전송계층이 알아서 갈린다(기획서 9장 분기 그대로).
 *
 *   서브 · CAN 큐브  : CAN으로 PC(USB-CAN)에 직접
 *   서브 · BLE 멤버  : BLE 허브 큐브로 → 허브가 자기 것과 함께 PC로 (중계는 ble_multirole)
 *   독립 · CAN 멤버  : CAN으로 edge central에 직접
 *   독립 · BLE 멤버  : BLE로 edge central에 직접(허브 경유 아님)
 *
 * 와이어 프레임: OpCode 0xB0(GetSensors), payload 7바이트
 *   [0]   kind   0x00=가속도(mg), 0x01=자이로(0.1°/s)
 *   [1:2] x (int16, big-endian)   [3:4] y   [5:6] z
 * 7바이트로 맞춘 이유는 Classic CAN 데이터필드(8B)에 멀티프레임 없이 들어가야 하기
 * 때문이다. 한 번 보고는 가속도·자이로 2프레임으로 나간다.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define RCUBE_SENSOR_KIND_ACCEL   0x00u
#define RCUBE_SENSOR_KIND_GYRO    0x01u
#define RCUBE_SENSOR_PAYLOAD_LEN  7u

#define RCUBE_SENSOR_PERIOD_MIN_MS      20u
#define RCUBE_SENSOR_PERIOD_MAX_MS   10000u
#define RCUBE_SENSOR_PERIOD_DEFAULT_MS 200u

/* 센서 태스크 기동(전송은 정지 상태로 시작). app_main에서 1회. */
void rcube_sensor_init(void);

/* 지금 1회만 읽어 보고(0xB0 GetSensors 대응). */
void rcube_sensor_report_once(void);

/* 주기 전송 시작/중지(0xB1 SetSensorStream 대응). 주기가 범위 밖이면 false. */
bool rcube_sensor_set_stream(bool on, uint16_t period_ms);

bool rcube_sensor_streaming(void);
uint16_t rcube_sensor_period_ms(void);
