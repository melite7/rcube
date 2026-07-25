/*
 * bmi088 — BMI088 6축 IMU 드라이버 (SPI, ESP32-S3)
 * ------------------------------------------------------------------
 * BMI088은 가속도(accel)와 자이로(gyro) 다이가 분리되어 CS가 2개다.
 * 핀맵(메인보드): SPI2(FSPI)
 *   IO11=MOSI, IO12=SCLK, IO13=MISO, IO10=CS_ACC(가속도), IO9=CS_GYR(자이로)
 *
 * bmi088_init():
 *   - SPI2 버스 + accel/gyro 디바이스 등록
 *   - 소프트리셋 → accel SPI 활성화·전원 ON → chip ID 검증
 *   - 두 chip ID(accel 0x1E, gyro 0x0F) 모두 확인되면 ESP_OK, 아니면 오류.
 *
 * ※ 개발보드(센서 미장착)에서는 chip ID가 안 맞아 ESP_ERR_NOT_FOUND를 돌려준다.
 *   메인보드(BMI088 장착)에서 정상 동작한다.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* SPI 버스/디바이스 준비 + 센서 초기화·검증. app_main에서 1회. */
esp_err_t bmi088_init(void);

/* 초기화에서 센서가 검출되었는지. */
bool bmi088_present(void);

/* 가속도 원시값(int16 ×3). ESP_OK 성공. */
esp_err_t bmi088_read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az);

/* 자이로 원시값(int16 ×3). ESP_OK 성공. */
esp_err_t bmi088_read_gyro_raw(int16_t *gx, int16_t *gy, int16_t *gz);

/* 원시값 변환 헬퍼(현재 설정 기준). accel→mg, gyro→dps(0.1dps 단위 아님, float). */
float bmi088_accel_mg(int16_t raw);
float bmi088_gyro_dps(int16_t raw);
