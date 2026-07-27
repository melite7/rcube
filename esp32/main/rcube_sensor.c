#include "rcube_sensor.h"
#include "rcube_cmd.h"
#include "bmi088.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "rcube_protocol.h"

static const char *TAG = "sensor";

#define HEADER_LEN 4

static volatile bool     s_streaming;
static volatile uint16_t s_period_ms = RCUBE_SENSOR_PERIOD_DEFAULT_MS;
static TaskHandle_t      s_task;
static bool              s_warned_absent;

/* 센서 프레임 1개를 만들어 현재 전송계층으로 내보낸다.
 * 발신 표시는 0xFE(허브) — 서브에서는 BLE 허브가 멤버 vid로 재기입해 PC로 올리고,
 * CAN에서는 CAN ID의 SrcId가 발신 노드를 담는다. */
static void send_axes(uint8_t kind, int16_t x, int16_t y, int16_t z)
{
    uint8_t f[HEADER_LEN + RCUBE_SENSOR_PAYLOAD_LEN];
    uint16_t total = sizeof(f);
    f[0] = RCUBE_ADDR_HUB;
    f[1] = RCUBE_OP_GetSensors;
    f[2] = (uint8_t)((total >> 8) & 0xFF);
    f[3] = (uint8_t)(total & 0xFF);
    f[4] = kind;
    f[5] = (uint8_t)((x >> 8) & 0xFF); f[6] = (uint8_t)(x & 0xFF);
    f[7] = (uint8_t)((y >> 8) & 0xFF); f[8] = (uint8_t)(y & 0xFF);
    f[9] = (uint8_t)((z >> 8) & 0xFF); f[10] = (uint8_t)(z & 0xFF);
    rcube_cmd_send_frame(f, total);
}

/* IMU를 한 번 읽어 가속도/자이로 프레임 2개를 보낸다.
 * 개발보드처럼 IMU가 실장되지 않은 경우에도 0으로 채워 보낸다 — 그래야 전송·중계·
 * 표시 경로를 센서 없이도 검증할 수 있다(최초 1회만 경고 로그). */
void rcube_sensor_report_once(void)
{
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    bool ok = false;
    if (bmi088_present()) {
        ok = (bmi088_read_accel_raw(&ax, &ay, &az) == ESP_OK) &&
             (bmi088_read_gyro_raw(&gx, &gy, &gz) == ESP_OK);
    }
    if (!ok && !s_warned_absent) {
        s_warned_absent = true;
        ESP_LOGW(TAG, "IMU 미검출/읽기 실패 → 센서값 0으로 전송(경로 검증용)");
    }
    if (ok) {
        /* 와이어 단위: 가속도 mg, 자이로 0.1°/s (앱과 맞춘 계약). */
        ax = (int16_t)bmi088_accel_mg(ax);
        ay = (int16_t)bmi088_accel_mg(ay);
        az = (int16_t)bmi088_accel_mg(az);
        gx = (int16_t)(bmi088_gyro_dps(gx) * 10.0f);
        gy = (int16_t)(bmi088_gyro_dps(gy) * 10.0f);
        gz = (int16_t)(bmi088_gyro_dps(gz) * 10.0f);
    }
    send_axes(RCUBE_SENSOR_KIND_ACCEL, ax, ay, az);
    send_axes(RCUBE_SENSOR_KIND_GYRO, gx, gy, gz);
}

static void sensor_task(void *arg)
{
    while (1) {
        if (!s_streaming) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        rcube_sensor_report_once();
        vTaskDelay(pdMS_TO_TICKS(s_period_ms));
    }
}

void rcube_sensor_init(void)
{
    s_streaming = false;
    s_period_ms = RCUBE_SENSOR_PERIOD_DEFAULT_MS;
    if (xTaskCreatePinnedToCore(sensor_task, "sensor", 3072, NULL, 4, &s_task, 0) != pdPASS) {
        ESP_LOGE(TAG, "sensor 태스크 생성 실패");
        return;
    }
    ESP_LOGI(TAG, "센서 모니터링 준비(대기). B1로 시작/중지, B0로 1회 조회");
}

bool rcube_sensor_set_stream(bool on, uint16_t period_ms)
{
    if (on) {
        if (period_ms < RCUBE_SENSOR_PERIOD_MIN_MS || period_ms > RCUBE_SENSOR_PERIOD_MAX_MS) {
            ESP_LOGW(TAG, "주기 %u ms 범위 밖(%u~%u)", period_ms,
                     RCUBE_SENSOR_PERIOD_MIN_MS, RCUBE_SENSOR_PERIOD_MAX_MS);
            return false;
        }
        s_period_ms = period_ms;
    }
    s_streaming = on;
    ESP_LOGI(TAG, "센서 전송 %s (주기 %u ms)", on ? "시작" : "중지", s_period_ms);
    return true;
}

bool rcube_sensor_streaming(void)   { return s_streaming; }
uint16_t rcube_sensor_period_ms(void) { return s_period_ms; }
