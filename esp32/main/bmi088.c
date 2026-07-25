#include "bmi088.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"

static const char *TAG = "bmi088";

/* ---- 핀맵 (메인보드 SPI2) ---- */
#define PIN_MOSI    11
#define PIN_SCLK    12
#define PIN_MISO    13
#define PIN_CS_ACC  10
#define PIN_CS_GYR   9
#define BMI_HOST    SPI2_HOST
#define BMI_CLK_HZ  (10 * 1000 * 1000)   /* BMI088 최대 10MHz */

/* ---- 레지스터 ---- */
#define ACC_CHIP_ID     0x00   /* 기대값 0x1E */
#define ACC_DATA_X_LSB  0x12   /* 0x12..0x17: X/Y/Z LSB,MSB */
#define ACC_CONF        0x40
#define ACC_RANGE       0x41
#define ACC_PWR_CONF    0x7C
#define ACC_PWR_CTRL    0x7D
#define ACC_SOFTRESET   0x7E

#define GYR_CHIP_ID     0x00   /* 기대값 0x0F */
#define GYR_RATE_X_LSB  0x02   /* 0x02..0x07 */
#define GYR_RANGE       0x0F
#define GYR_BANDWIDTH   0x10
#define GYR_SOFTRESET   0x14

#define ACC_CHIP_ID_VAL 0x1E
#define GYR_CHIP_ID_VAL 0x0F
#define SOFTRESET_CMD   0xB6

/* 설정값(스캐폴딩 기본): accel ±6g/100Hz, gyro ±2000dps/±… */
#define ACC_RANGE_6G    0x01   /* 0=±3g 1=±6g 2=±12g 3=±24g */
#define ACC_CONF_100HZ  0xA8   /* bwp=normal, odr=100Hz */
#define GYR_RANGE_2000  0x00   /* 0=±2000dps */
#define GYR_BW_2000_532 0x00   /* ODR 2000Hz / BW 532Hz */

static spi_device_handle_t s_acc;
static spi_device_handle_t s_gyr;
static bool s_present;

/* ---- 저수준 SPI ------------------------------------------------------ */
/* 쓰기: [reg&0x7F][val]. */
static esp_err_t reg_write(spi_device_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx };
    return spi_device_polling_transmit(dev, &t);
}

/* accel 읽기: [reg|0x80][dummy][data...] — accel은 첫 데이터 앞에 더미 1바이트. */
static esp_err_t acc_read(uint8_t reg, uint8_t *out, size_t n)
{
    uint8_t tx[10] = {0};
    uint8_t rx[10] = {0};
    size_t total = n + 2;                 /* reg + dummy + data */
    if (total > sizeof(tx)) return ESP_ERR_INVALID_SIZE;
    tx[0] = (uint8_t)(reg | 0x80);
    spi_transaction_t t = { .length = total * 8, .tx_buffer = tx, .rx_buffer = rx };
    esp_err_t err = spi_device_polling_transmit(s_acc, &t);
    if (err == ESP_OK) memcpy(out, rx + 2, n);   /* reg 에코 + 더미 건너뜀 */
    return err;
}

/* gyro 읽기: [reg|0x80][data...] — 더미 없음. */
static esp_err_t gyr_read(uint8_t reg, uint8_t *out, size_t n)
{
    uint8_t tx[10] = {0};
    uint8_t rx[10] = {0};
    size_t total = n + 1;                 /* reg + data */
    if (total > sizeof(tx)) return ESP_ERR_INVALID_SIZE;
    tx[0] = (uint8_t)(reg | 0x80);
    spi_transaction_t t = { .length = total * 8, .tx_buffer = tx, .rx_buffer = rx };
    esp_err_t err = spi_device_polling_transmit(s_gyr, &t);
    if (err == ESP_OK) memcpy(out, rx + 1, n);
    return err;
}

/* ---- 초기화 ---------------------------------------------------------- */
static esp_err_t bus_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    esp_err_t err = spi_bus_initialize(BMI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize 실패: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t acc_cfg = {
        .clock_speed_hz = BMI_CLK_HZ,
        .mode = 0,                    /* BMI088 SPI mode 0 */
        .spics_io_num = PIN_CS_ACC,
        .queue_size = 1,
    };
    spi_device_interface_config_t gyr_cfg = {
        .clock_speed_hz = BMI_CLK_HZ,
        .mode = 0,
        .spics_io_num = PIN_CS_GYR,
        .queue_size = 1,
    };
    err = spi_bus_add_device(BMI_HOST, &acc_cfg, &s_acc);
    if (err != ESP_OK) { ESP_LOGE(TAG, "add accel 실패: %s", esp_err_to_name(err)); return err; }
    err = spi_bus_add_device(BMI_HOST, &gyr_cfg, &s_gyr);
    if (err != ESP_OK) { ESP_LOGE(TAG, "add gyro 실패: %s", esp_err_to_name(err)); return err; }
    return ESP_OK;
}

esp_err_t bmi088_init(void)
{
    esp_err_t err = bus_init();
    if (err != ESP_OK) return err;

    uint8_t id = 0;

    /* --- 가속도: 소프트리셋 → SPI 활성화(더미 읽기) → 전원 ON --- */
    reg_write(s_acc, ACC_SOFTRESET, SOFTRESET_CMD);
    vTaskDelay(pdMS_TO_TICKS(50));
    acc_read(ACC_CHIP_ID, &id, 1);        /* 더미 읽기 — SPI 인터페이스 활성화 */
    vTaskDelay(pdMS_TO_TICKS(5));
    reg_write(s_acc, ACC_PWR_CONF, 0x00); /* suspend 해제(active) */
    vTaskDelay(pdMS_TO_TICKS(5));
    reg_write(s_acc, ACC_PWR_CTRL, 0x04); /* accelerometer ON */
    vTaskDelay(pdMS_TO_TICKS(50));
    reg_write(s_acc, ACC_RANGE, ACC_RANGE_6G);
    reg_write(s_acc, ACC_CONF, ACC_CONF_100HZ);
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t acc_id = 0;
    acc_read(ACC_CHIP_ID, &acc_id, 1);

    /* --- 자이로: 소프트리셋 → chip id --- */
    reg_write(s_gyr, GYR_SOFTRESET, SOFTRESET_CMD);
    vTaskDelay(pdMS_TO_TICKS(50));
    reg_write(s_gyr, GYR_RANGE, GYR_RANGE_2000);
    reg_write(s_gyr, GYR_BANDWIDTH, GYR_BW_2000_532);
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t gyr_id = 0;
    gyr_read(GYR_CHIP_ID, &gyr_id, 1);

    ESP_LOGI(TAG, "chip id: accel=0x%02x(기대 0x1E), gyro=0x%02x(기대 0x0F)", acc_id, gyr_id);

    if (acc_id == ACC_CHIP_ID_VAL && gyr_id == GYR_CHIP_ID_VAL) {
        s_present = true;
        ESP_LOGI(TAG, "BMI088 검출 OK (SPI2: MOSI%d SCLK%d MISO%d CS_ACC%d CS_GYR%d)",
                 PIN_MOSI, PIN_SCLK, PIN_MISO, PIN_CS_ACC, PIN_CS_GYR);
        return ESP_OK;
    }

    s_present = false;
    ESP_LOGW(TAG, "BMI088 미검출 — 센서 미장착(개발보드?) 또는 배선 확인. SPI 환경은 준비됨.");
    return ESP_ERR_NOT_FOUND;
}

bool bmi088_present(void) { return s_present; }

/* ---- 데이터 읽기 ----------------------------------------------------- */
esp_err_t bmi088_read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t d[6];
    esp_err_t err = acc_read(ACC_DATA_X_LSB, d, 6);
    if (err != ESP_OK) return err;
    *ax = (int16_t)((d[1] << 8) | d[0]);
    *ay = (int16_t)((d[3] << 8) | d[2]);
    *az = (int16_t)((d[5] << 8) | d[4]);
    return ESP_OK;
}

esp_err_t bmi088_read_gyro_raw(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t d[6];
    esp_err_t err = gyr_read(GYR_RATE_X_LSB, d, 6);
    if (err != ESP_OK) return err;
    *gx = (int16_t)((d[1] << 8) | d[0]);
    *gy = (int16_t)((d[3] << 8) | d[2]);
    *gz = (int16_t)((d[5] << 8) | d[4]);
    return ESP_OK;
}

/* ±6g, 16bit → mg. (범위 0x01 = ±6g) */
float bmi088_accel_mg(int16_t raw)
{
    return (float)raw / 32768.0f * 6000.0f;
}

/* ±2000dps, 16bit → dps. */
float bmi088_gyro_dps(int16_t raw)
{
    return (float)raw / 32768.0f * 2000.0f;
}
