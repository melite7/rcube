#include "board_led.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "led_strip.h"

static const char *TAG = "led";

/* 보드 실물 확인 결과 RGB LED(SK6812)는 GPIO38에 연결되어 있다. */
#define LED_GPIO 38
#define LED_COUNT 1
/* SK6812 데이터 라인 타이밍은 RMT(10MHz)로 정확히 생성한다(비트뱅잉 X). */
#define LED_RMT_RES_HZ (10 * 1000 * 1000)

/* 전역 밝기 상한(0~255). 호출부는 순수 색상(0/255)을 넘기고, 실제 출력은
 * 이 값으로 스케일된다. SK6812는 밝아서 기본을 약하게 잡는다. */
#define LED_BRIGHTNESS 16

static led_strip_handle_t s_strip;
static SemaphoreHandle_t s_lock;

void board_led_init(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_COUNT,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_SK6812,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_RMT_RES_HZ,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip));
    s_lock = xSemaphoreCreateMutex();
    configASSERT(s_lock != NULL);
    ESP_LOGI(TAG, "SK6812 ready on GPIO %d (RMT)", LED_GPIO);
}

void board_led_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (s_strip == NULL) {
        return;
    }
    /* 전역 밝기로 스케일(0/255 순수색을 약하게 낮춰 출력). */
    uint8_t sr = (uint16_t)r * LED_BRIGHTNESS / 255;
    uint8_t sg = (uint16_t)g * LED_BRIGHTNESS / 255;
    uint8_t sb = (uint16_t)b * LED_BRIGHTNESS / 255;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    ESP_ERROR_CHECK(led_strip_set_pixel(s_strip, 0, sr, sg, sb));
    ESP_ERROR_CHECK(led_strip_refresh(s_strip));
    xSemaphoreGive(s_lock);
}
