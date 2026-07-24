#include "rcube_buzzer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/ledc.h"

static const char *TAG = "buz";

/* 핀맵: IO14 = BUZ_PWM (LEDC). */
#define BUZZER_GPIO         14
#define BUZZER_LEDC_MODE    LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_TIMER   LEDC_TIMER_0
#define BUZZER_LEDC_CHANNEL LEDC_CHANNEL_0
#define BUZZER_DUTY_RES     LEDC_TIMER_10_BIT      /* 0~1023 */
#define BUZZER_DUTY_MAX     ((1u << 10) - 1u)

/* 내장 멜로디 진폭 0.9 고정 → 사각파 duty = 진폭*50%. */
#define BUZZER_AMPLITUDE    0.9f
/* 음 사이 짧은 무음(반복음 분리용). */
#define INTER_NOTE_GAP_MS   6

static QueueHandle_t s_queue;
static bool s_ready;

static uint32_t tone_duty(void)
{
    return (uint32_t)(BUZZER_AMPLITUDE * (float)(BUZZER_DUTY_MAX / 2));
}

void rcube_buzzer_tone(uint16_t freq_hz)
{
    if (!s_ready) {
        return;
    }
    if (freq_hz == 0) {
        ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
        ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
        return;
    }
    ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_TIMER, freq_hz);
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, tone_duty());
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

static void play_melody(const rcube_melody_t *m)
{
    ESP_LOGI(TAG, "play '%s' (%u notes)", m->name ? m->name : "?", m->count);
    for (uint8_t i = 0; i < m->count; i++) {
        const rcube_note_t *n = &m->notes[i];
        if (n->note_idx == RCUBE_NOTE_REST || n->note_idx >= RCUBE_PIANO_SCALE_LEN) {
            rcube_buzzer_tone(0);
        } else {
            rcube_buzzer_tone(rcube_piano_scale[n->note_idx]);
        }
        vTaskDelay(pdMS_TO_TICKS(n->dur_ms));
        rcube_buzzer_tone(0);
        vTaskDelay(pdMS_TO_TICKS(INTER_NOTE_GAP_MS));
    }
    rcube_buzzer_tone(0);
}

static void buzzer_task(void *arg)
{
    rcube_melody_id_t id;
    while (1) {
        if (xQueueReceive(s_queue, &id, portMAX_DELAY) == pdTRUE) {
            const rcube_melody_t *m = rcube_melody(id);
            if (m != NULL && m->notes != NULL && m->count > 0) {
                play_melody(m);
            }
        }
    }
}

void rcube_buzzer_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = BUZZER_LEDC_MODE,
        .timer_num       = BUZZER_LEDC_TIMER,
        .duty_resolution = BUZZER_DUTY_RES,
        .freq_hz         = 440,                 /* 초기값(재생 시 음마다 변경) */
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config 실패: %d", err);
        return;
    }
    ledc_channel_config_t ch = {
        .gpio_num   = BUZZER_GPIO,
        .speed_mode = BUZZER_LEDC_MODE,
        .channel    = BUZZER_LEDC_CHANNEL,
        .timer_sel  = BUZZER_LEDC_TIMER,
        .duty       = 0,                        /* 기본 무음 */
        .hpoint     = 0,
    };
    err = ledc_channel_config(&ch);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config 실패: %d", err);
        return;
    }

    s_queue = xQueueCreate(6, sizeof(rcube_melody_id_t));
    if (s_queue == NULL) {
        ESP_LOGE(TAG, "queue 생성 실패");
        return;
    }
    BaseType_t ok = xTaskCreatePinnedToCore(buzzer_task, "buzzer", 3072, NULL, 4, NULL, 0);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "task 생성 실패");
        return;
    }
    s_ready = true;
    ESP_LOGI(TAG, "buzzer ready on GPIO %d (LEDC)", BUZZER_GPIO);
}

void rcube_buzzer_play(rcube_melody_id_t id)
{
    if (!s_ready || s_queue == NULL) {
        return;
    }
    if (xQueueSend(s_queue, &id, 0) != pdTRUE) {
        ESP_LOGW(TAG, "melody 큐 가득참 → drop (id=%d)", (int)id);
    }
}
