#include "rcube_buzzer.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/ledc.h"

#include "rcube_ledc.h"

static const char *TAG = "buz";

/* 핀맵: IO14 = BUZ_PWM (LEDC).
 * 타이머·채널은 rcube_ledc.h에서 배정한다 — 서보(IO21)와 타이머를 공유하면
 * 주파수가 서로 덮어써지므로 그쪽에서 한 번에 관리한다. */
#define BUZZER_GPIO         14
#define BUZZER_LEDC_MODE    RCUBE_LEDC_MODE
#define BUZZER_LEDC_TIMER   RCUBE_LEDC_BUZZER_TIMER
#define BUZZER_LEDC_CHANNEL RCUBE_LEDC_BUZZER_CHANNEL
#define BUZZER_DUTY_RES     RCUBE_LEDC_BUZZER_RES     /* 0~1023 */
#define BUZZER_DUTY_MAX     ((1u << 10) - 1u)

/* 내장 멜로디 진폭 0.9 고정 → 사각파 duty = 진폭*50%. */
#define BUZZER_AMPLITUDE    0.9f
/* 음 사이 짧은 무음(반복음 분리용). */
#define INTER_NOTE_GAP_MS   6

/* 큐 항목: 대부분은 id만 쓰고, GROUP만 arg(그룹번호)를 함께 싣는다. */
typedef struct {
    rcube_melody_id_t id;
    uint8_t           arg;
} buz_item_t;

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
        /* duty만 0으로 두면 채널이 살아 있어, 타이머 주파수가 그대로 걸린 상태가 된다.
         * ledc_stop으로 신호 생성 자체를 멈추고 출력을 LOW로 고정한다(무음 보장). */
        ledc_stop(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0 /* idle level = LOW */);
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
    buz_item_t item;
    while (1) {
        if (xQueueReceive(s_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (item.id == RCUBE_MELODY_GROUP) {
            /* 그룹번호 알림은 음이 런타임에 정해지므로 여기서 만든다(기획서 5장). */
            rcube_note_t notes[RCUBE_GROUP_NOTE_COUNT];
            uint8_t n = rcube_melody_group_notes(item.arg, notes, RCUBE_GROUP_NOTE_COUNT);
            if (n == 0) {
                continue;
            }
            char name[24];
            snprintf(name, sizeof(name), "GROUP %02u", item.arg);
            rcube_melody_t m = { .notes = notes, .count = n, .name = name };
            play_melody(&m);
            continue;
        }
        const rcube_melody_t *m = rcube_melody(item.id);
        if (m != NULL && m->notes != NULL && m->count > 0) {
            play_melody(m);
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

    s_queue = xQueueCreate(6, sizeof(buz_item_t));
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
    /* 부팅 직후 무음 보장. 채널 설정만으로도 duty 0이지만, 신호 생성을 명시적으로
     * 멈춰 두어야 잔음이 남지 않는다. */
    ledc_stop(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ESP_LOGI(TAG, "buzzer ready on GPIO %d (LEDC timer%d ch%d, 무음 상태)",
             BUZZER_GPIO, (int)BUZZER_LEDC_TIMER, (int)BUZZER_LEDC_CHANNEL);
}

static void enqueue(rcube_melody_id_t id, uint8_t arg)
{
    if (!s_ready || s_queue == NULL) {
        return;
    }
    buz_item_t item = { .id = id, .arg = arg };
    if (xQueueSend(s_queue, &item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "melody 큐 가득참 → drop (id=%d)", (int)id);
    }
}

void rcube_buzzer_play(rcube_melody_id_t id)
{
    enqueue(id, 0);
}

void rcube_buzzer_play_group(uint8_t group_id)
{
    enqueue(RCUBE_MELODY_GROUP, group_id);
}
