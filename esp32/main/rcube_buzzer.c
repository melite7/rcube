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

/* ★ 부저 구동 극성 — Active-LOW (회로도 확인, 2026-07-30)
 *
 *   CONTROL_BUZZ ──[R13 1K]── B│ Q3 (PNP)
 *                             E├── VCC5_33_BUZZ
 *                             C├── BUZZER U6 ── GND_BUZZ
 *
 * PNP는 베이스가 이미터보다 낮을 때 도통한다. 따라서
 *   LOW  = Q3 ON  = 소리
 *   HIGH = Q3 OFF = 무음
 * 무음을 LOW로 두면 부저가 계속 켜져 발열한다(실제로 그랬다).
 *
 * 고임피던스도 쓰면 안 된다: 베이스에 풀업이 없어 플로팅되면 Q3가 선형 영역에
 * 걸려 스스로 전력을 태운다. 무음은 반드시 HIGH로 "확실히 끈다".
 *
 * ⚠️ VCC5_33_BUZZ는 반드시 3.3V여야 한다. 5V면 GPIO를 HIGH(3.3V)로 올려도
 *    V_EB = 5 - 3.3 = 1.7V로 여전히 도통해 부저가 꺼지지 않는다. */
#define BUZZER_ACTIVE_LOW   1

/* 내장 멜로디 진폭 0.9 고정(큐브_멜로디_데이터.xlsx '개요': 전 곡 0.9). */
#define BUZZER_AMPLITUDE    0.9f

/* 전역 음량 0~100(%). 사각파 ON 비율을 줄여 부저에 실리는 에너지를 낮춘다.
 * 100이면 ON≈45%(진폭 0.9 × 최대 50%)로 가장 크고, 낮추면 짧은 펄스가 되어 조용해진다.
 * 음높이(주파수)는 그대로이므로 음정은 변하지 않는다.
 * 더 줄이려면 이 값만 낮추면 된다. 하드웨어로 더 줄이려면 부저에 직렬 저항을 넣는다. */
#define BUZZER_VOLUME_DEFAULT 4
/* 음 사이 짧은 무음(반복음 분리용). */
#define INTER_NOTE_GAP_MS   6

/* 큐 항목: 대부분은 id만 쓰고, GROUP은 arg(그룹번호)를, TONE은 freq/dur를 함께 싣는다. */
typedef struct {
    rcube_melody_id_t id;
    uint8_t           arg;
    uint16_t          freq_hz;
    uint16_t          dur_ms;
} buz_item_t;

static QueueHandle_t s_queue;
static bool s_ready;

static uint8_t s_volume = BUZZER_VOLUME_DEFAULT;

/* 원하는 "ON 시간" 틱수. 사각파 최대는 50%이고, 거기에 멜로디 진폭과 음량을 곱한다. */
static uint32_t tone_duty(void)
{
    uint32_t half = (BUZZER_DUTY_MAX + 1u) / 2u;   /* 512 = 50% */
    uint32_t on = (uint32_t)((float)half * BUZZER_AMPLITUDE * (s_volume / 100.0f));
    if (on == 0 && s_volume > 0) {
        on = 1;   /* 음량이 아주 낮아도 완전 무음이 되지 않게 최소 1틱 */
    }
    return on;
}

void rcube_buzzer_set_volume(uint8_t percent)
{
    s_volume = (percent > 100) ? 100 : percent;
    ESP_LOGI(TAG, "음량 %u%% (ON 비율 ≈ %.1f%%)", s_volume,
             100.0f * (float)tone_duty() / (float)(BUZZER_DUTY_MAX + 1u));
}

uint8_t rcube_buzzer_volume(void) { return s_volume; }

/* 무음: Q3를 확실히 끄는 레벨로 고정한다(Active-LOW이므로 HIGH). */
#define BUZZER_IDLE_LEVEL (BUZZER_ACTIVE_LOW ? 1u : 0u)

/* 소리 낼 때 채널에 넣을 duty. LEDC duty는 "HIGH 시간"이라, Active-LOW에서는
 * 원하는 ON 비율(≈45%)만큼 LOW로 있어야 하므로 반전시킨다. */
static uint32_t tone_duty_wire(void)
{
    uint32_t on = tone_duty();
    return BUZZER_ACTIVE_LOW ? ((BUZZER_DUTY_MAX + 1u) - on) : on;
}

static void pin_silence(void)
{
    ledc_stop(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, BUZZER_IDLE_LEVEL);
}

void rcube_buzzer_tone(uint16_t freq_hz)
{
    if (!s_ready) {
        return;
    }
    if (freq_hz == 0) {
        pin_silence();
        return;
    }
    ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_TIMER, freq_hz);
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, tone_duty_wire());
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
        if (item.id == RCUBE_MELODY_TONE) {
            /* 미션 TONE 키프레임 / 0xE6 명령. 음 하나를 지정 주파수·길이로 낸다.
             * 여기서는 음 사이 간격(INTER_NOTE_GAP_MS)을 넣지 않는다 — 언제 울릴지는
             * 상위(미션 시퀀서)가 t_ms로 정하므로 부저가 시간을 더하면 어긋난다. */
            ESP_LOGI(TAG, "tone %u Hz / %u ms", item.freq_hz, item.dur_ms);
            rcube_buzzer_tone(item.freq_hz);
            vTaskDelay(pdMS_TO_TICKS(item.dur_ms));
            rcube_buzzer_tone(0);
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
        /* Active-LOW이므로 duty 0(=계속 LOW)은 "부저 ON"이다. 초기값부터 무음
         * 레벨로 잡아, 설정과 pin_silence() 사이의 짧은 구간에도 소리가 나지 않게 한다. */
        .duty       = BUZZER_ACTIVE_LOW ? (BUZZER_DUTY_MAX + 1u) : 0u,
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
    pin_silence();   /* Q3를 확실히 끈 상태로 시작 */
    ESP_LOGI(TAG, "buzzer ready on GPIO %d (LEDC timer%d ch%d, Active-%s, 무음=%s, "
                  "음량 %u%% → ON 비율 %.1f%%)",
             BUZZER_GPIO, (int)BUZZER_LEDC_TIMER, (int)BUZZER_LEDC_CHANNEL,
             BUZZER_ACTIVE_LOW ? "LOW(PNP)" : "HIGH",
             BUZZER_IDLE_LEVEL ? "HIGH" : "LOW",
             s_volume, 100.0f * (float)tone_duty() / (float)(BUZZER_DUTY_MAX + 1u));
}

static void enqueue_full(rcube_melody_id_t id, uint8_t arg, uint16_t freq_hz, uint16_t dur_ms)
{
    if (!s_ready || s_queue == NULL) {
        return;
    }
    buz_item_t item = { .id = id, .arg = arg, .freq_hz = freq_hz, .dur_ms = dur_ms };
    if (xQueueSend(s_queue, &item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "melody 큐 가득참 → drop (id=%d)", (int)id);
    }
}

static void enqueue(rcube_melody_id_t id, uint8_t arg)
{
    enqueue_full(id, arg, 0, 0);
}

void rcube_buzzer_play(rcube_melody_id_t id)
{
    enqueue(id, 0);
}

void rcube_buzzer_play_group(uint8_t group_id)
{
    enqueue(RCUBE_MELODY_GROUP, group_id);
}

void rcube_buzzer_play_tone(uint16_t freq_hz, uint16_t dur_ms)
{
    if (dur_ms == 0) {
        return;
    }
    enqueue_full(RCUBE_MELODY_TONE, 0, freq_hz, dur_ms);
}
