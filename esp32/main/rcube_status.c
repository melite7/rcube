#include "rcube_status.h"
#include "board_led.h"
#include "rcube_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "status";

typedef struct { uint8_t r, g, b; } rgb_t;

/* 자릿수/노드ID(0~9) → 색. 기획서 노드ID 규약(0=White 1=Red 2=Green …). */
static const rgb_t PALETTE[10] = {
    {255, 255, 255},  /* 0 White(미할당) */
    {255,   0,   0},  /* 1 Red */
    {  0, 255,   0},  /* 2 Green */
    {  0,   0, 255},  /* 3 Blue */
    {  0, 255, 255},  /* 4 Cyan */
    {255,   0, 255},  /* 5 Magenta */
    {255, 255,   0},  /* 6 Yellow */
    {148,   0, 211},  /* 7 Violet */
    {255,  90,   0},  /* 8 Orange */
    {128,   0,   0},  /* 9 어두운 Red */
};
static const rgb_t WHITE = {255, 255, 255};
static const rgb_t BLACK = {0, 0, 0};

/* 점멸 주기(기획서 5장): 일반 1초, 설정모드 0.25초. */
#define BLINK_MS        1000
#define CONFIG_BLINK_MS  250
/* 모드/색 변경을 이 간격으로 재확인해 페이즈를 즉시 끊는다. */
#define TICK_MS           25

/* 노드 태스크 외부에서 set, 노드 태스크에서 read. */
static volatile rcube_led_mode_t s_mode = RCUBE_LED_IDLE;
static volatile bool s_color_set;       /* 상위 지정색 사용 여부 */
static volatile uint8_t s_cr, s_cg, s_cb;
static volatile bool s_connect_entered; /* 연결모드 최초 진입 여부 */

static rgb_t color_for_digit(uint8_t d)
{
    return (d < 10) ? PALETTE[d] : PALETTE[0];
}

void rcube_status_node_color(uint8_t node_id, uint8_t *r, uint8_t *g, uint8_t *b)
{
    rgb_t c = color_for_digit(node_id);
    if (r) *r = c.r;
    if (g) *g = c.g;
    if (b) *b = c.b;
}

/* 지금 표시해야 할 색: 상위 지정색이 있으면 그것, 없으면 자기 노드ID 색. */
static rgb_t current_color(void)
{
    if (s_color_set) {
        rgb_t c = { s_cr, s_cg, s_cb };
        return c;
    }
    return color_for_digit(rcube_config_node_id());
}

/* ---- 그룹번호 LED(LED2): 전원 내내 ---------------------------------- */
static void group_task(void *arg)
{
    ESP_LOGI(TAG, "group LED 표시 시작(LED2)");
    while (1) {
        uint8_t group = rcube_config_group_id();
        rgb_t c1 = color_for_digit(group / 10);
        rgb_t c2 = color_for_digit(group % 10);
        board_led_set_group(c1.r, c1.g, c1.b); vTaskDelay(pdMS_TO_TICKS(500));
        board_led_set_group(BLACK.r, BLACK.g, BLACK.b); vTaskDelay(pdMS_TO_TICKS(500));
        board_led_set_group(c2.r, c2.g, c2.b); vTaskDelay(pdMS_TO_TICKS(1000));
        board_led_set_group(BLACK.r, BLACK.g, BLACK.b); vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ---- 노드ID LED(LED0·LED1) ------------------------------------------
 * color를 ms 동안 표시한다. 도중 모드가 바뀌거나 이번 주기의 기준색(base)이
 * 달라지면 즉시 true(중단) 반환.
 *
 * ※ 비교 기준이 base인 이유: 점멸의 OFF 구간에서는 출력색이 검정이므로,
 *   출력색과 current_color()를 비교하면 매 틱마다 "색이 바뀌었다"고 오판해
 *   OFF 구간이 0ms로 끊긴다(= 상시 점등처럼 보임). */
static bool node_show(rgb_t color, uint32_t ms, rcube_led_mode_t mode_now, rgb_t base)
{
    board_led_set_node(color.r, color.g, color.b);
    for (uint32_t t = 0; t < ms; t += TICK_MS) {
        if (s_mode != mode_now) {
            return true;   /* 모드 전이 → 페이즈 중단 */
        }
        if (mode_now != RCUBE_LED_CONFIG) {
            rgb_t now = current_color();
            if (now.r != base.r || now.g != base.g || now.b != base.b) {
                return true;   /* 상위 지정색 변경 → 즉시 반영 */
            }
        }
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    }
    return false;
}

static void node_task(void *arg)
{
    ESP_LOGI(TAG, "node LED 표시 시작(LED0·1)");
    while (1) {
        rcube_led_mode_t mode = s_mode;
        rgb_t base = current_color();   /* 이번 주기 동안의 기준색 */
        switch (mode) {
        case RCUBE_LED_CONFIG:
            /* 설정모드: 흰색 0.25s ON / 0.25s OFF 빠른 점멸 */
            if (node_show(WHITE, CONFIG_BLINK_MS, mode, base)) break;
            node_show(BLACK, CONFIG_BLINK_MS, mode, base);
            break;

        case RCUBE_LED_LINKED:
            /* 연결 완료: 상시 점등 (TICK 단위로 색/모드만 재확인) */
            node_show(base, BLINK_MS, mode, base);
            break;

        case RCUBE_LED_IDLE:
        case RCUBE_LED_HUB_WAIT:
        default:
            /* 미연결 / 허브 멤버 대기: 1s ON / 1s OFF 점멸 */
            if (node_show(base, BLINK_MS, mode, base)) break;
            node_show(BLACK, BLINK_MS, mode, base);
            break;
        }
    }
}

void rcube_status_start(void)
{
    s_mode = RCUBE_LED_IDLE;
    s_color_set = false;
    s_connect_entered = false;
    xTaskCreatePinnedToCore(group_task, "grpled", 3072, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(node_task, "nodeled", 3072, NULL, 4, NULL, 0);
}

void rcube_status_set_mode(rcube_led_mode_t mode)
{
    /* 설정모드의 흰색 빠른 점멸은 "설정모드인데 아직 안 붙었다"는 표시다. 그래서
     * 미연결 표시(IDLE = 광고중 1초 점멸)로는 덮이지 않는다 — 설정모드 광고를
     * 시작할 때 adv_start()가 IDLE을 부르므로 그대로 두면 곧바로 지워진다.
     * 반대로 상위가 실제로 붙으면(LINKED/HUB_WAIT) 설정모드 표시는 끝난다.
     * 그래야 연결 후 노드ID/가상 노드ID 색이 보인다(기획서 5장). */
    if (s_mode == RCUBE_LED_CONFIG && mode == RCUBE_LED_IDLE) {
        return;
    }
    if (s_mode != mode) {
        ESP_LOGI(TAG, "LED 모드 %d → %d", (int)s_mode, (int)mode);
        s_mode = mode;
    }
}

rcube_led_mode_t rcube_status_mode(void)
{
    return s_mode;
}

void rcube_status_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    s_cr = r; s_cg = g; s_cb = b;
    s_color_set = true;
}

void rcube_status_clear_color(void)
{
    s_color_set = false;
}

bool rcube_status_enter_connect_mode(void)
{
    if (s_connect_entered) {
        return false;
    }
    s_connect_entered = true;
    ESP_LOGI(TAG, "연결모드 진입");
    return true;
}

void rcube_status_exit_connect_mode(void)
{
    if (s_connect_entered) {
        s_connect_entered = false;
        ESP_LOGI(TAG, "연결모드 종료(대기모드) — 다음 버튼 누름이 다시 연결모드로 들어간다");
    }
}

void rcube_status_enter_config_mode(void)
{
    s_color_set = false;
    s_mode = RCUBE_LED_CONFIG;
    ESP_LOGI(TAG, "설정모드 진입(노드LED 흰색 0.25s 점멸)");
}

bool rcube_status_in_config_mode(void)
{
    return s_mode == RCUBE_LED_CONFIG;
}
