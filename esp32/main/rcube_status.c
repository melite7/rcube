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
    {255, 255, 255},  /* 0 White */
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

/* 노드LED 소유권 플래그(다른 태스크에서 set, 노드 태스크에서 read). */
static volatile bool s_config;   /* 설정모드: 흰색 0.25s 빠른 점멸 */
static volatile bool s_yield;    /* BLE/명령 레이어가 노드LED 소유(연결/광고) */
static volatile bool s_connect_entered;  /* 연결모드 최초 진입 여부 */

static rgb_t color_for_digit(uint8_t d)
{
    return (d < 10) ? PALETTE[d] : PALETTE[0];
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

/* ---- 노드ID LED(LED0·LED1) --------------------------------------------
 * color를 ms 동안 켠다. 도중 소유권이 넘어가면(s_yield) 즉시 true(중단) 반환.
 * 설정모드 전환(s_config) 역시 현재 페이즈를 끊도록 change 플래그로 감시. */
static bool node_show(rgb_t color, uint32_t ms, bool config_now)
{
    if (s_yield) {
        return true;
    }
    board_led_set_node(color.r, color.g, color.b);
    const uint32_t chunk = 25;
    for (uint32_t t = 0; t < ms; t += chunk) {
        if (s_yield || s_config != config_now) {
            return true;   /* 소유권 이전 또는 모드 변경 → 페이즈 중단 */
        }
        vTaskDelay(pdMS_TO_TICKS(chunk));
    }
    return false;
}

static void node_task(void *arg)
{
    ESP_LOGI(TAG, "node LED 표시 시작(LED0·1)");
    while (1) {
        if (s_yield) {
            vTaskDelay(pdMS_TO_TICKS(50));   /* BLE/명령이 소유 — 손대지 않음 */
            continue;
        }
        if (s_config) {
            /* 설정모드: 흰색 0.25s ON / 0.25s OFF 빠른 점멸 */
            node_show(WHITE, 250, true);
            node_show(BLACK, 250, true);
        } else {
            /* identity: 노드ID 색 1s ON / 1s OFF */
            rgb_t c = color_for_digit(rcube_config_node_id());
            node_show(c, 1000, false);
            node_show(BLACK, 1000, false);
        }
    }
}

void rcube_status_start_identity(void)
{
    s_config = false;
    s_yield = false;
    s_connect_entered = false;
    xTaskCreatePinnedToCore(group_task, "grpled", 3072, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(node_task, "nodeled", 3072, NULL, 4, NULL, 0);
}

bool rcube_status_enter_connect_mode(void)
{
    s_yield = true;   /* 광고/연결 색을 ble_rcube가 표시 */
    if (s_connect_entered) {
        return false;
    }
    s_connect_entered = true;
    ESP_LOGI(TAG, "연결모드 진입(노드LED → BLE 소유)");
    return true;
}

void rcube_status_enter_config_mode(void)
{
    s_config = true;
    s_yield = false;   /* 노드LED를 설정모드 흰색 점멸이 소유 */
    ESP_LOGI(TAG, "설정모드 진입(노드LED 흰색 0.25s 점멸)");
}

void rcube_status_on_connected(void)
{
    s_yield = true;   /* 연결됨 — 상위 지정색이 표시되도록 소유권 이전 */
}

void rcube_status_on_disconnected(void)
{
    if (s_config) {
        s_yield = false;   /* 설정모드였다면 흰색 점멸 재개 */
    }
}

bool rcube_status_in_config_mode(void)
{
    return s_config;
}
