#include "rcube_status.h"
#include "board_led.h"
#include "rcube_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "status";

typedef struct { uint8_t r, g, b; } rgb_t;

/* 자릿수/노드ID(0~9) → 색. 기획서 노드ID 규약과 정렬(1=Red, 2=Green…).
 *   0=White 1=Red 2=Green 3=Blue 4=Cyan 5=Magenta 6=Yellow 7=Violet 8=Orange
 *   9=어두운 Red(Red의 1/2 밝기). (노드ID는 0~8만, 그룹 자리는 0~9)
 * 자리 0=White라 그룹 00은 자연히 흰색+흰색으로 표시된다. */
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
static const rgb_t BLACK = {0, 0, 0};

/* 연결모드 플래그(버튼 태스크에서 set, 노드 표시 태스크에서 read). */
static volatile bool s_connect_mode;

static rgb_t color_for_digit(uint8_t d)
{
    return (d < 10) ? PALETTE[d] : PALETTE[0];
}

/* ---- 그룹번호 LED(LED2): 전원 내내 표시 ---------------------------- */
/* group LED를 ms 동안 켠다(중단 없음, 그룹 표시는 항상 유지). */
static void group_show(rgb_t c, uint32_t ms)
{
    board_led_set_group(c.r, c.g, c.b);
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static void group_task(void *arg)
{
    ESP_LOGI(TAG, "group LED 표시 시작(LED2)");
    while (1) {
        uint8_t group = rcube_config_group_id();
        rgb_t c1 = color_for_digit(group / 10);   /* 십의자리 = 첫색(짧게) */
        rgb_t c2 = color_for_digit(group % 10);   /* 일의자리 = 둘째색(길게) */
        group_show(c1, 500);
        group_show(BLACK, 500);
        group_show(c2, 1000);
        group_show(BLACK, 500);
    }
}

/* ---- 노드ID LED(LED0·LED1): 연결모드 전까지 1s ON / 1s OFF --------- */
/* color를 ms 동안 켠다. 도중 연결모드 전환되면 즉시 true(중단) 반환. */
static bool node_show_or_abort(rgb_t color, uint32_t ms)
{
    if (s_connect_mode) return true;
    board_led_set_node(color.r, color.g, color.b);
    const uint32_t chunk = 25;
    for (uint32_t t = 0; t < ms; t += chunk) {
        if (s_connect_mode) return true;
        vTaskDelay(pdMS_TO_TICKS(chunk));
    }
    return s_connect_mode;
}

static void node_task(void *arg)
{
    ESP_LOGI(TAG, "node LED 표시 시작(LED0·1, 노드ID 색 1s 점멸)");
    while (!s_connect_mode) {
        uint8_t node = rcube_config_node_id();      /* 매 주기 재읽어 반영 */
        rgb_t c = color_for_digit(node);            /* 0=White(미할당) */
        if (node_show_or_abort(c, 1000)) break;     /* 1s ON */
        if (node_show_or_abort(BLACK, 1000)) break; /* 1s OFF */
    }
    ESP_LOGI(TAG, "node LED 표시 종료(연결모드 진입)");
    vTaskDelete(NULL);
}

void rcube_status_start_identity(void)
{
    s_connect_mode = false;
    xTaskCreatePinnedToCore(group_task, "grpled", 3072, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(node_task, "nodeled", 3072, NULL, 4, NULL, 0);
}

bool rcube_status_enter_connect_mode(void)
{
    if (s_connect_mode) {
        return false;
    }
    s_connect_mode = true;   /* node 태스크가 곧 LED0·1을 놓아준다 */
    ESP_LOGI(TAG, "연결모드 진입");
    return true;
}

bool rcube_status_in_connect_mode(void)
{
    return s_connect_mode;
}
