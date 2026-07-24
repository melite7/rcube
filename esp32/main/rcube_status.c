#include "rcube_status.h"
#include "board_led.h"
#include "rcube_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "status";

typedef struct { uint8_t r, g, b; } rgb_t;

/* 자릿수(0~9) → 색. 기획서 노드ID 색상 규약과 맞춤(1=Red, 2=Green…).
 *   0=White 1=Red 2=Green 3=Blue 4=Cyan 5=Magenta 6=Yellow 7=Violet 8=Orange
 *   9=어두운 Red(Red의 1/2 밝기={128,0,0}).
 * 자리 0이 White라 그룹 00은 자연히 흰색+흰색으로 표시된다(별도 예외 없음). */
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
    {128,   0,   0},  /* 9 어두운 Red (Red의 1/2 밝기) */
};
static const rgb_t WHITE = {255, 255, 255};
static const rgb_t BLACK = {0, 0, 0};

/* 연결모드 플래그(버튼 태스크에서 set, 아이덴티티 태스크에서 read). */
static volatile bool s_connect_mode;

/* 10진 자리(0~9) → 표시 색. (그룹 전체가 0인 경우만 상위에서 흰색 처리) */
static rgb_t color_for_digit(uint8_t d)
{
    if (d < 10) {
        return PALETTE[d];
    }
    return WHITE;   /* 도달 불가(자리는 0~9) */
}

/* color를 ms 동안 켠다. 도중 연결모드 전환되면 즉시 true(중단) 반환. */
static bool show_or_abort(rgb_t color, uint32_t ms)
{
    if (s_connect_mode) {
        return true;
    }
    board_led_set(color.r, color.g, color.b);
    const uint32_t chunk = 25;
    for (uint32_t t = 0; t < ms; t += chunk) {
        if (s_connect_mode) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(chunk));
    }
    return s_connect_mode;
}

static void identity_task(void *arg)
{
    ESP_LOGI(TAG, "identity 표시 시작(그룹번호 색상)");
    while (!s_connect_mode) {
        /* 매 주기 그룹번호를 다시 읽어 런타임 변경 반영. */
        uint8_t group = rcube_config_group_id();
        rgb_t c1 = color_for_digit(group / 10);   /* 십의자리 = 첫색(짧게) */
        rgb_t c2 = color_for_digit(group % 10);   /* 일의자리 = 둘째색(길게) */
        /* 첫색 0.5s ON → 0.5s OFF → 둘째색 1.0s ON → 0.5s OFF (그룹 00 = 흰+흰) */
        if (show_or_abort(c1, 500))  break;
        if (show_or_abort(BLACK, 500)) break;
        if (show_or_abort(c2, 1000)) break;
        if (show_or_abort(BLACK, 500)) break;
    }
    ESP_LOGI(TAG, "identity 표시 종료(연결모드 진입)");
    vTaskDelete(NULL);
}

void rcube_status_start_identity(void)
{
    s_connect_mode = false;
    BaseType_t ok = xTaskCreatePinnedToCore(identity_task, "identity", 3072, NULL, 4, NULL, 0);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "identity task 생성 실패");
    }
}

bool rcube_status_enter_connect_mode(void)
{
    if (s_connect_mode) {
        return false;
    }
    s_connect_mode = true;   /* 아이덴티티 태스크가 곧 LED를 놓아준다 */
    ESP_LOGI(TAG, "연결모드 진입");
    return true;
}

bool rcube_status_in_connect_mode(void)
{
    return s_connect_mode;
}
