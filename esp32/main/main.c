/*
 * R큐브 메인보드 펌웨어 — app_main (Phase 1: BLE 연결 골격)
 * ------------------------------------------------------------------
 * 동작:
 *   1) 전원 인가 → 부팅 → 온보드 SK6812를 파란색으로 점등(부팅 성공 표시).
 *   2) BOOT 버튼(GPIO0)을 누르면 → 자신을 "RCUBE00.00"으로 BLE 광고 시작.
 *      외부(폰/PC)에서 이 이름으로 찾아 연결할 수 있다.
 *
 * LED 상태색: 파랑=대기, 청록=광고중, 초록=연결됨. (ble_rcube.c에서 갱신)
 *
 * 코어 배치 (로드맵 12.2):
 *   - Core 1 : 실시간 모션(고정주기 키프레임 송출 + 센서 수집).
 *   - Core 0 : 통신(NimBLE/TWAI), 역할 레이어, 시퀀서 등.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

#include "board_led.h"
#include "ble_rcube.h"
#include "rcube_config.h"

static const char *TAG = "rcube";

/* ESP32-S3 DevKit의 BOOT 버튼은 GPIO0에 연결(외부 풀업, 눌림=Low). */
#define BOOT_BTN_GPIO GPIO_NUM_0

/* Core 1 실시간 모션 태스크 (placeholder).
 * 지금은 주기만 증명한다. Phase 1에서 모터 UART 키프레임 송출로 대체. */
static void motion_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(20);   /* 50Hz 키프레임 주기(로드맵 11장) */
    TickType_t last = xTaskGetTickCount();
    uint32_t tick = 0;
    while (1) {
        /* TODO(Phase1): 현재 목표값 → 모터보드 UART 키프레임 송출 + 상태 수신 */
        if ((tick % 250) == 0) {   /* 5초마다 한 번만 로깅(BLE 로그 가독성) */
            ESP_LOGI(TAG, "[core%d] motion tick %lu", xPortGetCoreID(), (unsigned long)tick);
        }
        tick++;
        vTaskDelayUntil(&last, period);
    }
}

/* BOOT 버튼 감시: 눌림(하강 에지)을 디바운스로 감지해 BLE 광고를 시작한다. */
static void button_task(void *arg)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BOOT_BTN_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_LOGI(TAG, "BOOT button watch on GPIO %d", BOOT_BTN_GPIO);

    int prev = 1;
    while (1) {
        int level = gpio_get_level(BOOT_BTN_GPIO);
        if (prev == 1 && level == 0) {          /* 하강 에지 = 눌림 시작 */
            vTaskDelay(pdMS_TO_TICKS(20));       /* 디바운스 */
            if (gpio_get_level(BOOT_BTN_GPIO) == 0) {
                ESP_LOGI(TAG, "BOOT pressed -> start BLE advertising");
                ble_rcube_start_advertising();
                /* 눌린 동안 대기(연속 트리거 방지) */
                while (gpio_get_level(BOOT_BTN_GPIO) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
            }
        }
        prev = level;
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

void app_main(void)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);

    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);

    ESP_LOGI(TAG, "==== R-Cube firmware boot ====");
    ESP_LOGI(TAG, "chip: ESP32-S3, cores=%d, rev=%d", chip.cores, chip.revision);
    ESP_LOGI(TAG, "flash: %lu MB", (unsigned long)(flash_size / (1024 * 1024)));

    /* NVS: BLE(PHY 캘리브레이션 등)에서 필요. */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 영구 설정 로드(그룹번호/노드번호). 없으면 공장 디폴트로 채워 저장. */
    rcube_config_init();
    ESP_LOGI(TAG, "identity: group=0x%02x, node=0x%02x",
             rcube_config_group_id(), rcube_config_node_id());

    /* LED 준비 후 부팅 성공 표시 → 파란색 점등. */
    board_led_init();
    board_led_set(0, 0, 255);   /* blue (밝기는 board_led에서 스케일) */
    ESP_LOGI(TAG, "boot OK -> solid blue");

    /* BLE 스택 초기화(광고는 버튼을 눌러야 시작). */
    ble_rcube_init();

    /* 모션 태스크를 Core 1에 명시 핀닝 (로드맵 12.2). */
    BaseType_t motion_ret = xTaskCreatePinnedToCore(motion_task, "motion", 4096, NULL,
                                                    configMAX_PRIORITIES - 2, NULL, 1 /* core 1 */);
    /* 버튼 감시 태스크는 Core 0(통신측). */
    BaseType_t btn_ret = xTaskCreatePinnedToCore(button_task, "button", 3072, NULL,
                                                 5, NULL, 0 /* core 0 */);

    if (motion_ret != pdPASS || btn_ret != pdPASS) {
        ESP_LOGE(TAG, "task creation failed: motion=%d button=%d",
                 (int)motion_ret, (int)btn_ret);
    } else {
        ESP_LOGI(TAG, "tasks created successfully");
    }

    ESP_LOGI(TAG, "boot done. Press BOOT button to start BLE advertising as \"RCUBE00.00\".");
}
