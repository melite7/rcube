/*
 * R큐브 메인보드 펌웨어 — app_main
 * ------------------------------------------------------------------
 * 동작(기획서 5장 [버튼]·[LED]·[소리], 7.1~7.2 연결 절차):
 *   1) 전원 인가 → 설정(NVS) 로드 → 노드ID 색 1초 점멸(미할당이면 흰색)
 *      + 부팅음(노드ID 있으면 자기 멜로디, 없으면 디폴트).
 *   2) BOOT 버튼 짧게 → 연결모드. CMF=0(BLE)이면 "RCUBEROBOT.GG.NN"으로 광고하고,
 *      CMF=1(CAN)이면 광고 없이 CAN 하트비트로 상위가 붙는다.
 *   3) BOOT 버튼 3초 롱프레스 → 설정모드("RCUBECONFIG.GG.NN", 흰색 0.25초 점멸).
 *
 * 노드LED 표시는 rcube_status가 전담한다(모드=점멸/점등, 색=노드ID 또는 상위 지정).
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
#include "rcube_buzzer.h"
#include "rcube_status.h"
#include "bmi088.h"
#include "can_transport.h"

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

/* IMU(BMI088) 주기 읽기 태스크(스캐폴딩). 센서가 검출된 경우에만 기동.
 * 로드맵상 실시간 센서수집은 Core1이지만, 여기선 환경 검증용 저속 로깅(2Hz). */
static void imu_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(500);
    TickType_t last = xTaskGetTickCount();
    while (1) {
        int16_t ax, ay, az, gx, gy, gz;
        if (bmi088_read_accel_raw(&ax, &ay, &az) == ESP_OK &&
            bmi088_read_gyro_raw(&gx, &gy, &gz) == ESP_OK) {
            ESP_LOGI(TAG, "IMU acc[mg]=(%.0f,%.0f,%.0f) gyro[dps]=(%.1f,%.1f,%.1f)",
                     bmi088_accel_mg(ax), bmi088_accel_mg(ay), bmi088_accel_mg(az),
                     bmi088_gyro_dps(gx), bmi088_gyro_dps(gy), bmi088_gyro_dps(gz));
        } else {
            ESP_LOGW(TAG, "IMU 읽기 실패");
        }
        vTaskDelayUntil(&last, period);
    }
}

/* 설정모드 진입 롱프레스 임계(기획서 0724: 대기모드 3초). */
#define CONFIG_HOLD_MS 3000

/* BOOT 버튼 감시:
 *   - 짧게 누름(<3s) : 연결모드(광고 RCUBEROBOT) + 버튼음.
 *   - 길게 누름(≥3s): 설정모드 진입 — 노드LED 흰색 0.25s 점멸 + 광고 RCUBECONFIG.
 *     임계 도달 순간(누른 채) LED가 바뀌어 사용자가 놓을 시점을 안다. 놓아도 유지.
 * (개발보드 GPIO0 버튼 기준. 메인보드는 STM6601 IO48/IO2로 동일 로직.) */
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
                rcube_buzzer_play(RCUBE_MELODY_BUTTON_PRESSED);
                /* 누른 채 hold 시간 측정. 3초 도달 순간 설정모드로 전환. */
                uint32_t held = 0;
                bool config_triggered = false;
                while (gpio_get_level(BOOT_BTN_GPIO) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                    held += 20;
                    if (!config_triggered && held >= CONFIG_HOLD_MS) {
                        config_triggered = true;
                        ESP_LOGI(TAG, "BOOT held >=3s -> config mode (RCUBECONFIG)");
                        rcube_status_enter_config_mode();
                        ble_rcube_start_config_advertising();
                    }
                }
                /* 놓음. 짧게였으면 연결모드 진입. */
                if (!config_triggered) {
                    bool first = rcube_status_enter_connect_mode();
                    if (first) {
                        /* 기획서 5장 [소리 규칙]: 연결모드로 갈 때 멤버 큐브는
                         * 연결대기 멜로디를 연주한다(edge central 멜로디는 7.3에서). */
                        rcube_buzzer_play(RCUBE_MELODY_LINK_WAIT);
                        /* CMF=0(BLE)면 광고 시작, CMF=1(CAN)이면 광고 없이
                         * CAN 하트비트로 연결된다(부팅 시 이미 기동). */
                        if (ble_rcube_start_advertising()) {
                            ESP_LOGI(TAG, "BOOT short press -> connect mode: BLE 광고 시작");
                        } else {
                            ESP_LOGI(TAG, "BOOT short press -> connect mode: CAN 큐브(광고 없음)");
                        }
                    } else {
                        ESP_LOGI(TAG, "BOOT short press (already in connect/config)");
                    }
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

    /* 영구 설정 로드(그룹번호/노드번호/통신방식). 없으면 공장 디폴트로 채워 저장. */
    rcube_config_init();
    ESP_LOGI(TAG, "identity: group=0x%02x, node=0x%02x, cmf=%u(%s), term=0x%02x",
             rcube_config_group_id(), rcube_config_node_id(),
             rcube_config_cmf(), rcube_config_cmf() ? "CAN" : "BLE",
             rcube_config_term_id());

    /* LED / 부저 준비. 부팅 후 LED는 그룹번호 아이덴티티 표시가 담당한다. */
    board_led_init();
    rcube_buzzer_init();

    /* BLE 스택 초기화(광고는 버튼을 눌러야 시작). */
    ble_rcube_init();

    /* 부팅 성공: 상태 LED 태스크 기동 + 부팅음.
     * 기획서 5장 [소리 규칙]: 노드ID가 있으면 자기 멜로디(ID1 C4, ID2 D4 …)를
     * 2번, 없으면(비고정형) 디폴트 켜짐 멜로디를 연주한다. */
    rcube_status_start();
    rcube_melody_id_t boot_melody = rcube_melody_node_id(rcube_config_node_id());
    rcube_buzzer_play(boot_melody);
    ESP_LOGI(TAG, "boot OK -> LED(노드ID %s) + %s melody",
             rcube_config_node_id() ? "색 점멸" : "흰색 점멸(미할당)",
             rcube_melody(boot_melody)->name);

    /* IMU(BMI088) SPI 환경 준비 + 검증. 검출되면 주기 읽기 태스크 기동. */
    bmi088_init();
    if (bmi088_present()) {
        xTaskCreatePinnedToCore(imu_task, "imu", 3072, NULL, 5, NULL, 1 /* core 1 */);
    } else {
        ESP_LOGW(TAG, "IMU 미검출 → 읽기 태스크 생략(환경만 준비됨)");
    }

    /* 통신방식이 CAN이면 TWAI 전송계층 기동(하트비트 발행 + CAN 명령 수신). */
    if (rcube_config_cmf() == 1) {
        esp_err_t can_err = can_transport_init(rcube_config_node_id(), rcube_config_term_id());
        if (can_err == ESP_OK) {
            ESP_LOGI(TAG, "CMF=CAN → CAN transport 활성(BLE는 설정/복구용으로만)");
        } else {
            ESP_LOGE(TAG, "CAN transport 초기화 실패: %s", esp_err_to_name(can_err));
        }
    }

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

    ESP_LOGI(TAG, "boot done. 버튼 짧게=연결모드(%s), 3초 롱프레스=설정모드(RCUBECONFIG.%02u.%02u).",
             rcube_config_cmf() == 1 ? "CAN 하트비트" : "BLE 광고 RCUBEROBOT",
             rcube_config_group_id(), rcube_config_node_id());
}
