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
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

#include "board_led.h"
#include "ble_rcube.h"
#include "ble_multirole.h"
#include "rcube_config.h"
#include "rcube_buzzer.h"
#include "rcube_status.h"
#include "bmi088.h"
#include "can_transport.h"
#include "rcube_sensor.h"
#include "motor_uart.h"
#include "motion_core.h"

static const char *TAG = "rcube";

/* ESP32-S3 DevKit의 BOOT 버튼은 GPIO0에 연결(외부 풀업, 눌림=Low). */
#define BOOT_BTN_GPIO GPIO_NUM_0

/* 힙 여유 주기 로깅(12.5 MicroPython 임베딩 실측용).
 * 실시간 모션은 motion_core가 Core 1에서 전담하므로, 이 태스크는 Core 0에 둔다. */
static void heaplog_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(5000);
    TickType_t last = xTaskGetTickCount();
    while (1) {
        ESP_LOGI(TAG, "[heap] 런타임 internal free=%u KB (largest %u KB), 최저기록 %u KB",
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024),
                 (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024),
                 (unsigned)(esp_get_minimum_free_heap_size() / 1024));
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

/* ---- 메모리 실측 (MicroPython 임베딩 타당성 판단용, 12.5) -------------
 * 관심 있는 숫자는 두 가지다.
 *   free      : 남은 내부 RAM 총량
 *   largest   : 최대 "연속" 블록 — MicroPython GC heap은 한 덩어리로 잡아야 하므로
 *               총량이 아니라 이 값이 상한이다.
 * PSRAM은 sdkconfig에 CONFIG_SPIRAM이 켜져 있을 때만 잡힌다(개발보드에 없으면 0). */
static void log_heap(const char *stage)
{
    size_t free_int = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t large_int = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t free_dma = heap_caps_get_free_size(MALLOC_CAP_DMA);
    size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t large_psram = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "[heap] %-22s internal free=%u KB (largest %u KB) · DMA free=%u KB · "
                  "PSRAM free=%u KB (largest %u KB)",
             stage, (unsigned)(free_int / 1024), (unsigned)(large_int / 1024),
             (unsigned)(free_dma / 1024),
             (unsigned)(free_psram / 1024), (unsigned)(large_psram / 1024));
}

/* 설정모드 진입 롱프레스 임계(기획서 0724: 대기모드 3초). */
#define CONFIG_HOLD_MS 3000

/* 연결모드 진입 동작 (기획서 5장 [연결모드 진입 동작], 7.4-2·3).
 *
 *   ECF=1 (edge central / 리드 큐브) : 스스로 central로 시작한다. PC가 없으므로
 *     자기 광고는 하지 않고, 저장된 멤버 맵대로 BLE 멤버는 직접 스캔·연결하고
 *     CAN 멤버는 부팅/하트비트로 발견한다. 엣지 멜로디를 연주한다.
 *   ECF=0 (일반 큐브) : 연결대기 멜로디 + CMF에 따른 신호 발행
 *     (BLE면 RCUBEROBOT 광고, CAN이면 광고 없이 하트비트로 상위가 붙는다). */
static void enter_connect_mode(void)
{
    if (rcube_config_ecf() == 1) {
        rcube_buzzer_play(RCUBE_MELODY_EDGE);
        /* 노드01이므로 자기 노드ID 색(Red). 멤버 전원 연결 전까지 점멸. */
        uint8_t r, g, b;
        rcube_status_node_color(rcube_config_node_id(), &r, &g, &b);
        rcube_status_set_color(r, g, b);
        rcube_status_set_mode(RCUBE_LED_HUB_WAIT);

        bool ble = ble_multirole_start_edge();
        bool can = can_transport_start_edge();
        ESP_LOGI(TAG, "연결모드: edge central 시작 (BLE 멤버 %s / CAN 멤버 %s)",
                 ble ? "대기" : "없음", can ? "대기" : "없음");
        if (!ble && !can) {
            ESP_LOGW(TAG, "edge central인데 멤버 맵이 비어 있다 — 설정모드에서 재설정 필요");
        }
        return;
    }

    rcube_buzzer_play(RCUBE_MELODY_LINK_WAIT);
    if (ble_rcube_start_advertising()) {
        ESP_LOGI(TAG, "연결모드: BLE 광고 시작(RCUBEROBOT)");
    } else {
        ESP_LOGI(TAG, "연결모드: CAN 큐브 — 광고 없이 하트비트로 연결");
    }
}

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
                        enter_connect_mode();
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
    log_heap("0 boot(기준선)");

    /* ★ 안전 최우선(핀맵 문서 §6): 다른 무엇보다 먼저 모터 게이트를 차단으로 잡는다.
     * 부팅 중 의도치 않은 구동을 막기 위해 통신·설정보다 앞선다. */
    if (motor_uart_init() == ESP_OK) {
        int fail = motor_uart_selftest();
        if (fail) {
            ESP_LOGE(TAG, "모터 프로토콜 자체검증 실패 %d건 — 프레이밍 확인 필요", fail);
        }
    }

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
    log_heap("1 NVS+LED+부저");

    /* BLE 스택 초기화(광고는 버튼을 눌러야 시작). */
    ble_rcube_init();
    log_heap("2 NimBLE(최대8연결)");

    /* 부팅 성공: 상태 LED 태스크 기동 + 부팅음.
     * 기획서 5장 [소리 규칙]: 노드ID가 있으면 자기 멜로디(ID1 C4, ID2 D4 …)를
     * 2번, 없으면(비고정형) 디폴트 켜짐 멜로디를 연주한다. */
    rcube_status_start();
    rcube_melody_id_t boot_melody = rcube_melody_node_id(rcube_config_node_id());
    rcube_buzzer_play(boot_melody);
    ESP_LOGI(TAG, "boot OK -> LED(노드ID %s) + %s melody",
             rcube_config_node_id() ? "색 점멸" : "흰색 점멸(미할당)",
             rcube_melody(boot_melody)->name);

    /* 센서 모니터링(기획서 9장) — 전송은 상위의 B1 명령으로 시작한다. */
    rcube_sensor_init();

    /* IMU(BMI088) SPI 환경 준비 + 검증. 검출되면 주기 읽기 태스크 기동. */
    bmi088_init();
    if (bmi088_present()) {
        xTaskCreatePinnedToCore(imu_task, "imu", 3072, NULL, 5, NULL, 1 /* core 1 */);
    } else {
        ESP_LOGW(TAG, "IMU 미검출 → 읽기 태스크 생략(환경만 준비됨)");
    }

    /* 통신 서버 선택(기획서 7.4-3 ★서버선택).
     *   - 일반 큐브 : 자기 CMF가 CAN이면 CAN 전송계층을 켠다.
     *   - edge central(ECF=1) : 자기 CMF와 무관하게, 멤버 맵에 CAN 멤버가 하나라도
     *     있으면 CAN 서버를 켠다(BLE 멤버는 연결모드에서 NimBLE central로 붙는다).
     * 즉 CAN만/BLE만/둘 다가 맵에 따라 결정된다. */
    const bool is_edge = (rcube_config_ecf() == 1);
    const bool can_members = is_edge && rcube_config_has_member(RCUBE_MEMBER_CAN);
    const bool ble_members = is_edge && rcube_config_has_member(RCUBE_MEMBER_BLE);
    if (rcube_config_cmf() == 1 || can_members) {
        esp_err_t can_err = can_transport_init(rcube_config_node_id(), rcube_config_term_id());
        if (can_err == ESP_OK) {
            ESP_LOGI(TAG, "CAN transport 활성(%s)",
                     can_members ? "edge central: CAN 멤버 있음" : "CMF=CAN");
        } else {
            ESP_LOGE(TAG, "CAN transport 초기화 실패: %s", esp_err_to_name(can_err));
        }
    }
    if (is_edge) {
        ESP_LOGI(TAG, "ECF=1 edge central: 유닛 %u대, 서버 = %s%s%s",
                 rcube_config_unit_count(),
                 can_members ? "CAN" : "",
                 (can_members && ble_members) ? "+" : "",
                 ble_members ? "BLE" : "");
    }
    log_heap("3 +IMU/CAN");

    /* 실시간 모션 루프(Core 1 핀닝, 로드맵 12.2·12.3). */
    motion_core_init();

    /* 버튼 감시·힙 로깅 태스크는 Core 0(통신측). */
    BaseType_t btn_ret = xTaskCreatePinnedToCore(button_task, "button", 3072, NULL,
                                                 5, NULL, 0 /* core 0 */);
    xTaskCreatePinnedToCore(heaplog_task, "heaplog", 3072, NULL, 1, NULL, 0);

    if (btn_ret != pdPASS) {
        ESP_LOGE(TAG, "button task creation failed");
    } else {
        ESP_LOGI(TAG, "tasks created successfully");
    }

    log_heap("4 전 태스크 기동 후");
    ESP_LOGI(TAG, "[heap] ↑ '4'의 largest = MicroPython GC heap으로 뽑아낼 수 있는 상한(12.5). "
                  "PSRAM free=0이면 sdkconfig에 CONFIG_SPIRAM 미설정(개발보드에 PSRAM 없음).");

    ESP_LOGI(TAG, "boot done. 버튼 짧게=연결모드(%s), 3초 롱프레스=설정모드(RCUBECONFIG.%02u.%02u).",
             rcube_config_cmf() == 1 ? "CAN 하트비트" : "BLE 광고 RCUBEROBOT",
             rcube_config_group_id(), rcube_config_node_id());
}
