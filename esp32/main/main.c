/*
 * R큐브 메인보드 펌웨어 — app_main (Phase 0 스켈레톤)
 * ------------------------------------------------------------------
 * 이 파일은 "빌드가 도는 최소 골격"이다. 부팅 로그를 찍고,
 * 로드맵 12.2의 코어 배치를 미리 반영해 Core 1에 고정된 모션 태스크
 * 자리(placeholder)를 만든다. 실제 로직은 Phase 1부터 채운다.
 *
 * 코어 배치 (로드맵 12.2):
 *   - Core 1 : 실시간 모션(고정주기 키프레임 송출 + 센서 수집). 정확한 주기가 생명.
 *   - Core 0 : 통신(NimBLE/TWAI), 역할 레이어, 데이터테이블 시퀀서, MicroPython VM.
 *
 * 부팅 시퀀스 (로드맵 12.6) — 추후 구현 예정:
 *   1) 플래시에서 ECF/CMF/노드ID/그룹번호 (+CMF=1: 종단노드ID, ECF=1: 멤버맵/N) 읽기
 *   2) 공통 초기화(Core1 모션, 모터 UART, 센서, LED, 외부포트 자동탐지, 상태머신)
 *   3) 통신 스택 선택(CMF/멤버맵)  4) 종단 GPIO  5) ECF로 역할 레이어 분기
 *   (+ 대기모드 버튼 3초 롱프레스 → 설정 모드 분기)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"

static const char *TAG = "rcube";

/* Core 1 실시간 모션 태스크 (placeholder).
 * 지금은 주기만 증명한다. Phase 1에서 모터 UART 키프레임 송출로 대체. */
static void motion_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(20);   /* 50Hz 키프레임 주기(로드맵 11장) */
    TickType_t last = xTaskGetTickCount();
    uint32_t tick = 0;
    while (1) {
        /* TODO(Phase1): 현재 목표값 → 모터보드 UART 키프레임 송출 + 상태 수신 */
        if ((tick % 50) == 0) {
            ESP_LOGI(TAG, "[core%d] motion tick %lu", xPortGetCoreID(), (unsigned long)tick);
        }
        tick++;
        vTaskDelayUntil(&last, period);
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
    ESP_LOGI(TAG, "TODO: boot sequence 12.6 (ECF/CMF/member-map branch)");

    /* 모션 태스크를 Core 1에 명시 핀닝 (로드맵 12.2 — 기본값 충돌 주의) */
    xTaskCreatePinnedToCore(motion_task, "motion", 4096, NULL,
                            configMAX_PRIORITIES - 2, NULL, 1 /* core 1 */);

    ESP_LOGI(TAG, "boot done. (Phase 0 skeleton)");
}
