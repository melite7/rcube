#include "motion_core.h"
#include "motor_uart.h"
#include "rcube_cmd.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "rcube_protocol.h"

static const char *TAG = "motion";

#define HEADER_LEN 4

/* ---- 상태 (뮤텍스 보호. Core 0에서 push/execute, Core 1에서 소비) ---- */
static SemaphoreHandle_t s_lock;

static motion_waypoint_t s_queue[MOTION_QUEUE_DEPTH];
static uint16_t s_head, s_tail, s_count;

static bool     s_running;          /* Run 트리거를 받아 실행 중 */
static uint64_t s_t0_master_us;     /* 실행 기준 시각(마스터 시계) */
static int64_t  s_time_offset_us;   /* 마스터시각 = 로컬 + offset */
static uint16_t s_period_ms = MOTION_PERIOD_DEFAULT_MS;
static int64_t  s_last_push_us;     /* 굶주림 감지용 */
static bool     s_starved;          /* 이미 굶주림 정지를 알렸는가 */
static int32_t  s_last_sent_pos;    /* 마지막으로 내려보낸 목표(감속 유지용) */
static bool     s_have_sent;

/* ---- 시계 ------------------------------------------------------------ */

void motion_core_set_time_offset(int64_t offset_us)
{
    s_time_offset_us = offset_us;
    ESP_LOGI(TAG, "시계 오프셋 %lld us 적용(마스터 = 로컬 + offset)", (long long)offset_us);
}

int64_t motion_core_master_now_us(void)
{
    return esp_timer_get_time() + s_time_offset_us;
}

/* ---- 큐 -------------------------------------------------------------- */

static void queue_clear_locked(void)
{
    s_head = s_tail = s_count = 0;
    s_running = false;
    s_t0_master_us = 0;
}

bool motion_core_push(const motion_waypoint_t *wp, bool buffered)
{
    if (wp == NULL) {
        return false;
    }
    bool ok = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_count < MOTION_QUEUE_DEPTH) {
        s_queue[s_tail] = *wp;
        s_tail = (uint16_t)((s_tail + 1) % MOTION_QUEUE_DEPTH);
        s_count++;
        s_last_push_us = esp_timer_get_time();
        s_starved = false;
        if (!buffered && !s_running) {
            /* Immediate: 트리거 없이 지금 기준으로 출발한다. */
            s_running = true;
            s_t0_master_us = (uint64_t)motion_core_master_now_us();
        }
        ok = true;
    }
    xSemaphoreGive(s_lock);
    if (!ok) {
        ESP_LOGW(TAG, "큐 가득참(%d) → 웨이포인트 버림 seq=%u", MOTION_QUEUE_DEPTH, wp->seq);
    }
    return ok;
}

bool motion_core_execute(uint8_t action, uint64_t t_start_us)
{
    bool ok = true;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    switch (action) {
    case MOTION_EXEC_RUN:
        if (s_count == 0) {
            ok = false;   /* 빈 버퍼 Run은 거부(12.9) */
            break;
        }
        s_running = true;
        s_t0_master_us = (t_start_us != 0) ? t_start_us
                                           : (uint64_t)motion_core_master_now_us();
        break;
    case MOTION_EXEC_CANCEL:
        s_running = false;
        queue_clear_locked();
        break;
    case MOTION_EXEC_FLUSH:
        /* 미실행 버퍼만 폐기. 실행 중이면 현재 목표는 유지된다. */
        s_head = s_tail = s_count = 0;
        break;
    default:
        ok = false;
        break;
    }
    xSemaphoreGive(s_lock);

    if (!ok) {
        ESP_LOGW(TAG, "ExecuteBuffer 거부: action=0x%02x (빈 버퍼이거나 알 수 없는 동작)", action);
        return false;
    }
    if (action == MOTION_EXEC_RUN) {
        int64_t delay_us = (int64_t)s_t0_master_us - motion_core_master_now_us();
        ESP_LOGI(TAG, "Run: T0까지 %lld us 대기 (대기 웨이포인트 %u개)",
                 (long long)delay_us, motion_core_pending());
    } else {
        ESP_LOGI(TAG, "ExecuteBuffer action=0x%02x 처리", action);
    }
    return true;
}

uint16_t motion_core_pending(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    uint16_t n = s_count;
    xSemaphoreGive(s_lock);
    return n;
}

bool motion_core_set_period(uint16_t period_ms)
{
    if (period_ms < MOTION_PERIOD_MIN_MS || period_ms > MOTION_PERIOD_MAX_MS) {
        return false;
    }
    s_period_ms = period_ms;
    ESP_LOGI(TAG, "키프레임 주기 %u ms", period_ms);
    return true;
}

/* ---- 완료 통보 (0xB7 MotionComplete, 확장 규격 §4) -------------------- */

static void report_complete(uint8_t reason, uint16_t seq, int32_t position, uint8_t err)
{
    uint8_t f[HEADER_LEN + 8];
    uint16_t total = sizeof(f);
    f[0] = RCUBE_ADDR_HUB;
    f[1] = RCUBE_OP_MotionComplete;
    f[2] = (uint8_t)((total >> 8) & 0xFF);
    f[3] = (uint8_t)(total & 0xFF);
    f[4] = reason;
    f[5] = (uint8_t)((seq >> 8) & 0xFF);
    f[6] = (uint8_t)(seq & 0xFF);
    f[7] = (uint8_t)((position >> 24) & 0xFF);
    f[8] = (uint8_t)((position >> 16) & 0xFF);
    f[9] = (uint8_t)((position >> 8) & 0xFF);
    f[10] = (uint8_t)(position & 0xFF);
    f[11] = err;
    rcube_cmd_send_frame(f, total);
    ESP_LOGI(TAG, "→ 0xB7 MotionComplete: reason=%u seq=%u pos=%ld err=%u",
             reason, seq, (long)position, err);
}

/* ---- 안전 정지 -------------------------------------------------------- */

void motion_core_emergency_stop(const char *reason)
{
    uint16_t seq;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    seq = s_count ? s_queue[s_head].seq : 0;
    queue_clear_locked();
    xSemaphoreGive(s_lock);

    motor_uart_safe_stop();
    ESP_LOGE(TAG, "긴급 정지: %s", reason ? reason : "(사유 없음)");
    report_complete(0x02 /* Cancel로 중단 */, seq, s_last_sent_pos, 0);
}

/* ---- 고정주기 루프 (Core 1) ------------------------------------------ */

/* 0.01°/s → ERPM 변환. 극쌍수·감속비가 확정되지 않아(확장 규격 §6-2) 지금은
 * 0을 돌려 드라이버 기본 속도를 쓰게 한다. 값이 확정되면 여기만 채우면 된다. */
static int32_t velocity_to_erpm(int16_t vel_centi_dps)
{
    (void)vel_centi_dps;
    return 0;
}

/* 드라이버에 키프레임 1개 송출. */
static void send_waypoint(const motion_waypoint_t *wp)
{
    float deg = wp->position / 100.0f;
    int32_t erpm = velocity_to_erpm(wp->velocity);
    motor_uart_set_pos_spd(deg, erpm, 0 /* 가속 자동 */);
    s_last_sent_pos = wp->position;
    s_have_sent = true;
}

/* 드라이버 감시(12.9 supervisor). 폴트면 true. */
static bool supervise(void)
{
    if (motor_uart_fault_asserted()) {
        motion_core_emergency_stop("MC_FAULT 활성");
        return true;
    }
    motor_status_t st;
    motor_uart_get_status(&st);
    if (st.valid && st.error_code != 0) {
        motor_uart_safe_stop();
        ESP_LOGE(TAG, "드라이버 에러코드 0x%02x → 정지", st.error_code);
        report_complete(0x03 /* 폴트 */, 0, s_last_sent_pos, st.error_code);
        xSemaphoreTake(s_lock, portMAX_DELAY);
        queue_clear_locked();
        xSemaphoreGive(s_lock);
        return true;
    }
    return false;
}

static void motion_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    ESP_LOGI(TAG, "모션 루프 시작(Core %d, 주기 %u ms)", xPortGetCoreID(), s_period_ms);

    while (1) {
        if (!supervise()) {
            int64_t now_master = motion_core_master_now_us();

            xSemaphoreTake(s_lock, portMAX_DELAY);
            bool running = s_running;
            bool due = false;
            motion_waypoint_t wp = {0};
            if (running && s_count > 0) {
                /* T0 + t_offset 이 지났으면 이 웨이포인트를 내보낼 때다. */
                int64_t due_at = (int64_t)s_t0_master_us + s_queue[s_head].t_offset_us;
                if (now_master >= due_at) {
                    wp = s_queue[s_head];
                    s_head = (uint16_t)((s_head + 1) % MOTION_QUEUE_DEPTH);
                    s_count--;
                    due = true;
                }
            }
            bool drained = running && (s_count == 0) && !due;
            int64_t since_push = esp_timer_get_time() - s_last_push_us;
            xSemaphoreGive(s_lock);

            if (due) {
                send_waypoint(&wp);
                if (wp.flags & MOTION_WP_FLAG_INPOS) {
                    /* 결정적 지점(집기/놓기 등): 도달을 상위에 알린다(12.9 handshake).
                     * TODO 드라이버 실제 위치와의 in-position 판정은 실보드에서 임계 확정. */
                    report_complete(0x00, wp.seq, wp.position, 0);
                }
            } else if (drained) {
                /* 버퍼를 다 실행했다. 마지막 목표는 드라이버가 유지한다. */
                xSemaphoreTake(s_lock, portMAX_DELAY);
                s_running = false;
                xSemaphoreGive(s_lock);
                report_complete(0x01 /* 버퍼 전체 완료 */, 0, s_last_sent_pos, 0);
            } else if (s_have_sent && since_push > MOTION_STARVE_TIMEOUT_MS * 1000 && !s_starved) {
                /* 상위가 목표를 못 주고 있다 → 안전 정지(12.9 상호 watchdog).
                 * 드라이버도 자체적으로 무신호 시 정지하지만 여기서 먼저 끊는다. */
                s_starved = true;
                motor_uart_safe_stop();
                ESP_LOGW(TAG, "목표 미갱신 %lld ms → 안전 정지",
                         (long long)(since_push / 1000));
                report_complete(0x03, 0, s_last_sent_pos, 0);
            }
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(s_period_ms));
    }
}

void motion_core_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    configASSERT(s_lock != NULL);
    queue_clear_locked();
    s_last_push_us = esp_timer_get_time();

    /* Core 1에 명시 핀닝 + 고우선순위. 기획서 12.2: Core 1에는 모션·센서만 둔다. */
    BaseType_t ok = xTaskCreatePinnedToCore(motion_task, "motion", 4096, NULL,
                                            configMAX_PRIORITIES - 2, NULL, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "모션 태스크 생성 실패");
    }
}
