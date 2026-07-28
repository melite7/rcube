#include "motor_uart.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static const char *TAG = "motor";

/* ---- 핀맵 (docs/ESP32_핀맵_펌웨어참조_20260724.md) ---- */
#define MC_UART_PORT   UART_NUM_1
#define MC_TX_GPIO     GPIO_NUM_17
#define MC_RX_GPIO     GPIO_NUM_18
#define MC_FAULT_GPIO  GPIO_NUM_8    /* 입력. 모터보드 폴트 알림 */
#define MC_GATE_GPIO   GPIO_NUM_35   /* 출력. 안전상태=차단 */
#define MC_BOOT0_GPIO  GPIO_NUM_36   /* 출력. 모터보드 펌웨어 업데이트 패스스루 */
#define MC_NRST_GPIO   GPIO_NUM_37   /* 출력. Active-Low */

/* MC_GATE_EN의 "차단" 레벨. 회로도상 R 풀다운으로 기본 차단이므로 Low=차단.
 * 실보드 확인 후 반대면 이 값만 뒤집는다. */
#define MC_GATE_LEVEL_DISABLE 0
#define MC_GATE_LEVEL_ENABLE  1
/* MC_FAULT의 활성 레벨(Active-Low 가정). 실보드 확인 필요. */
#define MC_FAULT_ACTIVE_LEVEL 0

/* ※ 개발보드(DevKitC-1 N16R8)는 Octal PSRAM이 GPIO33~37을 함께 쓴다. 지금은
 *   CONFIG_SPIRAM이 꺼져 있어 PSRAM 컨트롤러가 이 핀을 구동하지 않으므로 충돌하지
 *   않는다. PSRAM을 켜려면 양산 모듈(N16R2, Quad)로 가야 한다. */

/* 보레이트 921600 — 2026-07-28 확정(매뉴얼에는 명시가 없다). */
#define MC_BAUD 921600

#define MC_RX_BUF   1024
#define MC_TX_BUF   512
#define FRAME_MAX   80      /* 우리가 보내는 최대 프레임(MIT 21B 데이터 + 오버헤드) */
#define RX_PAYLOAD_MAX 256  /* GET_VALUES 회신이 0x55(85B)라 넉넉히 */

static bool s_ready;
static bool s_gate_enabled;
static motor_status_t s_status;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

/* ---- CRC-16/XMODEM (매뉴얼 crc16(): init 0x0000, poly 0x1021, MSB-first) ---- */
static uint16_t crc16_xmodem(const uint8_t *d, size_t len)
{
    uint16_t c = 0x0000;
    for (size_t i = 0; i < len; i++) {
        c ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++) {
            c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
        }
    }
    return c;
}

/* ---- 프레임 조립 ---------------------------------------------------- */
/* body = [ID][data…]. out에 완성 프레임을 쓰고 길이를 반환(실패 0). */
static size_t build_frame(uint8_t *out, size_t out_cap, const uint8_t *body, size_t body_len)
{
    if (body_len == 0 || body_len > 255 || out_cap < body_len + 5) {
        return 0;
    }
    size_t i = 0;
    out[i++] = 0xAA;
    out[i++] = (uint8_t)body_len;
    memcpy(out + i, body, body_len);
    i += body_len;
    uint16_t crc = crc16_xmodem(body, body_len);
    out[i++] = (uint8_t)(crc >> 8);
    out[i++] = (uint8_t)(crc & 0xFF);
    out[i++] = 0xBB;
    return i;
}

static esp_err_t send_body(const uint8_t *body, size_t body_len)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t f[FRAME_MAX];
    size_t n = build_frame(f, sizeof(f), body, body_len);
    if (n == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    int w = uart_write_bytes(MC_UART_PORT, f, n);
    return (w == (int)n) ? ESP_OK : ESP_FAIL;
}

static void put_i32(uint8_t *p, int32_t v)
{
    p[0] = (uint8_t)((v >> 24) & 0xFF);
    p[1] = (uint8_t)((v >> 16) & 0xFF);
    p[2] = (uint8_t)((v >> 8) & 0xFF);
    p[3] = (uint8_t)(v & 0xFF);
}

static int32_t get_i32(const uint8_t *p)
{
    return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                     ((uint32_t)p[2] << 8) | p[3]);
}

static int16_t get_i16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

/* ---- 하행 명령 ------------------------------------------------------ */

esp_err_t motor_uart_set_pos_spd(float deg, int32_t erpm, int32_t acc_erpm)
{
    uint8_t b[13];
    b[0] = MOTOR_CMD_SET_POS_SPD;
    put_i32(&b[1], (int32_t)(deg * 1000.0f));   /* ★ 도 ×1,000 (SET_POS와 다름) */
    put_i32(&b[5], erpm);
    put_i32(&b[9], acc_erpm);
    return send_body(b, sizeof(b));
}

esp_err_t motor_uart_set_pos(float deg)
{
    uint8_t b[5];
    b[0] = MOTOR_CMD_SET_POS;
    put_i32(&b[1], (int32_t)(deg * 1000000.0f)); /* ★ 도 ×1,000,000 */
    return send_body(b, sizeof(b));
}

esp_err_t motor_uart_set_rpm(int32_t erpm)
{
    uint8_t b[5];
    b[0] = MOTOR_CMD_SET_RPM;
    put_i32(&b[1], erpm);
    return send_body(b, sizeof(b));
}

esp_err_t motor_uart_set_current(float amps)
{
    uint8_t b[5];
    b[0] = MOTOR_CMD_SET_CURRENT;
    put_i32(&b[1], (int32_t)(amps * 1000.0f));
    return send_body(b, sizeof(b));
}

esp_err_t motor_uart_set_duty(float duty)
{
    uint8_t b[5];
    b[0] = MOTOR_CMD_SET_DUTY;
    put_i32(&b[1], (int32_t)(duty * 100000.0f));
    return send_body(b, sizeof(b));
}

esp_err_t motor_uart_set_origin(uint8_t mode)
{
    uint8_t b[2] = { MOTOR_CMD_SET_POS_ORIGIN, mode };
    return send_body(b, sizeof(b));
}

esp_err_t motor_uart_request_status(void)
{
    uint8_t b[1] = { MOTOR_CMD_GET_VALUES };
    return send_body(b, sizeof(b));
}

esp_err_t motor_uart_set_detect(bool on)
{
    /* 매뉴얼 예제: AA 02 4C 04 … — [ID][mode]. 0=중지, 4=주기 전송. */
    uint8_t b[2] = { MOTOR_CMD_SET_DETECT, (uint8_t)(on ? 0x04 : 0x00) };
    return send_body(b, sizeof(b));
}

esp_err_t motor_uart_safe_stop(void)
{
    esp_err_t e = motor_uart_set_current(0.0f);
    motor_uart_set_gate(false);
    return e;
}

/* ---- 게이트/폴트 GPIO ----------------------------------------------- */

static void gpio_out_init(gpio_num_t pin, int level)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(pin, level);
}

void motor_uart_set_gate(bool enable)
{
    gpio_set_level(MC_GATE_GPIO, enable ? MC_GATE_LEVEL_ENABLE : MC_GATE_LEVEL_DISABLE);
    if (s_gate_enabled != enable) {
        ESP_LOGW(TAG, "모터 게이트 %s", enable ? "인에이블" : "차단(안전상태)");
    }
    s_gate_enabled = enable;
}

bool motor_uart_gate_enabled(void) { return s_gate_enabled; }

bool motor_uart_fault_asserted(void)
{
    return gpio_get_level(MC_FAULT_GPIO) == MC_FAULT_ACTIVE_LEVEL;
}

/* ---- 상행 수신 ------------------------------------------------------ */

/* GET_VALUES(0x45) 회신 파싱. 매뉴얼 4.3.2.1의 필드 순서를 따른다.
 * 회신이 길고(0x55) 예약 필드가 많아, 쓰는 값만 오프셋으로 뽑는다. */
static void parse_get_values(const uint8_t *p, size_t len)
{
    /* [0]=ID(0x45) 다음부터:
     *   +1  mos temp   int16 /10
     *   +3  motor temp int16 /10
     *   +5  output cur int32 /100
     *   +9  input cur  int32 /100
     *   +13 Id cur     int32 /100
     *   +17 Iq cur     int32 /100
     *   +21 duty       int16 /1000
     *   +23 erpm       int32
     *   +27 input volt int16 /10
     *   +29 예약 24B
     *   +53 status code 1B
     *   +54 outer loop position float(4B, IEEE754 BE) */
    if (len < 58) {
        return;
    }
    float pos_deg = 0.0f;
    uint32_t raw = ((uint32_t)p[54] << 24) | ((uint32_t)p[55] << 16) |
                   ((uint32_t)p[56] << 8) | p[57];
    memcpy(&pos_deg, &raw, sizeof(pos_deg));

    portENTER_CRITICAL(&s_lock);
    s_status.valid = true;
    s_status.updated_us = esp_timer_get_time();
    s_status.mos_temp_c = get_i16(&p[1]) / 10.0f;
    s_status.motor_temp_c = get_i16(&p[3]) / 10.0f;
    s_status.current_a = get_i32(&p[5]) / 100.0f;
    s_status.erpm = get_i32(&p[23]);
    s_status.input_voltage_v = get_i16(&p[27]) / 10.0f;
    s_status.error_code = p[53];
    s_status.position_deg = pos_deg;
    portEXIT_CRITICAL(&s_lock);
}

/* ROTOR_POSITION(0x57) 회신: [ID][float 4B]. SET_DETECT로 10ms마다 온다. */
static void parse_rotor_position(const uint8_t *p, size_t len)
{
    if (len < 5) {
        return;
    }
    float pos_deg = 0.0f;
    uint32_t raw = ((uint32_t)p[1] << 24) | ((uint32_t)p[2] << 16) |
                   ((uint32_t)p[3] << 8) | p[4];
    memcpy(&pos_deg, &raw, sizeof(pos_deg));

    portENTER_CRITICAL(&s_lock);
    s_status.valid = true;
    s_status.updated_us = esp_timer_get_time();
    s_status.position_deg = pos_deg;
    portEXIT_CRITICAL(&s_lock);
}

static void dispatch_frame(const uint8_t *body, size_t len)
{
    switch (body[0]) {
    case MOTOR_CMD_GET_VALUES:     parse_get_values(body, len); break;
    case MOTOR_CMD_ROTOR_POSITION: parse_rotor_position(body, len); break;
    default:
        ESP_LOGD(TAG, "미처리 회신 ID=0x%02x len=%u", body[0], (unsigned)len);
        break;
    }
}

/* 바이트 스트림 상태머신. 헤더/길이/CRC/테일을 모두 검사한 뒤에만 디스패치한다. */
typedef enum { W_HEADER = 0, W_LEN, W_BODY, W_CRC_HI, W_CRC_LO, W_TAIL } rx_state_t;

static rx_state_t s_rx_state;
static uint8_t s_rx_body[RX_PAYLOAD_MAX];
static size_t s_rx_len, s_rx_got;
static uint16_t s_rx_crc;

static void rx_feed(uint8_t b)
{
    switch (s_rx_state) {
    case W_HEADER:
        if (b == 0xAA) s_rx_state = W_LEN;
        break;
    case W_LEN:
        /* 0xAA 프레임은 LEN이 1바이트라 최대 255 — RX_PAYLOAD_MAX(256) 안에 항상 들어간다.
         * (매뉴얼의 0xAB 롱 프레임(2바이트 길이)은 우리가 쓰는 명령 범위 밖이라 미지원) */
        if (b == 0) { s_rx_state = W_HEADER; break; }
        s_rx_len = b; s_rx_got = 0; s_rx_state = W_BODY;
        break;
    case W_BODY:
        s_rx_body[s_rx_got++] = b;
        if (s_rx_got >= s_rx_len) s_rx_state = W_CRC_HI;
        break;
    case W_CRC_HI:
        s_rx_crc = (uint16_t)b << 8; s_rx_state = W_CRC_LO;
        break;
    case W_CRC_LO:
        s_rx_crc |= b; s_rx_state = W_TAIL;
        break;
    case W_TAIL:
        if (b == 0xBB && crc16_xmodem(s_rx_body, s_rx_len) == s_rx_crc) {
            dispatch_frame(s_rx_body, s_rx_len);
        } else if (b != 0xBB) {
            ESP_LOGW(TAG, "프레임 테일 불일치(0x%02x)", b);
        } else {
            ESP_LOGW(TAG, "CRC 불일치 (len=%u)", (unsigned)s_rx_len);
        }
        s_rx_state = W_HEADER;
        break;
    }
}

static void rx_task(void *arg)
{
    uint8_t buf[128];
    while (1) {
        int n = uart_read_bytes(MC_UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(50));
        for (int i = 0; i < n; i++) {
            rx_feed(buf[i]);
        }
    }
}

void motor_uart_get_status(motor_status_t *out)
{
    if (out == NULL) return;
    portENTER_CRITICAL(&s_lock);
    *out = s_status;
    portEXIT_CRITICAL(&s_lock);
}

int64_t motor_uart_silence_ms(void)
{
    motor_status_t s;
    motor_uart_get_status(&s);
    if (!s.valid) {
        return INT64_MAX;
    }
    return (esp_timer_get_time() - s.updated_us) / 1000;
}

/* ---- 자체 검증 ------------------------------------------------------ */

/* 매뉴얼 4.4.2 예제 프레임. {설명, body(hex), 기대 CRC} */
typedef struct { const char *name; const uint8_t *body; size_t len; uint16_t crc; } vec_t;

static const uint8_t V_DUTY_P[]  = {0x46, 0x00, 0x00, 0x4E, 0x20};
static const uint8_t V_DUTY_N[]  = {0x46, 0xFF, 0xFF, 0xB1, 0xE0};
static const uint8_t V_CUR_P[]   = {0x47, 0x00, 0x00, 0x13, 0x88};
static const uint8_t V_CUR_N[]   = {0x47, 0xFF, 0xFF, 0xEC, 0x78};
static const uint8_t V_BRAKE[]   = {0x48, 0x00, 0x00, 0x13, 0x88};
static const uint8_t V_RPM_P[]   = {0x49, 0x00, 0x00, 0x03, 0xE8};
static const uint8_t V_RPM_N[]   = {0x49, 0xFF, 0xFF, 0xFC, 0x18};
static const uint8_t V_POS180[]  = {0x4A, 0x0A, 0xBA, 0x95, 0x00};
static const uint8_t V_POS90[]   = {0x4A, 0x05, 0x5D, 0x4A, 0x80};
static const uint8_t V_POSSPD[]  = {0x3C, 0x00, 0x02, 0xBF, 0x20, 0x00, 0x00, 0x13, 0x88,
                                    0x00, 0x00, 0x75, 0x30};
static const uint8_t V_GETVAL[]  = {0x45};
static const uint8_t V_GETPOS[]  = {0x4C, 0x04};

#define VEC(a, c) { #a, a, sizeof(a), c }
static const vec_t VECTORS[] = {
    VEC(V_DUTY_P, 0xD64C), VEC(V_DUTY_N, 0x883F),
    VEC(V_CUR_P, 0x301C),  VEC(V_CUR_N, 0x583C),
    VEC(V_BRAKE, 0x55E5),
    VEC(V_RPM_P, 0x9061),  VEC(V_RPM_N, 0xF841),
    VEC(V_POS180, 0xE14D), VEC(V_POS90, 0x8493),
    VEC(V_POSSPD, 0x181C),
    VEC(V_GETVAL, 0x1861), VEC(V_GETPOS, 0x0825),
};

int motor_uart_selftest(void)
{
    int fail = 0;
    for (size_t i = 0; i < sizeof(VECTORS) / sizeof(VECTORS[0]); i++) {
        uint16_t c = crc16_xmodem(VECTORS[i].body, VECTORS[i].len);
        if (c != VECTORS[i].crc) {
            ESP_LOGE(TAG, "selftest CRC 불일치 %s: 계산 0x%04X ≠ 문서 0x%04X",
                     VECTORS[i].name, c, VECTORS[i].crc);
            fail++;
        }
    }
    /* 스케일 검증: 180° 명령이 문서 예제와 바이트 단위로 같아야 한다. */
    uint8_t b[13];
    b[0] = MOTOR_CMD_SET_POS;
    put_i32(&b[1], (int32_t)(180.0f * 1000000.0f));
    if (memcmp(b, V_POS180, sizeof(V_POS180)) != 0) {
        ESP_LOGE(TAG, "selftest SET_POS 스케일 불일치(도×1,000,000이어야 함)");
        fail++;
    }
    b[0] = MOTOR_CMD_SET_POS_SPD;
    put_i32(&b[1], (int32_t)(180.0f * 1000.0f));
    put_i32(&b[5], 5000);
    put_i32(&b[9], 30000);
    if (memcmp(b, V_POSSPD, sizeof(V_POSSPD)) != 0) {
        ESP_LOGE(TAG, "selftest SET_POS_SPD 스케일 불일치(도×1,000이어야 함)");
        fail++;
    }

    if (fail == 0) {
        ESP_LOGI(TAG, "selftest OK — 매뉴얼 예제 %u개 CRC + 위치 스케일 2종 일치",
                 (unsigned)(sizeof(VECTORS) / sizeof(VECTORS[0])));
    }
    return fail;
}

bool motor_uart_loopback_test(void)
{
    if (!s_ready) {
        return false;
    }
    uart_flush_input(MC_UART_PORT);
    s_rx_state = W_HEADER;

    uint8_t body[1] = { MOTOR_CMD_GET_VALUES };
    uint8_t expect[FRAME_MAX];
    size_t n = build_frame(expect, sizeof(expect), body, sizeof(body));
    uart_write_bytes(MC_UART_PORT, expect, n);

    uint8_t got[FRAME_MAX] = {0};
    int r = uart_read_bytes(MC_UART_PORT, got, n, pdMS_TO_TICKS(200));
    bool ok = (r == (int)n) && (memcmp(got, expect, n) == 0);
    ESP_LOGI(TAG, "loopback %s (보낸 %u B, 받은 %d B) — IO%d↔IO%d 점퍼 필요",
             ok ? "OK" : "실패", (unsigned)n, r, MC_TX_GPIO, MC_RX_GPIO);
    return ok;
}

/* ---- 초기화 --------------------------------------------------------- */

esp_err_t motor_uart_init(void)
{
    /* ★ 순서가 중요하다: 통신보다 먼저 게이트를 차단으로 잡아, 부팅 중 의도치 않은
     *   구동을 막는다(핀맵 문서 §6 안전상태). */
    gpio_out_init(MC_GATE_GPIO, MC_GATE_LEVEL_DISABLE);
    s_gate_enabled = false;
    gpio_out_init(MC_BOOT0_GPIO, 0);   /* 정상 부팅(플래시) */
    gpio_out_init(MC_NRST_GPIO, 1);    /* Active-Low → High=리셋 해제 */

    gpio_config_t fin = {
        .pin_bit_mask = 1ULL << MC_FAULT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&fin);

    uart_config_t cfg = {
        .baud_rate = MC_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(MC_UART_PORT, MC_RX_BUF, MC_TX_BUF, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install 실패: %s", esp_err_to_name(err));
        return err;
    }
    err = uart_param_config(MC_UART_PORT, &cfg);
    if (err == ESP_OK) {
        err = uart_set_pin(MC_UART_PORT, MC_TX_GPIO, MC_RX_GPIO,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART 설정 실패: %s", esp_err_to_name(err));
        uart_driver_delete(MC_UART_PORT);
        return err;
    }
    s_ready = true;

    xTaskCreatePinnedToCore(rx_task, "mc_rx", 4096, NULL, 8, NULL, 1 /* core 1 */);

    ESP_LOGI(TAG, "모터 UART 준비: TX%d RX%d @%d, 게이트=차단, FAULT=IO%d",
             MC_TX_GPIO, MC_RX_GPIO, MC_BAUD, MC_FAULT_GPIO);
    return ESP_OK;
}
