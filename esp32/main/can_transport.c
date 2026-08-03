#include "can_transport.h"
#include "rcube_cmd.h"
#include "rcube_config.h"
#include "rcube_status.h"
#include "rcube_buzzer.h"
#include "rcube_sensor.h"

#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#include "rcube_protocol.h"

static const char *TAG = "can";

/* ---- 핀맵 ---- */
#define CAN_TX_GPIO      GPIO_NUM_4
#define CAN_RX_GPIO      GPIO_NUM_5
#define CAN_STB_GPIO     GPIO_NUM_6    /* Low=정상, High=대기 */
#define CAN_TERM_EN_GPIO GPIO_NUM_7    /* High=120Ω 종단 ON */

#define HEARTBEAT_MS 1000

static uint8_t s_node_id;
static uint8_t s_term_id;
static bool s_running;

/* ---- edge central(ECF=1)의 CAN 멤버 대기 상태 (기획서 7.4-4) ---- */
static bool    s_edge;                       /* CAN 멤버 대기 중 */
static uint8_t s_edge_expected;              /* 기대 CAN 멤버 수 */
static uint8_t s_edge_found;                 /* 발견된 수 */
static bool    s_edge_seen[RCUBE_MAX_NODES]; /* 인덱스 = 노드ID-1 */

/* 멤버 노드ID를 발견 처리한다. 전부 모이면 완료 표시. rx_task에서만 호출. */
static void edge_note_member(uint8_t node_id)
{
    if (!s_edge || node_id < 1 || node_id > RCUBE_MAX_NODES) {
        return;
    }
    if (node_id == s_node_id) {
        return;   /* 자기 자신 */
    }
    if (rcube_config_member_cmf(node_id) != RCUBE_MEMBER_CAN) {
        return;   /* 우리 유닛의 CAN 멤버가 아님 */
    }
    if (s_edge_seen[node_id - 1]) {
        return;   /* 이미 발견 */
    }
    s_edge_seen[node_id - 1] = true;
    s_edge_found++;
    ESP_LOGI(TAG, "edge central: CAN 멤버 노드 0x%02x 발견 (%u/%u)",
             node_id, s_edge_found, s_edge_expected);

    /* 기획서 7.4-5: 각 큐브는 고정형이라 자기 노드ID 색을 스스로 켠다.
     * edge central은 확인만 하되, 색 확인 명령을 한 번 보내 표시를 맞춘다. */
    uint8_t r, g, b;
    rcube_status_node_color(node_id, &r, &g, &b);
    uint8_t led[4] = {1, r, g, b};   /* SetSK6812LED payload = [n][R,G,B] */
    can_transport_send(RCUBE_PRI_PERIPHERAL, RCUBE_OP_SetSK6812LED, node_id, led, sizeof(led));

    if (s_edge_found >= s_edge_expected) {
        ESP_LOGI(TAG, "edge central: CAN 멤버 전원 발견(%u대)", s_edge_expected);
        rcube_buzzer_play(RCUBE_MELODY_LINK_COMPLETED);
        /* 기획서 9장 [독립로봇유닛]: CAN 분기에도 센서 전송 시작을 지시한다.
         * (BLE 분기는 ble_multirole이 자기 완료 시점에 따로 지시한다.) */
        uint16_t period = RCUBE_SENSOR_PERIOD_DEFAULT_MS;
        uint8_t p[3] = {1, (uint8_t)(period >> 8), (uint8_t)(period & 0xFF)};
        can_transport_send(RCUBE_PRI_CONFIG, RCUBE_OP_SetSensorStream,
                           RCUBE_ADDR_BROADCAST, p, sizeof(p));
    } else {
        rcube_buzzer_play(RCUBE_MELODY_LINK);
    }
}

bool can_transport_start_edge(void)
{
    uint8_t me = rcube_config_node_id();
    uint8_t expected = 0;
    for (uint8_t nid = 1; nid <= RCUBE_MAX_NODES; nid++) {
        if (nid != me && rcube_config_member_cmf(nid) == RCUBE_MEMBER_CAN) {
            expected++;
        }
    }
    if (expected == 0) {
        ESP_LOGI(TAG, "edge central: CAN 멤버 없음 → CAN 서버 대기 생략");
        return false;
    }
    if (!s_running) {
        ESP_LOGE(TAG, "edge central: CAN 멤버가 있으나 TWAI 미기동 — 초기화 실패 확인");
        return false;
    }
    memset(s_edge_seen, 0, sizeof(s_edge_seen));
    s_edge_found = 0;
    s_edge_expected = expected;
    s_edge = true;
    ESP_LOGI(TAG, "edge central: CAN 멤버 %u대 대기 시작(부팅/하트비트 수신)", expected);
    return true;
}

uint8_t can_transport_edge_found(void)    { return s_edge_found; }
uint8_t can_transport_edge_expected(void) { return s_edge_expected; }

/* ---- 저수준 송신 ---------------------------------------------------- */

/* 세그먼트 1개 송신(MULTI=1). */
static esp_err_t send_segment(uint8_t priority, uint8_t op_code, uint8_t dst,
                              const uint8_t *seg, uint8_t seg_len)
{
    twai_message_t msg = {0};
    msg.identifier = RCUBE_CAN_ID(priority, op_code, 1 /*multi*/, 0 /*flag*/, s_node_id, dst);
    msg.extd = 1;
    msg.data_length_code = seg_len;
    memcpy(msg.data, seg, seg_len);
    return twai_transmit(&msg, pdMS_TO_TICKS(20));
}

/* 8바이트를 넘는 페이로드를 §5 세그먼트 규격으로 나눠 보낸다.
 *   FIRST : [hdr][전체길이 BE16][데이터 5B]
 *   이후   : [hdr][데이터 7B]
 * 한 세그먼트라도 실패하면 중단한다 — 부분 전송은 수신 측 타임아웃으로 폐기된다. */
static esp_err_t send_multiframe(uint8_t priority, uint8_t op_code, uint8_t dst,
                                 const uint8_t *data, uint16_t len)
{
    if (len > RCUBE_CAN_REASSEMBLY_MAX) {
        ESP_LOGE(TAG, "멀티프레임 길이 초과: %u > %u", len, RCUBE_CAN_REASSEMBLY_MAX);
        return ESP_ERR_INVALID_SIZE;
    }
    uint16_t sent = 0;
    uint8_t index = 0;
    uint8_t seg[8];

    /* 첫 세그먼트: 전체 길이를 함께 싣는다. */
    uint8_t chunk = (uint8_t)((len < RCUBE_CAN_SEG_FIRST_DATA) ? len : RCUBE_CAN_SEG_FIRST_DATA);
    bool last = (chunk >= len);
    seg[0] = (uint8_t)(RCUBE_CAN_SEG_FIRST | (last ? RCUBE_CAN_SEG_LAST : 0) | index);
    seg[1] = (uint8_t)((len >> 8) & 0xFF);
    seg[2] = (uint8_t)(len & 0xFF);
    memcpy(&seg[3], data, chunk);
    esp_err_t err = send_segment(priority, op_code, dst, seg, (uint8_t)(3 + chunk));
    if (err != ESP_OK) return err;
    sent += chunk;

    while (sent < len) {
        index++;
        if (index > RCUBE_CAN_SEG_MAX_INDEX) {
            return ESP_ERR_INVALID_SIZE;
        }
        uint16_t remain = (uint16_t)(len - sent);
        chunk = (uint8_t)((remain < RCUBE_CAN_SEG_DATA) ? remain : RCUBE_CAN_SEG_DATA);
        last = ((sent + chunk) >= len);
        seg[0] = (uint8_t)((last ? RCUBE_CAN_SEG_LAST : 0) | index);
        memcpy(&seg[1], data + sent, chunk);
        err = send_segment(priority, op_code, dst, seg, (uint8_t)(1 + chunk));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "멀티프레임 세그먼트 %u 송신 실패: %s", index, esp_err_to_name(err));
            return err;
        }
        sent += chunk;
    }
    return ESP_OK;
}

esp_err_t can_transport_send(uint8_t priority, uint8_t op_code,
                             uint8_t dst, const uint8_t *data, uint16_t len)
{
    if (!s_running) return ESP_ERR_INVALID_STATE;

    if (len > 8) {
        return send_multiframe(priority, op_code, dst, data, len);
    }

    twai_message_t msg = {0};
    msg.identifier = RCUBE_CAN_ID(priority, op_code, 0 /*multi*/, 0 /*flag*/, s_node_id, dst);
    msg.extd = 1;
    msg.data_length_code = (uint8_t)len;
    if (data && len) memcpy(msg.data, data, len);

    return twai_transmit(&msg, pdMS_TO_TICKS(10));
}

/* rcube_cmd 응답 콜백: 표준프레임을 CAN으로 회신(dst=마스터 0xFE).
 * 8B를 넘는 회신(GetNodeConfig·TimeSync 왕복 등)은 멀티프레임으로 나간다. */
static int can_reply(const uint8_t *frame, uint16_t len)
{
    if (len < 4) return -1;
    uint8_t op = frame[1];
    esp_err_t err = can_transport_send(RCUBE_PRI_QUERY, op, RCUBE_CAN_SRC_MASTER,
                                       frame + 4, (uint16_t)(len - 4));
    return (err == ESP_OK) ? 0 : -1;
}

/* ---- 멀티프레임 재조립 (§5) ------------------------------------------
 * (SrcId, OpCode)별로 버퍼를 하나 둔다. 순번이 어긋나거나 타임아웃이면 통째로
 * 버린다 — 부분 실행은 절대 하지 않는다. */
#define REASM_SLOTS 4

typedef struct {
    bool     active;
    uint8_t  src, op, dst;
    uint16_t total;        /* FIRST가 알려준 전체 길이 */
    uint16_t got;
    uint8_t  next_index;
    int64_t  started_us;
    uint8_t  buf[RCUBE_CAN_REASSEMBLY_MAX];
} reasm_t;

static reasm_t s_reasm[REASM_SLOTS];

static reasm_t *reasm_find(uint8_t src, uint8_t op)
{
    for (int i = 0; i < REASM_SLOTS; i++) {
        if (s_reasm[i].active && s_reasm[i].src == src && s_reasm[i].op == op) {
            return &s_reasm[i];
        }
    }
    return NULL;
}

/* 빈 슬롯을 준다. 없으면 가장 오래된 것을 회수한다(그쪽은 어차피 타임아웃 대상). */
static reasm_t *reasm_alloc(void)
{
    reasm_t *oldest = &s_reasm[0];
    for (int i = 0; i < REASM_SLOTS; i++) {
        if (!s_reasm[i].active) {
            return &s_reasm[i];
        }
        if (s_reasm[i].started_us < oldest->started_us) {
            oldest = &s_reasm[i];
        }
    }
    ESP_LOGW(TAG, "재조립 슬롯 부족 → src=0x%02x op=0x%02x 폐기", oldest->src, oldest->op);
    return oldest;
}

static void reasm_expire(void)
{
    int64_t now = esp_timer_get_time();
    for (int i = 0; i < REASM_SLOTS; i++) {
        if (s_reasm[i].active &&
            (now - s_reasm[i].started_us) > (int64_t)RCUBE_CAN_REASSEMBLY_TIMEOUT_MS * 1000) {
            ESP_LOGW(TAG, "재조립 타임아웃 폐기: src=0x%02x op=0x%02x (%u/%u B)",
                     s_reasm[i].src, s_reasm[i].op, s_reasm[i].got, s_reasm[i].total);
            s_reasm[i].active = false;
        }
    }
}

/* 세그먼트 1개 처리. 완성되면 out에 페이로드를 채우고 길이를 반환(미완성 0). */
static uint16_t reasm_feed(uint8_t src, uint8_t op, uint8_t dst,
                           const uint8_t *d, uint8_t dlc, uint8_t *out, uint16_t out_cap)
{
    if (dlc < 1) return 0;
    uint8_t hdr = d[0];
    uint8_t index = RCUBE_CAN_SEG_INDEX(hdr);
    bool first = (hdr & RCUBE_CAN_SEG_FIRST) != 0;
    bool last = (hdr & RCUBE_CAN_SEG_LAST) != 0;

    reasm_t *r = reasm_find(src, op);

    if (first) {
        if (dlc < 3) return 0;
        uint16_t total = (uint16_t)((d[1] << 8) | d[2]);
        if (total == 0 || total > RCUBE_CAN_REASSEMBLY_MAX) {
            ESP_LOGW(TAG, "FIRST 길이 이상: %u", total);
            return 0;
        }
        if (r == NULL) {
            r = reasm_alloc();
        }
        r->active = true;
        r->src = src; r->op = op; r->dst = dst;
        r->total = total;
        r->got = 0;
        r->next_index = 1;
        r->started_us = esp_timer_get_time();
        uint8_t chunk = (uint8_t)(dlc - 3);
        if (chunk > total) chunk = (uint8_t)total;
        memcpy(r->buf, &d[3], chunk);
        r->got = chunk;
    } else {
        if (r == NULL) {
            /* FIRST를 못 받았다 — 중간부터 온 조각은 버린다. */
            return 0;
        }
        if (index != r->next_index) {
            ESP_LOGW(TAG, "세그먼트 순번 불일치(기대 %u, 수신 %u) → 폐기",
                     r->next_index, index);
            r->active = false;
            return 0;
        }
        uint8_t chunk = (uint8_t)(dlc - 1);
        if (r->got + chunk > r->total) {
            chunk = (uint8_t)(r->total - r->got);
        }
        memcpy(r->buf + r->got, &d[1], chunk);
        r->got = (uint16_t)(r->got + chunk);
        r->next_index++;
    }

    if (!last) {
        return 0;
    }
    uint16_t n = r->got;
    r->active = false;
    if (n != r->total) {
        ESP_LOGW(TAG, "재조립 길이 불일치(%u ≠ %u) → 폐기", n, r->total);
        return 0;
    }
    if (n > out_cap) {
        return 0;
    }
    memcpy(out, r->buf, n);
    ESP_LOGI(TAG, "멀티프레임 재조립 완료: src=0x%02x op=0x%02x %u B", src, op, n);
    return n;
}

/* ---- 수신 → 표준프레임 재구성 → 명령 레이어 ------------------------- */
static void rx_task(void *arg)
{
    while (1) {
        twai_message_t msg;
        esp_err_t err = twai_receive(&msg, portMAX_DELAY);
        if (err != ESP_OK || !msg.extd) {
            continue;
        }
        uint32_t id = msg.identifier;
        uint8_t op  = RCUBE_CAN_OPCODE(id);
        uint8_t dst = RCUBE_CAN_DST(id);
        uint8_t src = RCUBE_CAN_SRC(id);

        /* edge central: 멤버의 부팅 알림/하트비트로 존재를 확인한다(7.4-4).
         * 이 두 프레임은 브로드캐스트로 오므로 아래 대상 필터보다 먼저 본다. */
        if (s_edge && (op == RCUBE_OP_NodeAnnounce || op == RCUBE_OP_Heartbeat)) {
            uint8_t nid = (msg.data_length_code >= 1) ? msg.data[0] : src;
            edge_note_member(nid);
            continue;
        }

        /* 나(node_id)/허브(0xFE)/브로드캐스트(0xFF) 대상만 처리. */
        if (dst != s_node_id && dst != RCUBE_ADDR_HUB && dst != RCUBE_ADDR_BROADCAST) {
            continue;
        }

        reasm_expire();   /* 끊긴 재조립을 먼저 정리 */

        /* 표준프레임 [target][op][size BE][data...] 로 재구성해 공용 디스패처로. */
        static uint8_t frame[4 + RCUBE_CAN_REASSEMBLY_MAX];
        uint8_t dlc = msg.data_length_code > 8 ? 8 : msg.data_length_code;
        uint16_t plen;

        if (RCUBE_CAN_MULTI(id)) {
            /* MULTI=1 → 세그먼트. 다 모여야 디스패치한다(§5). */
            plen = reasm_feed(src, op, dst, msg.data, dlc,
                              frame + 4, RCUBE_CAN_REASSEMBLY_MAX);
            if (plen == 0) {
                continue;   /* 아직 미완성이거나 폐기됨 */
            }
        } else {
            plen = dlc;
            if (dlc) memcpy(frame + 4, msg.data, dlc);
        }

        uint16_t total = (uint16_t)(4 + plen);
        /* CAN은 DstId에 노드ID를 그대로 싣는다. 명령 레이어는 "나에게 온 것"을 0xFE로
         * 보므로(BLE 중계와 동일한 규약), 내 노드ID로 온 프레임은 여기서 0xFE로
         * 정규화한다. 이렇게 하면 가상ID/저장 노드ID가 겹쳐도 오인이 없다. */
        frame[0] = (dst == s_node_id) ? RCUBE_ADDR_HUB : dst;
        frame[1] = op;
        frame[2] = (uint8_t)((total >> 8) & 0xFF);
        frame[3] = (uint8_t)(total & 0xFF);
        dlc = (uint8_t)plen;
        ESP_LOGI(TAG, "RX op=0x%02x src=0x%02x dst=0x%02x dlc=%u",
                 op, RCUBE_CAN_SRC(id), dst, dlc);
        rcube_cmd_on_frame(frame, total);
    }
}

/* ---- 버스 상태 감시/복구 --------------------------------------------
 * TWAI는 송신 에러가 쌓이면(TEC≥256) BUS-OFF로 떨어지고, 그 뒤의 twai_transmit는
 * 전부 ESP_ERR_INVALID_STATE로 거부된다. 드라이버는 스스로 복귀하지 않으므로
 * (twai_initiate_recovery → 버스가 조용해지면 STOPPED → twai_start) 여기서 되살린다.
 *
 * 이게 없으면 "큐브를 먼저 켜고 PC/PCAN을 나중에 연결" 하는 흔한 순서에서
 * 첫 프레임이 ACK를 못 받아 즉시 BUS-OFF → 재부팅 전까지 영영 조용해진다. */
static void bus_watch(void)
{
    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK) {
        return;
    }
    switch (st.state) {
    case TWAI_STATE_BUS_OFF:
        ESP_LOGW(TAG, "BUS-OFF (tx_err=%"PRIu32" bus_err=%"PRIu32" tx_failed=%"PRIu32
                      " arb_lost=%"PRIu32") → 복구 시작",
                 st.tx_error_counter, st.bus_error_count, st.tx_failed_count, st.arb_lost_count);
        twai_initiate_recovery();
        break;
    case TWAI_STATE_STOPPED:
        if (twai_start() == ESP_OK) {
            ESP_LOGI(TAG, "버스 복구 완료 — 송신 재개");
        }
        break;
    default:
        break;
    }
}

/* ---- 하트비트/부팅 알림 --------------------------------------------- */
static void heartbeat_task(void *arg)
{
    /* 부팅 알림(NodeAnnounce, CAN-Discovery). data[0]=node_id. */
    uint8_t nd = s_node_id;
    can_transport_send(RCUBE_PRI_SAFETY_SYNC, RCUBE_OP_NodeAnnounce, RCUBE_ADDR_BROADCAST, &nd, 1);

    TickType_t last = xTaskGetTickCount();
    uint32_t fail = 0;
    uint32_t tick = 0;
    while (1) {
        bus_watch();   /* BUS-OFF면 복구를 걸고, 복구가 끝났으면 다시 start */

        esp_err_t err = can_transport_send(RCUBE_PRI_SAFETY_SYNC, RCUBE_OP_Heartbeat,
                                           RCUBE_ADDR_BROADCAST, &nd, 1);
        if (err != ESP_OK) {
            if ((fail++ % 10) == 0) {
                ESP_LOGW(TAG, "heartbeat TX 실패(%s) — 트랜시버/버스 확인(개발보드는 정상)",
                         esp_err_to_name(err));
            }
        } else if (fail != 0) {
            ESP_LOGI(TAG, "heartbeat TX 재개(실패 %"PRIu32"회 후)", fail);
            fail = 0;
        }

        /* 진단: 10초마다 카운터를 남긴다(정상이면 전부 0). */
        if ((tick++ % 10) == 0) {
            twai_status_info_t st;
            if (twai_get_status_info(&st) == ESP_OK) {
                ESP_LOGI(TAG, "bus state=%d tx_err=%"PRIu32" rx_err=%"PRIu32" bus_err=%"PRIu32
                              " tx_failed=%"PRIu32" arb_lost=%"PRIu32" rx=%"PRIu32,
                         (int)st.state, st.tx_error_counter, st.rx_error_counter,
                         st.bus_error_count, st.tx_failed_count, st.arb_lost_count,
                         st.msgs_to_rx);
            }
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(HEARTBEAT_MS));
    }
}

/* ---- 초기화 --------------------------------------------------------- */
static void gpio_out(gpio_num_t pin, int level)
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

/* ---- 부팅 자가시험(진단용) ------------------------------------------
 * NO_ACK 모드로 몇 프레임 쏴 본다. 이 모드는 상대의 ACK를 요구하지 않으므로,
 * "버스에 아무도 없어서" 실패하는 경우와 "ESP↔트랜시버 구간이 잘못돼서" 실패하는
 * 경우를 갈라준다(비트 되읽기 검사는 그대로 동작한다).
 *   · 성공(state=RUNNING, bus_err=0) → TX/RX/트랜시버 정상. 앞선 BUS-OFF는 ACK 부재.
 *     이때 프레임은 실제 버스로 나가므로 PCAN-View에도 보여야 한다.
 *   · 실패(bus_err 급증)            → ESP↔트랜시버 구간 문제(TX/RX 교차, STB, VIO/전원,
 *                                      또는 트랜시버 미실장).
 * 배선을 의심할 상황이 오면 1로 켜서 부팅 로그만 보면 된다(평소엔 0).
 *
 * 2026-08-03 실사용 사례: 트랜시버 모듈(MCP2562FD)에 VIO(3.3V)만 주고 VDD(5V)를
 * 빠뜨렸을 때, 이 시험이 "RXD는 구동됨 + 자기 dominant 되읽기 실패"로 곧장 짚어냈다. */
#define CAN_SELFTEST_ON_BOOT 0

#if CAN_SELFTEST_ON_BOOT
/* 핀 레벨 루프백: TWAI 컨트롤러를 빼고 IO4(TXD)를 손으로 흔들어 IO5(RXD)를 읽는다.
 * 트랜시버에 전원이 들어와 있고 CANH/CANL이 살아 있으면, TXD=0(dominant)을 내리는
 * 순간 자기 자신이 만든 dominant를 RXD로 되받아야 한다(전파지연 수백 ns).
 * ※ dominant 유지는 트랜시버의 TXD dominant timeout(수백 µs) 아래로 짧게만 한다. */
static void can_pin_loopback_test(void)
{
    gpio_config_t tx = {
        .pin_bit_mask = 1ULL << CAN_TX_GPIO, .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config_t rx = {
        .pin_bit_mask = 1ULL << CAN_RX_GPIO, .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&tx);
    gpio_config(&rx);

    /* (a) RXD 선 자체가 "구동되는 출력"에 붙어 있는지 본다.
     *     내부 풀다운을 걸어도 1로 읽히면 = 누군가(트랜시버 RXD)가 High로 밀고 있다.
     *     풀다운에 끌려 0이 되면 = 그 핀은 떠 있다(미결선/무전원 하이임피던스). */
    gpio_set_level(CAN_TX_GPIO, 1);
    gpio_set_pull_mode(CAN_RX_GPIO, GPIO_PULLDOWN_ONLY);
    esp_rom_delay_us(500);
    int pd = gpio_get_level(CAN_RX_GPIO);
    gpio_set_pull_mode(CAN_RX_GPIO, GPIO_PULLUP_ONLY);
    esp_rom_delay_us(500);
    int pu = gpio_get_level(CAN_RX_GPIO);
    gpio_set_pull_mode(CAN_RX_GPIO, GPIO_FLOATING);

    /* (b) STB 두 극성 모두에서 루프백을 시도한다. 모듈의 그 핀이 STBY(High=대기)가
     *     아니라 EN(High=동작)이면, 우리가 Low로 잡는 지금이 오히려 "꺼짐"이다. */
    int rec[2], dom[2];
    for (int stb = 0; stb <= 1; stb++) {
        gpio_set_level(CAN_STB_GPIO, stb);
        esp_rom_delay_us(2000);              /* 모드 전환 시간 */
        gpio_set_level(CAN_TX_GPIO, 1);      /* recessive */
        esp_rom_delay_us(500);
        rec[stb] = gpio_get_level(CAN_RX_GPIO);
        gpio_set_level(CAN_TX_GPIO, 0);      /* dominant (TXD dominant timeout 아래로 짧게) */
        esp_rom_delay_us(200);
        dom[stb] = gpio_get_level(CAN_RX_GPIO);
        gpio_set_level(CAN_TX_GPIO, 1);      /* 반드시 recessive로 복귀 */
    }
    gpio_set_level(CAN_STB_GPIO, 0);         /* 설계값(Low=정상)으로 복귀 */

    ESP_LOGW(TAG, "[핀 루프백] RXD 선 상태: 풀다운=%d 풀업=%d %s", pd, pu,
             pd == 1 ? "(구동되는 출력에 연결됨)" : "(떠 있음 — 미결선/무전원 의심)");
    ESP_LOGW(TAG, "[핀 루프백] STB=0: TXD1→RXD%d, TXD0→RXD%d | STB=1: TXD1→RXD%d, TXD0→RXD%d",
             rec[0], dom[0], rec[1], dom[1]);

    const char *verdict;
    if (rec[0] == 1 && dom[0] == 0) {
        verdict = "정상 — 트랜시버 전원·루프백 OK (STB=Low가 맞음)";
    } else if (rec[1] == 1 && dom[1] == 0) {
        verdict = "★ STB=High에서만 루프백 성공 → 그 핀은 STBY가 아니라 EN(High=동작)이다. "
                  "펌웨어의 STB 극성을 뒤집어야 한다";
    } else if (pd == 0) {
        verdict = "RXD가 떠 있다 → 트랜시버 무전원(VCC/GND) 또는 RXD 배선 끊김/미실장";
    } else if (rec[0] == 0 && dom[0] == 0) {
        verdict = "RXD가 항상 Low(dominant 고착) → CANH-CANL 단락 · 다른 노드 고착 의심";
    } else {
        verdict = "RXD는 구동되는데 자기 dominant가 안 돌아옴 "
                  "→ 트랜시버가 송신 불가 상태(대기모드/VCC 부족) 또는 TXD 배선 끊김";
    }
    ESP_LOGW(TAG, "[핀 루프백] 판정: %s", verdict);

    /* (c) 배선 교차 검사 — 역할을 뒤집어 본다.
     *     IO4를 입력으로 두고 재보면, 거기에 "구동되는 출력"이 붙어 있다는 건
     *     IO4가 트랜시버의 TXD(입력)가 아니라 RXD(출력)에 물려 있다는 뜻이다.
     *     이어서 IO5를 출력으로 dominant를 내려 IO4가 따라 내려오면 교차 확정. */
    gpio_set_direction(CAN_TX_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(CAN_TX_GPIO, GPIO_PULLDOWN_ONLY);
    esp_rom_delay_us(500);
    int tx_pd = gpio_get_level(CAN_TX_GPIO);
    gpio_set_pull_mode(CAN_TX_GPIO, GPIO_FLOATING);

    gpio_set_direction(CAN_RX_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CAN_RX_GPIO, 1);
    esp_rom_delay_us(500);
    int rev_rec = gpio_get_level(CAN_TX_GPIO);
    gpio_set_level(CAN_RX_GPIO, 0);
    esp_rom_delay_us(200);
    int rev_dom = gpio_get_level(CAN_TX_GPIO);
    gpio_set_level(CAN_RX_GPIO, 1);

    /* 원래 방향으로 되돌린다(TWAI 드라이버가 곧 핀을 다시 가져간다). */
    gpio_set_direction(CAN_RX_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(CAN_TX_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CAN_TX_GPIO, 1);

    ESP_LOGW(TAG, "[역방향] IO4 풀다운 읽기=%d · IO5출력 1→IO4=%d, IO5출력 0→IO4=%d%s",
             tx_pd, rev_rec, rev_dom,
             (rev_rec == 1 && rev_dom == 0)
                 ? "  ★ 역방향 루프백 성공 = TXD/RXD 교차 배선(IO4↔RXD, IO5↔TXD)"
                 : "");
}

static void can_selftest(void)
{
    can_pin_loopback_test();

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO,
                                                          TWAI_MODE_NO_ACK);
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        ESP_LOGE(TAG, "[자가시험] 드라이버 설치 실패");
        return;
    }
    if (twai_start() != ESP_OK) {
        ESP_LOGE(TAG, "[자가시험] start 실패");
        twai_driver_uninstall();
        return;
    }
    twai_message_t m = {0};
    m.identifier = RCUBE_CAN_ID(RCUBE_PRI_SAFETY_SYNC, RCUBE_OP_NodeAnnounce,
                                0 /*multi*/, 0 /*flag*/, s_node_id, RCUBE_ADDR_BROADCAST);
    m.extd = 1;
    m.data_length_code = 1;
    m.data[0] = s_node_id;
    esp_err_t tx = ESP_OK;
    for (int i = 0; i < 3 && tx == ESP_OK; i++) {
        tx = twai_transmit(&m, pdMS_TO_TICKS(50));
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    twai_status_info_t st = {0};
    twai_get_status_info(&st);
    ESP_LOGW(TAG, "[자가시험 NO_ACK] tx=%s state=%d tx_err=%"PRIu32" bus_err=%"PRIu32
                  " tx_failed=%"PRIu32" arb_lost=%"PRIu32" (0x%08"PRIX32" ×3 송신 시도)",
             esp_err_to_name(tx), (int)st.state, st.tx_error_counter, st.bus_error_count,
             st.tx_failed_count, st.arb_lost_count, m.identifier);
    ESP_LOGW(TAG, "[자가시험] 판정: %s",
             (st.state == TWAI_STATE_RUNNING && st.bus_error_count == 0)
                 ? "TX/RX·트랜시버 정상 → 앞선 BUS-OFF는 버스에 ACK할 상대가 없었던 것"
                 : "ESP↔트랜시버 구간 이상(TX/RX 교차·STB·전원·미실장 확인)");
    twai_stop();
    twai_driver_uninstall();
}
#endif

esp_err_t can_transport_init(uint8_t node_id, uint8_t term_id)
{
    s_node_id = node_id;
    s_term_id = term_id;

    /* 트랜시버 정상모드(STB=Low). */
    gpio_out(CAN_STB_GPIO, 0);
    /* 자가종단: 이 큐브가 CAN 버스 끝단(최대 CAN 노드ID == term_id)이면 종단 ON. */
    int terminate = (term_id != 0 && node_id == term_id) ? 1 : 0;
    gpio_out(CAN_TERM_EN_GPIO, terminate);
    ESP_LOGI(TAG, "STB=정상(Low), TERM_EN=%s (node=0x%02x, term=0x%02x)",
             terminate ? "ON(120Ω)" : "OFF", node_id, term_id);

#if CAN_SELFTEST_ON_BOOT
    can_selftest();
#endif

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    g.tx_queue_len = 5;
    g.rx_queue_len = 10;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g, &t, &f);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_driver_install 실패: %s", esp_err_to_name(err));
        return err;
    }
    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_start 실패: %s", esp_err_to_name(err));
        twai_driver_uninstall();
        return err;
    }
    s_running = true;

    /* 명령 회신을 CAN으로 보내도록 응답 콜백 교체(이 큐브는 CAN 경로). */
    rcube_cmd_override_send(can_reply);

    xTaskCreatePinnedToCore(rx_task, "can_rx", 4096, NULL, 9, NULL, 0);
    xTaskCreatePinnedToCore(heartbeat_task, "can_hb", 3072, NULL, 5, NULL, 0);

    ESP_LOGI(TAG, "CAN transport ready (500kbit, TX%d RX%d, node=0x%02x)",
             CAN_TX_GPIO, CAN_RX_GPIO, node_id);
    return ESP_OK;
}
