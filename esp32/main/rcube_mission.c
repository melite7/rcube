#include "rcube_mission.h"
#include "rcube_config.h"
#include "rcube_params.h"
#include "rcube_cmd.h"
#include "rcube_buzzer.h"
#include "motion_core.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_crc.h"
#include "esp_timer.h"
#include "nvs.h"

#include "rcube_protocol.h"

static const char *TAG = "mission";

#define NVS_NS        "rcube"
#define KEY_ACTIVE    "msn_slot"   /* 활성 슬롯 인덱스(0/1) */

/* 헤더 오프셋 (확장 규격 §2.5.1) */
#define OFF_MAGIC     0
#define OFF_VER       4
#define OFF_TYPE      5
#define OFF_FLAGS     6
#define OFF_NNODES    7
#define OFF_UNITSIG   8
#define OFF_BODYLEN   12
#define OFF_BODYCRC   16
#define OFF_NAME      20

static const esp_partition_t *s_part;
static uint8_t  s_active_slot;              /* 현재 유효한 슬롯 */
static uint8_t  s_hdr[RCUBE_MISSION_HDR_LEN];
static bool     s_loaded;

/* 업로드 진행 상태(비활성 슬롯에 쓴다) */
static bool     s_up_active;
static uint8_t  s_up_slot;
static uint32_t s_up_total, s_up_written, s_up_crc, s_up_unitsig;
static uint16_t s_up_next_seq;
static uint8_t  s_up_type;

/* 실행 상태 */
static TaskHandle_t s_play_task;
static volatile bool s_running, s_paused, s_stop_req;

/* ---- 바이트 도우미 (헤더는 little-endian: 플래시에 그대로 얹는 구조라
 *      와이어 BE 규약과 무관하게 호스트 표현을 쓴다) ---- */
static uint32_t rd_u32le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void wr_u32le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF); p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static uint32_t slot_offset(uint8_t slot) { return slot * RCUBE_MISSION_SLOT_SIZE; }

/* ---- 유닛 서명 ------------------------------------------------------- */

uint32_t rcube_mission_unit_sig(void)
{
    uint8_t buf[1 + RCUBE_MAX_NODES];
    buf[0] = rcube_config_unit_count();
    memcpy(&buf[1], rcube_config_member_map(), RCUBE_MAX_NODES);
    return esp_crc32_le(0, buf, sizeof(buf));
}

/* ---- 활성 슬롯 ------------------------------------------------------- */

static void store_active_slot(uint8_t slot)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_set_u8(h, KEY_ACTIVE, slot);
    nvs_commit(h);
    nvs_close(h);
}

/* 슬롯의 헤더를 읽어 유효하면 s_hdr에 싣는다. */
static bool load_slot(uint8_t slot)
{
    uint8_t hdr[RCUBE_MISSION_HDR_LEN];
    if (esp_partition_read(s_part, slot_offset(slot), hdr, sizeof(hdr)) != ESP_OK) {
        return false;
    }
    if (memcmp(hdr, RCUBE_MISSION_MAGIC, 4) != 0 || hdr[OFF_VER] != RCUBE_MISSION_VER) {
        return false;
    }
    uint32_t body_len = rd_u32le(&hdr[OFF_BODYLEN]);
    if (body_len == 0 || body_len > RCUBE_MISSION_SLOT_SIZE - RCUBE_MISSION_HDR_LEN) {
        return false;
    }
    memcpy(s_hdr, hdr, sizeof(hdr));
    return true;
}

esp_err_t rcube_mission_init(void)
{
    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                      ESP_PARTITION_SUBTYPE_ANY, "storage");
    if (s_part == NULL) {
        ESP_LOGE(TAG, "storage 파티션을 찾지 못했습니다 — 미션 저장 불가");
        return ESP_ERR_NOT_FOUND;
    }
    if (s_part->size < 2 * RCUBE_MISSION_SLOT_SIZE) {
        ESP_LOGE(TAG, "storage 파티션이 작습니다(%lu < %u)",
                 (unsigned long)s_part->size, 2 * RCUBE_MISSION_SLOT_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }

    nvs_handle_t h;
    s_active_slot = 0;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        uint8_t v = 0;
        if (nvs_get_u8(h, KEY_ACTIVE, &v) == ESP_OK && v < 2) {
            s_active_slot = v;
        }
        nvs_close(h);
    }

    s_loaded = load_slot(s_active_slot);
    if (s_loaded) {
        char name[RCUBE_MISSION_NAME_LEN + 1] = {0};
        memcpy(name, &s_hdr[OFF_NAME], RCUBE_MISSION_NAME_LEN);
        ESP_LOGI(TAG, "미션 적재됨(slot %u): '%s' type=%u %lu bytes (%lu 키프레임)",
                 s_active_slot, name, s_hdr[OFF_TYPE],
                 (unsigned long)rd_u32le(&s_hdr[OFF_BODYLEN]),
                 (unsigned long)(rd_u32le(&s_hdr[OFF_BODYLEN]) / RCUBE_MISSION_REC_LEN));
    } else {
        ESP_LOGI(TAG, "적재된 미션 없음 (storage %luKB, 슬롯 2×%uKB)",
                 (unsigned long)(s_part->size / 1024), RCUBE_MISSION_SLOT_SIZE / 1024);
    }
    return ESP_OK;
}

/* ---- 업로드 ---------------------------------------------------------- */

uint8_t rcube_mission_begin(uint8_t type, uint32_t total_len, uint32_t crc32,
                            uint32_t unit_sig)
{
    if (s_part == NULL) return RCUBE_RC_BAD_STATE;
    if (s_running) return RCUBE_RC_BAD_STATE;     /* 실행 중 교체 금지 */
    if (type != RCUBE_MISSION_TYPE_TABLE) {
        ESP_LOGW(TAG, "F0: type=%u 미지원(현재 TABLE=1만)", type);
        return RCUBE_RC_BAD_PARAM;
    }
    if (total_len == 0 || total_len > RCUBE_MISSION_SLOT_SIZE - RCUBE_MISSION_HDR_LEN) {
        return RCUBE_RC_BAD_LENGTH;
    }
    if ((total_len % RCUBE_MISSION_REC_LEN) != 0) {
        ESP_LOGW(TAG, "F0: TABLE 본문이 %u바이트 배수가 아님(%lu)",
                 RCUBE_MISSION_REC_LEN, (unsigned long)total_len);
        return RCUBE_RC_BAD_LENGTH;
    }

    /* 항상 비활성 슬롯에 쓴다 — 실패해도 기존 미션이 살아 있어야 한다. */
    s_up_slot = (uint8_t)(s_active_slot ^ 1);
    uint32_t off = slot_offset(s_up_slot);
    uint32_t erase = ((RCUBE_MISSION_HDR_LEN + total_len) + 4095) & ~4095u;
    if (esp_partition_erase_range(s_part, off, erase) != ESP_OK) {
        ESP_LOGE(TAG, "슬롯 %u 소거 실패", s_up_slot);
        return RCUBE_RC_FLASH_FAIL;
    }

    s_up_active = true;
    s_up_total = total_len;
    s_up_written = 0;
    s_up_crc = crc32;
    s_up_unitsig = unit_sig;
    s_up_next_seq = 0;
    s_up_type = type;
    ESP_LOGI(TAG, "F0 업로드 시작: slot %u, %lu bytes, crc=0x%08lx, unit_sig=0x%08lx",
             s_up_slot, (unsigned long)total_len, (unsigned long)crc32,
             (unsigned long)unit_sig);
    return RCUBE_RC_OK;
}

uint8_t rcube_mission_chunk(uint16_t seq, const uint8_t *data, uint16_t len)
{
    if (!s_up_active) return RCUBE_RC_BAD_STATE;
    if (seq != s_up_next_seq) {
        ESP_LOGW(TAG, "F1: 순번 어긋남(기대 %u, 수신 %u) → 업로드 중단",
                 s_up_next_seq, seq);
        s_up_active = false;
        return RCUBE_RC_SEQ_GAP;
    }
    if (len == 0 || s_up_written + len > s_up_total) {
        s_up_active = false;
        return RCUBE_RC_BAD_LENGTH;
    }
    uint32_t off = slot_offset(s_up_slot) + RCUBE_MISSION_HDR_LEN + s_up_written;
    if (esp_partition_write(s_part, off, data, len) != ESP_OK) {
        s_up_active = false;
        return RCUBE_RC_FLASH_FAIL;
    }
    s_up_written += len;
    s_up_next_seq++;
    return RCUBE_RC_OK;
}

uint8_t rcube_mission_commit(void)
{
    if (!s_up_active) return RCUBE_RC_BAD_STATE;
    if (s_up_written != s_up_total) {
        ESP_LOGW(TAG, "F2: 길이 불일치(%lu/%lu)",
                 (unsigned long)s_up_written, (unsigned long)s_up_total);
        s_up_active = false;
        return RCUBE_RC_BAD_LENGTH;
    }

    /* 쓴 내용을 되읽어 CRC 검증 — 플래시에 실제로 남은 것을 확인해야 의미가 있다. */
    uint32_t crc = 0;
    uint8_t buf[256];
    uint32_t remain = s_up_total, off = slot_offset(s_up_slot) + RCUBE_MISSION_HDR_LEN;
    while (remain > 0) {
        uint32_t n = (remain > sizeof(buf)) ? sizeof(buf) : remain;
        if (esp_partition_read(s_part, off, buf, n) != ESP_OK) {
            s_up_active = false;
            return RCUBE_RC_FLASH_FAIL;
        }
        crc = esp_crc32_le(crc, buf, n);
        off += n;
        remain -= n;
    }
    if (crc != s_up_crc) {
        ESP_LOGE(TAG, "F2: CRC 불일치(계산 0x%08lx ≠ 선언 0x%08lx) → 폐기",
                 (unsigned long)crc, (unsigned long)s_up_crc);
        s_up_active = false;
        return RCUBE_RC_CRC_FAIL;
    }

    /* 헤더를 마지막에 쓴다 — 헤더가 유효하다는 건 본문이 온전하다는 뜻이 된다. */
    uint8_t hdr[RCUBE_MISSION_HDR_LEN] = {0};
    memcpy(hdr, RCUBE_MISSION_MAGIC, 4);
    hdr[OFF_VER] = RCUBE_MISSION_VER;
    hdr[OFF_TYPE] = s_up_type;
    hdr[OFF_FLAGS] = 0;
    hdr[OFF_NNODES] = rcube_config_unit_count();
    wr_u32le(&hdr[OFF_UNITSIG], s_up_unitsig);
    wr_u32le(&hdr[OFF_BODYLEN], s_up_total);
    wr_u32le(&hdr[OFF_BODYCRC], s_up_crc);
    snprintf((char *)&hdr[OFF_NAME], RCUBE_MISSION_NAME_LEN, "M%lu",
             (unsigned long)(s_up_total / RCUBE_MISSION_REC_LEN));

    if (esp_partition_write(s_part, slot_offset(s_up_slot), hdr, sizeof(hdr)) != ESP_OK) {
        s_up_active = false;
        return RCUBE_RC_FLASH_FAIL;
    }

    /* 여기서부터 새 미션이 유효하다. 포인터 전환은 원자적. */
    s_active_slot = s_up_slot;
    store_active_slot(s_active_slot);
    memcpy(s_hdr, hdr, sizeof(hdr));
    s_loaded = true;
    s_up_active = false;

    ESP_LOGI(TAG, "F2 커밋 완료: slot %u 활성, %lu 키프레임",
             s_active_slot, (unsigned long)(s_up_total / RCUBE_MISSION_REC_LEN));
    return RCUBE_RC_OK;
}

/* ---- 조회·삭제 ------------------------------------------------------- */

uint8_t rcube_mission_info(uint8_t *out, uint8_t cap)
{
    if (out == NULL || cap < 1 + RCUBE_MISSION_HDR_LEN) {
        return 0;
    }
    out[0] = (uint8_t)rcube_mission_state();
    if (s_loaded) {
        memcpy(&out[1], s_hdr, RCUBE_MISSION_HDR_LEN);
    } else {
        memset(&out[1], 0, RCUBE_MISSION_HDR_LEN);
    }
    return 1 + RCUBE_MISSION_HDR_LEN;
}

uint8_t rcube_mission_delete(void)
{
    if (s_running) return RCUBE_RC_BAD_STATE;
    if (s_part == NULL) return RCUBE_RC_BAD_STATE;
    /* 헤더만 지우면 load_slot이 실패하므로 미션이 없는 것과 같다. */
    if (esp_partition_erase_range(s_part, slot_offset(s_active_slot), 4096) != ESP_OK) {
        return RCUBE_RC_FLASH_FAIL;
    }
    s_loaded = false;
    ESP_LOGI(TAG, "F4 미션 삭제(slot %u)", s_active_slot);
    return RCUBE_RC_OK;
}

/* ---- 실행 ------------------------------------------------------------ */

rcube_mission_state_t rcube_mission_state(void)
{
    if (s_running) return RCUBE_MISSION_RUNNING;
    return s_loaded ? RCUBE_MISSION_LOADED : RCUBE_MISSION_NONE;
}

/* 레코드 1개 읽기. 실패 시 false. */
static bool read_record(uint32_t index, uint16_t *t_ms, uint8_t *node,
                        uint8_t *kind, int32_t *value)
{
    uint8_t r[RCUBE_MISSION_REC_LEN];
    uint32_t off = slot_offset(s_active_slot) + RCUBE_MISSION_HDR_LEN
                   + index * RCUBE_MISSION_REC_LEN;
    if (esp_partition_read(s_part, off, r, sizeof(r)) != ESP_OK) {
        return false;
    }
    *t_ms = (uint16_t)(r[0] | (r[1] << 8));
    *node = r[2];
    *kind = r[3];
    *value = (int32_t)((uint32_t)r[4] | ((uint32_t)r[5] << 8) |
                       ((uint32_t)r[6] << 16) | ((uint32_t)r[7] << 24));
    return true;
}

/* 실행 전 전수 검증(확장 규격 §2.5.4). 통과하면 RCUBE_RC_OK. */
static uint8_t validate(uint32_t *out_count)
{
    if (!s_loaded) return RCUBE_RC_BAD_STATE;
    if (s_hdr[OFF_TYPE] != RCUBE_MISSION_TYPE_TABLE) return RCUBE_RC_BAD_PARAM;

    uint32_t sig = rd_u32le(&s_hdr[OFF_UNITSIG]);
    if (sig != 0 && sig != rcube_mission_unit_sig()) {
        ESP_LOGE(TAG, "unit_sig 불일치(미션 0x%08lx ≠ 이 유닛 0x%08lx) — 다른 구성의 "
                      "미션이다. 엉뚱한 축이 움직이지 않도록 거부한다.",
                 (unsigned long)sig, (unsigned long)rcube_mission_unit_sig());
        return RCUBE_RC_BAD_PARAM;
    }

    uint32_t count = rd_u32le(&s_hdr[OFF_BODYLEN]) / RCUBE_MISSION_REC_LEN;
    uint8_t me = rcube_config_node_id();
    for (uint32_t i = 0; i < count; i++) {
        uint16_t t; uint8_t node, kind; int32_t v;
        if (!read_record(i, &t, &node, &kind, &v)) return RCUBE_RC_FLASH_FAIL;
        if (kind != RCUBE_MISSION_KIND_ANGLE && kind != RCUBE_MISSION_KIND_TONE) {
            ESP_LOGE(TAG, "키프레임 %lu: 미지원 kind=%u", (unsigned long)i, kind);
            return RCUBE_RC_BAD_PARAM;
        }
        if (node < 1 || node > RCUBE_MAX_NODES) return RCUBE_RC_NODE_NOT_FOUND;
        /* 멤버 맵이 있으면 등장 노드가 실제 유닛에 있는지 본다(단일 큐브면 생략). */
        if (rcube_config_unit_count() > 0 &&
            rcube_config_member_cmf(node) == RCUBE_MEMBER_NONE) {
            ESP_LOGE(TAG, "키프레임 %lu: 노드 %u가 멤버 맵에 없다",
                     (unsigned long)i, node);
            return RCUBE_RC_NODE_NOT_FOUND;
        }
        if (kind == RCUBE_MISSION_KIND_TONE) {
            /* 길이 0은 "언제 끝나는지 없는 음"이라 시퀀서가 진행할 수 없다. */
            if (RCUBE_MISSION_TONE_DUR(v) == 0) {
                ESP_LOGE(TAG, "키프레임 %lu: TONE 길이가 0", (unsigned long)i);
                return RCUBE_RC_BAD_PARAM;
            }
            continue;   /* 톤은 각도 리밋 검사 대상이 아니다 */
        }
        /* ★ 3단 안전 1단: 자기 축 목표가 소프트리밋 안인지 실행 전에 전수 검사한다.
         * 중간에 멈추는 것보다 한 축도 움직이지 않고 거부하는 편이 안전하다. */
        if (node == me && !rcube_params_angle_ok(v)) {
            ESP_LOGE(TAG, "키프레임 %lu: 목표 %.2f°가 소프트리밋 밖 → 실행 거부",
                     (unsigned long)i, v / 100.0f);
            return RCUBE_RC_ANGLE_LIMIT;
        }
    }
    *out_count = count;
    return RCUBE_RC_OK;
}

/* 미션 시작(T0) 기준 t_ms가 될 때까지 기다린다. 중지 요청이 오면 즉시 나온다.
 * 남은 시간이 길 때는 10ms씩, 가까워지면 1틱씩 자며 접근한다 — 톤의 시작 시각이
 * 큐브 간에 어긋나면 "동시에"가 깨지므로 마지막 구간은 촘촘히 본다. */
static void wait_until(int64_t t0_us, uint16_t t_ms)
{
    int64_t deadline = t0_us + (int64_t)t_ms * 1000;
    while (!s_stop_req) {
        int64_t remain = deadline - esp_timer_get_time();
        if (remain <= 0) {
            return;
        }
        vTaskDelay(remain > 20000 ? pdMS_TO_TICKS(10) : 1);
    }
}

/* TONE 키프레임 1개를 멤버에게 보낸다(0xE6). 통신방식은 rcube_cmd가 멤버 맵으로 고른다. */
static void dispatch_tone(uint8_t node, uint16_t freq, uint16_t dur)
{
    uint8_t p[4] = {
        (uint8_t)(freq >> 8), (uint8_t)(freq & 0xFF),
        (uint8_t)(dur >> 8),  (uint8_t)(dur & 0xFF),
    };
    if (rcube_cmd_send_to_node(node, RCUBE_OP_GenerateBuzzerTone, p, sizeof(p)) != 0) {
        ESP_LOGW(TAG, "노드 %u에 톤 전달 실패(경로 없음) — %u Hz / %u ms", node, freq, dur);
    }
}

/* 시퀀서 — "무엇을, 언제"를 풀어 자기 축과 멤버에게 분배한다(기획서 8장 중앙 제어).
 *   ANGLE : 자기 노드분만 motion_core 큐에 적재하고 마지막에 한 번에 출발시킨다.
 *           (멤버 축 분배는 다축 동기 규격(§3.4)을 얹을 때 이어 붙인다.)
 *   TONE  : t_ms가 될 때까지 기다렸다가, 같은 시각의 레코드를 한 묶음으로 낸다.
 *           원격 멤버를 먼저 보내고 자기 것을 마지막에 울린다 — 전송 지연만큼
 *           멤버가 늦으므로, 자기가 먼저 울리면 그만큼 어긋난다. */
static void play_task(void *arg)
{
    uint32_t count = (uint32_t)(uintptr_t)arg;
    uint8_t me = rcube_config_node_id();
    bool repeat = (s_hdr[OFF_FLAGS] & RCUBE_MISSION_FLAG_REPEAT) != 0;

    ESP_LOGI(TAG, "미션 실행 시작: %lu 키프레임, 자기 노드=%u, 반복=%s",
             (unsigned long)count, me, repeat ? "예" : "아니오");
    do {
        uint16_t seq = 0;
        bool pushed_angle = false;
        int64_t t0 = esp_timer_get_time();
        uint32_t i = 0;
        while (i < count && !s_stop_req) {
            while (s_paused && !s_stop_req) {
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            uint16_t t_ms; uint8_t node, kind; int32_t v;
            if (!read_record(i, &t_ms, &node, &kind, &v)) {
                ESP_LOGE(TAG, "키프레임 %lu 읽기 실패 → 중단", (unsigned long)i);
                break;
            }

            if (kind == RCUBE_MISSION_KIND_ANGLE) {
                i++;
                if (node != me) {
                    continue;
                }
                motion_waypoint_t wp = {
                    .t_offset_us = (uint32_t)t_ms * 1000u,
                    .position = v,
                    .velocity = 0,
                    .flags = 0,
                    .seq = seq,
                };
                if (!motion_core_push(&wp, true /* Buffered */)) {
                    ESP_LOGW(TAG, "모션 큐 가득참 — 잠시 대기");
                    vTaskDelay(pdMS_TO_TICKS(50));
                    i--;   /* 같은 레코드 재시도(seq는 성공했을 때만 올린다) */
                } else {
                    seq++;
                    pushed_angle = true;
                }
                continue;
            }

            /* TONE — 같은 t_ms의 연속 레코드를 한 묶음으로 처리한다. */
            wait_until(t0, t_ms);
            uint16_t group_t = t_ms;
            uint16_t self_freq = 0, self_dur = 0;
            while (i < count && !s_stop_req) {
                if (!read_record(i, &t_ms, &node, &kind, &v)) {
                    break;
                }
                if (t_ms != group_t || kind != RCUBE_MISSION_KIND_TONE) {
                    break;
                }
                uint16_t freq = RCUBE_MISSION_TONE_FREQ(v);
                uint16_t dur  = RCUBE_MISSION_TONE_DUR(v);
                if (node == me) {
                    self_freq = freq;
                    self_dur = dur;
                } else {
                    dispatch_tone(node, freq, dur);
                }
                i++;
            }
            if (self_dur > 0 && !s_stop_req) {
                rcube_buzzer_play_tone(self_freq, self_dur);
            }
        }
        if (pushed_angle && !s_stop_req) {
            motion_core_execute(MOTION_EXEC_RUN, 0);   /* 적재분 즉시 출발 */
        }
        /* 반복이면 한 사이클이 끝날 시간을 두고 다시 채운다. */
        if (repeat && !s_stop_req) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    } while (repeat && !s_stop_req);

    ESP_LOGI(TAG, "미션 실행 종료(%s)", s_stop_req ? "중지 요청" : "완료");
    s_running = false;
    s_paused = false;
    s_play_task = NULL;
    vTaskDelete(NULL);
}

uint8_t rcube_mission_control(uint8_t action)
{
    switch (action) {
    case RCUBE_MISSION_RUN: {
        if (s_running) return RCUBE_RC_BAD_STATE;
        uint32_t count = 0;
        uint8_t rc = validate(&count);
        if (rc != RCUBE_RC_OK) {
            return rc;
        }
        s_stop_req = false;
        s_paused = false;
        s_running = true;
        if (xTaskCreatePinnedToCore(play_task, "mission", 4096,
                                    (void *)(uintptr_t)count, 4, &s_play_task, 0) != pdPASS) {
            s_running = false;
            return RCUBE_RC_BAD_STATE;
        }
        return RCUBE_RC_OK;
    }
    case RCUBE_MISSION_STOP:
        if (!s_running) return RCUBE_RC_BAD_STATE;
        s_stop_req = true;
        motion_core_execute(MOTION_EXEC_CANCEL, 0);
        ESP_LOGI(TAG, "F9 Stop — 모션 큐를 비우고 정지");
        return RCUBE_RC_OK;
    case RCUBE_MISSION_PAUSE:
        if (!s_running) return RCUBE_RC_BAD_STATE;
        s_paused = true;
        return RCUBE_RC_OK;
    case RCUBE_MISSION_RESUME:
        if (!s_running) return RCUBE_RC_BAD_STATE;
        s_paused = false;
        return RCUBE_RC_OK;
    default:
        return RCUBE_RC_BAD_PARAM;
    }
}

/* ---- 연결 완료 → 자동 실행 (기획서 7.4-6) ---------------------------- */

/* 분기별 완료 표시. 인덱스 = CMF(0=BLE, 1=CAN). */
static bool s_branch_done[2];
static bool s_autostarted;

void rcube_mission_branch_ready(uint8_t cmf)
{
    if (cmf > RCUBE_MEMBER_CAN) {
        return;
    }
    s_branch_done[cmf] = true;

    if (rcube_config_ecf() != 1 || s_autostarted || s_running) {
        return;
    }
    /* 맵이 요구하는 분기가 다 모여야 "필요한 모든 큐브가 연결"이다. 혼합 구성에서
     * 한쪽 분기만 보고 출발하면 아직 붙지 않은 큐브가 미션에서 빠진다. */
    if (rcube_config_has_member(RCUBE_MEMBER_BLE) && !s_branch_done[RCUBE_MEMBER_BLE]) {
        return;
    }
    if (rcube_config_has_member(RCUBE_MEMBER_CAN) && !s_branch_done[RCUBE_MEMBER_CAN]) {
        return;
    }
    if (!s_loaded) {
        ESP_LOGW(TAG, "유닛 연결 완료 — 그러나 적재된 미션이 없다(7.3-2에서 업로드 필요)");
        return;
    }

    s_autostarted = true;
    uint8_t rc = rcube_mission_control(RCUBE_MISSION_RUN);
    if (rc == RCUBE_RC_OK) {
        ESP_LOGI(TAG, "유닛 연결 완료 → 저장된 미션 자동 실행(7.4-6)");
    } else {
        ESP_LOGE(TAG, "유닛 연결 완료 — 미션 자동 실행 거부됨(rc=0x%02x). "
                      "unit_sig/노드ID/리밋 검증 실패 여부를 로그에서 확인하라.", rc);
    }
}
