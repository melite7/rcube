#include "rcube_config.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "cfg";

#define RCUBE_NVS_NAMESPACE "rcube"
#define KEY_GROUP_ID        "group_id"
#define KEY_NODE_ID         "node_id"
#define KEY_CMF             "cmf"
#define KEY_TERM_ID         "term_id"
#define KEY_ECF             "ecf"
#define KEY_UNIT_N          "unit_n"
#define KEY_MEMBERS         "members"   /* blob, RCUBE_MAX_NODES 바이트 */

/* 캐시(단일 스레드 초기화 후 읽기 위주). */
static uint8_t s_group_id = RCUBE_DEFAULT_GROUP_ID;
static uint8_t s_node_id  = RCUBE_DEFAULT_NODE_ID;
static uint8_t s_cmf      = RCUBE_DEFAULT_CMF;
static uint8_t s_term_id  = RCUBE_DEFAULT_TERM_ID;
static uint8_t s_ecf      = RCUBE_DEFAULT_ECF;
static uint8_t s_unit_n;
static uint8_t s_members[RCUBE_MAX_NODES];   /* 전부 RCUBE_MEMBER_NONE = 맵 없음 */

static void clear_member_map(void)
{
    for (int i = 0; i < RCUBE_MAX_NODES; i++) {
        s_members[i] = RCUBE_MEMBER_NONE;
    }
    s_unit_n = 0;
}

/* 키가 없으면 디폴트로 채우고 dirty 표시. 있으면 캐시에 로드. */
static esp_err_t load_or_default_u8(nvs_handle_t h, const char *key,
                                    uint8_t *cache, uint8_t def, bool *dirty)
{
    uint8_t v;
    esp_err_t err = nvs_get_u8(h, key, &v);
    if (err == ESP_OK) {
        *cache = v;
        return ESP_OK;
    }
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        *cache = def;
        esp_err_t werr = nvs_set_u8(h, key, def);
        if (werr != ESP_OK) {
            ESP_LOGE(TAG, "'%s' 디폴트 저장 실패: %s", key, esp_err_to_name(werr));
            return werr;
        }
        *dirty = true;
        ESP_LOGI(TAG, "'%s' 없음 → 공장 디폴트 0x%02x 저장", key, def);
        return ESP_OK;
    }
    ESP_LOGE(TAG, "'%s' 읽기 실패: %s", key, esp_err_to_name(err));
    return err;
}

esp_err_t rcube_config_init(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(RCUBE_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open('%s') 실패: %s", RCUBE_NVS_NAMESPACE, esp_err_to_name(err));
        return err;
    }

    bool dirty = false;
    esp_err_t e1 = load_or_default_u8(h, KEY_GROUP_ID, &s_group_id, RCUBE_DEFAULT_GROUP_ID, &dirty);
    esp_err_t e2 = load_or_default_u8(h, KEY_NODE_ID, &s_node_id, RCUBE_DEFAULT_NODE_ID, &dirty);
    esp_err_t e3 = load_or_default_u8(h, KEY_CMF, &s_cmf, RCUBE_DEFAULT_CMF, &dirty);
    esp_err_t e4 = load_or_default_u8(h, KEY_TERM_ID, &s_term_id, RCUBE_DEFAULT_TERM_ID, &dirty);
    esp_err_t e5 = load_or_default_u8(h, KEY_ECF, &s_ecf, RCUBE_DEFAULT_ECF, &dirty);

    /* 멤버 맵(ECF=1 큐브만 가진다). 없으면 "맵 없음"으로 둔다 — 디폴트를 굳이
     * 써 두지 않는 이유는, 맵의 부재 자체가 "독립유닛 아님"을 뜻하기 때문. */
    clear_member_map();
    uint8_t n = 0;
    if (nvs_get_u8(h, KEY_UNIT_N, &n) == ESP_OK) {
        s_unit_n = n;
    }
    size_t mlen = sizeof(s_members);
    if (nvs_get_blob(h, KEY_MEMBERS, s_members, &mlen) != ESP_OK || mlen != sizeof(s_members)) {
        clear_member_map();
    }

    if (dirty) {
        esp_err_t cerr = nvs_commit(h);
        if (cerr != ESP_OK) {
            ESP_LOGE(TAG, "nvs_commit 실패: %s", esp_err_to_name(cerr));
        }
    }
    nvs_close(h);

    if (e1 != ESP_OK) return e1;
    if (e2 != ESP_OK) return e2;
    if (e3 != ESP_OK) return e3;
    if (e4 != ESP_OK) return e4;
    if (e5 != ESP_OK) return e5;

    ESP_LOGI(TAG, "config loaded: group_id=0x%02x, node_id=0x%02x, cmf=%u(%s), term_id=0x%02x, ecf=%u",
             s_group_id, s_node_id, s_cmf, s_cmf ? "CAN" : "BLE", s_term_id, s_ecf);
    if (s_ecf) {
        char buf[RCUBE_MAX_NODES * 8 + 1];
        int off = 0;
        for (int i = 0; i < RCUBE_MAX_NODES && off < (int)sizeof(buf) - 8; i++) {
            if (s_members[i] == RCUBE_MEMBER_NONE) continue;
            off += snprintf(buf + off, sizeof(buf) - off, "%d:%s ",
                            i + 1, s_members[i] == RCUBE_MEMBER_CAN ? "CAN" : "BLE");
        }
        buf[off] = '\0';
        ESP_LOGI(TAG, "edge central: unit_n=%u, member map = %s", s_unit_n, off ? buf : "(없음)");
    }
    return ESP_OK;
}

uint8_t rcube_config_group_id(void)   { return s_group_id; }
uint8_t rcube_config_node_id(void)    { return s_node_id; }
uint8_t rcube_config_cmf(void)        { return s_cmf; }
uint8_t rcube_config_term_id(void)    { return s_term_id; }
uint8_t rcube_config_ecf(void)        { return s_ecf; }
uint8_t rcube_config_unit_count(void) { return s_unit_n; }

const uint8_t *rcube_config_member_map(void) { return s_members; }

uint8_t rcube_config_member_cmf(uint8_t node_id)
{
    if (node_id < 1 || node_id > RCUBE_MAX_NODES) {
        return RCUBE_MEMBER_NONE;
    }
    return s_members[node_id - 1];
}

bool rcube_config_has_member(uint8_t cmf)
{
    for (int i = 0; i < RCUBE_MAX_NODES; i++) {
        if ((uint8_t)(i + 1) == s_node_id) {
            continue;   /* 자기 자신은 멤버가 아니다 */
        }
        if (s_members[i] == cmf) {
            return true;
        }
    }
    return false;
}

/* 단일 키 저장 헬퍼. */
static esp_err_t store_u8(const char *key, uint8_t value)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(RCUBE_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open 실패: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_set_u8(h, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "'%s'=0x%02x 저장 실패: %s", key, value, esp_err_to_name(err));
    }
    return err;
}

esp_err_t rcube_config_set_group_id(uint8_t group_id)
{
    esp_err_t err = store_u8(KEY_GROUP_ID, group_id);
    if (err == ESP_OK) {
        s_group_id = group_id;
        ESP_LOGI(TAG, "group_id → 0x%02x", group_id);
    }
    return err;
}

esp_err_t rcube_config_set_node_id(uint8_t node_id)
{
    esp_err_t err = store_u8(KEY_NODE_ID, node_id);
    if (err == ESP_OK) {
        s_node_id = node_id;
        ESP_LOGI(TAG, "node_id → 0x%02x", node_id);
    }
    return err;
}

esp_err_t rcube_config_set_cmf(uint8_t cmf)
{
    esp_err_t err = store_u8(KEY_CMF, cmf);
    if (err == ESP_OK) {
        s_cmf = cmf;
        ESP_LOGI(TAG, "cmf → %u (%s)", cmf, cmf ? "CAN" : "BLE");
    }
    return err;
}

esp_err_t rcube_config_set_term_id(uint8_t term_id)
{
    esp_err_t err = store_u8(KEY_TERM_ID, term_id);
    if (err == ESP_OK) {
        s_term_id = term_id;
        ESP_LOGI(TAG, "term_id → 0x%02x", term_id);
    }
    return err;
}

esp_err_t rcube_config_set_edge(uint8_t ecf, uint8_t unit_count, const uint8_t *member_map)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(RCUBE_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open 실패: %s", esp_err_to_name(err));
        return err;
    }
    uint8_t map[RCUBE_MAX_NODES];
    if (ecf && member_map != NULL) {
        memcpy(map, member_map, sizeof(map));
    } else {
        /* 강등(ecf=0)이면 멤버 맵도 함께 지운다(기획서 7.3 [독립→일반 되돌리기]). */
        memset(map, RCUBE_MEMBER_NONE, sizeof(map));
        unit_count = 0;
    }
    err = nvs_set_u8(h, KEY_ECF, ecf ? 1 : 0);
    if (err == ESP_OK) err = nvs_set_u8(h, KEY_UNIT_N, unit_count);
    if (err == ESP_OK) err = nvs_set_blob(h, KEY_MEMBERS, map, sizeof(map));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "edge 설정 저장 실패: %s", esp_err_to_name(err));
        return err;
    }
    s_ecf = ecf ? 1 : 0;
    s_unit_n = unit_count;
    memcpy(s_members, map, sizeof(s_members));
    ESP_LOGI(TAG, "edge 설정 저장: ecf=%u unit_n=%u", s_ecf, s_unit_n);
    return ESP_OK;
}

esp_err_t rcube_config_reset_factory(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(RCUBE_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open 실패: %s", esp_err_to_name(err));
        return err;
    }
    uint8_t empty[RCUBE_MAX_NODES];
    memset(empty, RCUBE_MEMBER_NONE, sizeof(empty));

    err = nvs_set_u8(h, KEY_GROUP_ID, RCUBE_DEFAULT_GROUP_ID);
    if (err == ESP_OK) err = nvs_set_u8(h, KEY_NODE_ID, RCUBE_DEFAULT_NODE_ID);
    if (err == ESP_OK) err = nvs_set_u8(h, KEY_CMF, RCUBE_DEFAULT_CMF);
    if (err == ESP_OK) err = nvs_set_u8(h, KEY_TERM_ID, RCUBE_DEFAULT_TERM_ID);
    /* 기획서 7.3 [노드ID/세팅 초기화 - 공통]: ECF=0, 멤버 맵도 삭제. */
    if (err == ESP_OK) err = nvs_set_u8(h, KEY_ECF, RCUBE_DEFAULT_ECF);
    if (err == ESP_OK) err = nvs_set_u8(h, KEY_UNIT_N, 0);
    if (err == ESP_OK) err = nvs_set_blob(h, KEY_MEMBERS, empty, sizeof(empty));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err == ESP_OK) {
        s_group_id = RCUBE_DEFAULT_GROUP_ID;
        s_node_id = RCUBE_DEFAULT_NODE_ID;
        s_cmf = RCUBE_DEFAULT_CMF;
        s_term_id = RCUBE_DEFAULT_TERM_ID;
        s_ecf = RCUBE_DEFAULT_ECF;
        clear_member_map();
        ESP_LOGI(TAG, "공장 초기화: group=0x%02x node=0x%02x cmf=%u term=0x%02x ecf=%u (멤버맵 삭제)",
                 s_group_id, s_node_id, s_cmf, s_term_id, s_ecf);
    }
    return err;
}
