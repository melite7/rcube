#include "rcube_config.h"

#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "cfg";

#define RCUBE_NVS_NAMESPACE "rcube"
#define KEY_GROUP_ID        "group_id"
#define KEY_NODE_ID         "node_id"

/* 캐시(단일 스레드 초기화 후 읽기 위주). */
static uint8_t s_group_id = RCUBE_DEFAULT_GROUP_ID;
static uint8_t s_node_id  = RCUBE_DEFAULT_NODE_ID;

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

    if (dirty) {
        esp_err_t cerr = nvs_commit(h);
        if (cerr != ESP_OK) {
            ESP_LOGE(TAG, "nvs_commit 실패: %s", esp_err_to_name(cerr));
        }
    }
    nvs_close(h);

    if (e1 != ESP_OK) return e1;
    if (e2 != ESP_OK) return e2;

    ESP_LOGI(TAG, "config loaded: group_id=0x%02x, node_id=0x%02x", s_group_id, s_node_id);
    return ESP_OK;
}

uint8_t rcube_config_group_id(void) { return s_group_id; }
uint8_t rcube_config_node_id(void)  { return s_node_id; }

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

esp_err_t rcube_config_reset_factory(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(RCUBE_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open 실패: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_set_u8(h, KEY_GROUP_ID, RCUBE_DEFAULT_GROUP_ID);
    if (err == ESP_OK) err = nvs_set_u8(h, KEY_NODE_ID, RCUBE_DEFAULT_NODE_ID);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err == ESP_OK) {
        s_group_id = RCUBE_DEFAULT_GROUP_ID;
        s_node_id = RCUBE_DEFAULT_NODE_ID;
        ESP_LOGI(TAG, "공장 초기화: group_id=0x%02x, node_id=0x%02x", s_group_id, s_node_id);
    }
    return err;
}
