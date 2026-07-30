#include "rcube_params.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "params";

#define NVS_NS   "rcube"
#define KEY_PARAMS "params"

/* 기본값: 전부 해제. 실보드 튜닝 전에는 큐브가 목표를 막지 않아야 개발이 진행된다.
 * 리밋은 기구 형상이 확정된 뒤 상위가 D8/CD/CE로 넣고 DB로 저장한다. */
static rcube_params_t s_p;

esp_err_t rcube_params_init(void)
{
    memset(&s_p, 0, sizeof(s_p));

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open 실패: %s", esp_err_to_name(err));
        return err;
    }
    size_t len = sizeof(s_p);
    err = nvs_get_blob(h, KEY_PARAMS, &s_p, &len);
    nvs_close(h);

    if (err != ESP_OK || len != sizeof(s_p)) {
        memset(&s_p, 0, sizeof(s_p));
        ESP_LOGI(TAG, "저장된 파라미터 없음 → 리밋 전부 해제(기본값)");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "파라미터 로드: 각도 %.2f~%.2f°, 최대속도 %.1f°/s, 추종오차 %.2f°, "
                  "과전류 %.2fA, 과온 %u°C",
             s_p.angle_min / 100.0f, s_p.angle_max / 100.0f, s_p.max_vel / 10.0f,
             s_p.follow_err / 100.0f, s_p.over_current / 100.0f, s_p.over_temp);
    return ESP_OK;
}

const rcube_params_t *rcube_params(void) { return &s_p; }

void rcube_params_set_angle_limits(int32_t min_centi_deg, int32_t max_centi_deg)
{
    s_p.angle_min = min_centi_deg;
    s_p.angle_max = max_centi_deg;
    if (min_centi_deg == 0 && max_centi_deg == 0) {
        ESP_LOGW(TAG, "각도 소프트리밋 해제 — 상위가 보내는 목표를 그대로 따른다");
    } else {
        ESP_LOGI(TAG, "각도 소프트리밋: %.2f° ~ %.2f°",
                 min_centi_deg / 100.0f, max_centi_deg / 100.0f);
    }
}

void rcube_params_set_motion_limits(uint16_t max_vel, uint16_t max_acc, uint16_t max_jerk)
{
    s_p.max_vel = max_vel;
    s_p.max_acc = max_acc;
    s_p.max_jerk = max_jerk;
    ESP_LOGI(TAG, "모션 리밋: 속도 %.1f°/s, 가속 %u°/s², 저크 %u°/s³ (0=제한없음)",
             max_vel / 10.0f, max_acc, max_jerk);
}

void rcube_params_set_fault_thresholds(uint16_t follow_err, uint16_t over_current,
                                       uint8_t over_temp)
{
    s_p.follow_err = follow_err;
    s_p.over_current = over_current;
    s_p.over_temp = over_temp;
    ESP_LOGI(TAG, "폴트 임계: 추종오차 %.2f°, 과전류 %.2fA, 과온 %u°C (0=감시끔)",
             follow_err / 100.0f, over_current / 100.0f, over_temp);
}

esp_err_t rcube_params_save(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(h, KEY_PARAMS, &s_p, sizeof(s_p));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "파라미터 NVS 저장 완료");
    } else {
        ESP_LOGE(TAG, "파라미터 저장 실패: %s", esp_err_to_name(err));
    }
    return err;
}

bool rcube_params_angle_ok(int32_t centi_deg)
{
    if (s_p.angle_min == 0 && s_p.angle_max == 0) {
        return true;   /* 리밋 미설정 */
    }
    return (centi_deg >= s_p.angle_min) && (centi_deg <= s_p.angle_max);
}

uint16_t rcube_params_clamp_vel(uint16_t vel_deci_dps)
{
    if (s_p.max_vel == 0 || vel_deci_dps <= s_p.max_vel) {
        return vel_deci_dps;
    }
    return s_p.max_vel;
}
