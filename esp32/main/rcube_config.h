/*
 * rcube_config — R큐브 영구 설정(NVS 저장)
 * ------------------------------------------------------------------
 * 모든 R큐브는 자신의 "그룹번호"와 "노드번호"를 비휘발성으로 보관한다.
 * 전원이 꺼져도 유지되며, 값이 없으면(공장 초기/NVS 소거 후) 공장 디폴트로
 * 채워 저장한다.
 *
 *   - 그룹번호(group_id) : 8bit. 멀티롤 그룹 매칭·DiscoveryGroupIdSound 등에 사용.
 *   - 노드번호(node_id)  : 8bit. 개별 노드 주소(RCUBE_ADDR_NODE_MIN..MAX).
 *
 * NVS 네임스페이스 "rcube", 키 "group_id"/"node_id" (각 u8).
 * NVS 플래시 초기화(nvs_flash_init)는 app_main에서 먼저 끝나 있어야 한다.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

/* 공장 디폴트값 (일단 이 값으로 저장; 추후 조정 가능). */
#define RCUBE_DEFAULT_GROUP_ID  0x00u   /* 0 = 미지정(그룹 없음) */
#define RCUBE_DEFAULT_NODE_ID   0x01u   /* 1 = RCUBE_ADDR_NODE_MIN */

/* NVS에서 설정을 로드한다. 값이 없으면 공장 디폴트로 채워 저장한다.
 * app_main 초기화(nvs_flash_init 이후)에서 1회 호출. */
esp_err_t rcube_config_init(void);

/* 캐시된 현재 값 조회. */
uint8_t rcube_config_group_id(void);
uint8_t rcube_config_node_id(void);

/* 값 변경 + NVS 영구 저장. 성공 시 ESP_OK. */
esp_err_t rcube_config_set_group_id(uint8_t group_id);
esp_err_t rcube_config_set_node_id(uint8_t node_id);

/* 공장 디폴트로 되돌려 저장(ResetConfig 대응). */
esp_err_t rcube_config_reset_factory(void);
