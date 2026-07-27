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

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* 공장 디폴트값 (일단 이 값으로 저장; 추후 조정 가능). */
#define RCUBE_DEFAULT_GROUP_ID  0x00u   /* 0 = 미지정(그룹 없음) */
#define RCUBE_DEFAULT_NODE_ID   0x00u   /* 0 = 미할당(공장 출하값). 순서고정 시 1~8 저장 */
#define RCUBE_DEFAULT_CMF       0x00u   /* 통신방식: 0=BLE(공장기본), 1=CAN */
#define RCUBE_DEFAULT_TERM_ID   0x00u   /* CAN 종단노드ID(=CAN 큐브 중 최대 노드ID). 0=없음 */
#define RCUBE_DEFAULT_ECF       0x00u   /* 0=일반 큐브(공장기본), 1=edge central(리드 큐브) */

/* ---- 멤버 맵 (기획서 7.3-3 ★보강) ------------------------------------
 * edge central(ECF=1)이 저장하는 "멤버별 {노드ID, CMF}" 표.
 * 인덱스 i = 노드ID (i+1), 값 = 그 노드의 통신방식.
 * 부팅 시 이 맵으로 (a) 어느 멤버를 CAN/BLE로 연결할지, (b) 어떤 통신 서버를
 * 켤지(CAN만/BLE만/둘 다)를 판단한다. */
#define RCUBE_MAX_NODES         8u      /* 한 로봇유닛당 최대 큐브 수(기획서 [연결 가능 최대 수]) */
#define RCUBE_MEMBER_NONE       0xFFu   /* 그 노드ID는 유닛에 없음 */
#define RCUBE_MEMBER_BLE        0x00u
#define RCUBE_MEMBER_CAN        0x01u

/* NVS에서 설정을 로드한다. 값이 없으면 공장 디폴트로 채워 저장한다.
 * app_main 초기화(nvs_flash_init 이후)에서 1회 호출. */
esp_err_t rcube_config_init(void);

/* 캐시된 현재 값 조회. */
uint8_t rcube_config_group_id(void);
uint8_t rcube_config_node_id(void);
uint8_t rcube_config_cmf(void);       /* 0=BLE, 1=CAN */
uint8_t rcube_config_term_id(void);   /* CAN 종단노드ID */
uint8_t rcube_config_ecf(void);       /* 0=일반, 1=edge central */
uint8_t rcube_config_unit_count(void);/* 유닛 전체 큐브 수 N (ECF=1일 때 의미) */

/* 멤버 맵(RCUBE_MAX_NODES 바이트). 인덱스 i = 노드ID(i+1)의 CMF 또는 NONE. */
const uint8_t *rcube_config_member_map(void);

/* 멤버 맵 조회 헬퍼: 노드ID(1~8)의 통신방식. 없으면 RCUBE_MEMBER_NONE. */
uint8_t rcube_config_member_cmf(uint8_t node_id);

/* 자기 자신을 뺀 멤버 중 해당 통신방식이 하나라도 있는지(서버 선택용, 7.4-3). */
bool rcube_config_has_member(uint8_t cmf);

/* 값 변경 + NVS 영구 저장. 성공 시 ESP_OK. */
esp_err_t rcube_config_set_group_id(uint8_t group_id);
esp_err_t rcube_config_set_node_id(uint8_t node_id);
esp_err_t rcube_config_set_cmf(uint8_t cmf);
esp_err_t rcube_config_set_term_id(uint8_t term_id);

/* edge central 설정 저장(기획서 7.3-3). ecf=0이면 강등 — 멤버 맵/N도 함께 지운다. */
esp_err_t rcube_config_set_edge(uint8_t ecf, uint8_t unit_count, const uint8_t *member_map);

/* 공장 디폴트로 되돌려 저장(ResetConfig 대응). */
esp_err_t rcube_config_reset_factory(void);
