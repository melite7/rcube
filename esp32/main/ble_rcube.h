/*
 * ble_rcube — R큐브 BLE 페리페럴(NimBLE)
 * ------------------------------------------------------------------
 * 로드맵 12.1/12.2: 통신 스택은 Core 0. 지금은 "이름 광고 + 연결 수락"만.
 *
 * 동작:
 *   - ble_rcube_init(): NimBLE 스택/GATT 초기화. 광고는 아직 시작하지 않는다.
 *   - ble_rcube_start_advertising(): 광고 시작 요청(BOOT 버튼에서 호출).
 *     스택 sync 이전에 호출돼도 sync 완료 시 자동으로 광고를 시작한다.
 *   - 연결이 끊기면 재연결을 위해 광고를 자동 재개한다.
 *
 * 광고 이름: "RCUBE00.00"
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "host/ble_uuid.h"

/* R큐브 커스텀 GATT (peripheral 광고 / central 탐색 양쪽에서 공유).
 *   Service 52434245-0000-... ("RCBE"), Char 52434245-0001-... (RD|WR|NOTIFY) */
extern const ble_uuid128_t rcube_svc_uuid;
extern const ble_uuid128_t rcube_chr_uuid;

/* NimBLE 스택 초기화(광고 시작 안 함). app_main에서 1회 호출. */
void ble_rcube_init(void);

/* 연결모드 광고 시작을 요청한다(일반 이름 RCUBEROBOT). 임의 태스크에서 호출 가능.
 * CMF=1(CAN) 큐브는 BLE로 광고하지 않으므로 false를 돌려준다(기획서 5장/7.2-8). */
bool ble_rcube_start_advertising(void);

/* 설정모드 광고를 시작한다(이름 RCUBECONFIG). 이미 광고 중이면 이름을 바꿔 재시작. */
void ble_rcube_start_config_advertising(void);

/* 현재 광고 중이거나 연결된 상태인지. */
bool ble_rcube_is_active(void);

/* 완성된 와이어 프레임을 PC(peripheral 연결) 쪽으로 notify 회신. 성공 0. */
int ble_rcube_notify_pc(const uint8_t *frame, uint16_t len);

/* central(멤버 스캔/연결)에서 쓸 자기 주소 타입. sync 이후 유효. */
uint8_t ble_rcube_own_addr_type(void);
