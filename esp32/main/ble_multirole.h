/*
 * ble_multirole — 아그리게이터(BLE central) 역할 (Phase 5)
 * ------------------------------------------------------------------
 * PC에 대해서는 peripheral(ble_rcube.c)로 붙어 있는 상태에서, 이 큐브가
 * 추가로 central 이 되어 주변 R큐브(멤버)들을 스캔·연결한다.
 *
 * 동작:
 *   - A0(SetMultiroleAggregator) 수신 → ble_multirole_start_aggregator()
 *     → 이름 "RCUBE" 광고를 스캔, 멤버(총 N-1대)를 순차 연결.
 *   - 멤버가 연결되고 RCBE 특성까지 찾으면(READY) 가상ID(2..N)를 부여하고
 *     rcube_cmd_report_members()로 PC에 0xA1(멤버 수)을 회신.
 *   - PC가 멤버 대상(target=vid) 프레임을 보내면 ble_multirole_forward()로
 *     해당 멤버에게 GATT write(중계). 이때 target은 0xFE로 재기입한다.
 *   - PC 연결이 끊기면 ble_multirole_stop_aggregator()로 전 멤버 해제.
 *
 * 모든 함수는 NimBLE 호스트 태스크 컨텍스트(GATT/GAP 콜백)에서 호출된다.
 */
#pragma once

#include <stdint.h>

/* 아그리게이터 승격: 멤버 스캔·연결 시작(link_count = 본인 포함 총 N). */
void ble_multirole_start_aggregator(uint8_t link_count, uint8_t group_mode);

/* 아그리게이터 해제: 모든 멤버 연결 종료·스캔 중지·상태 초기화. */
void ble_multirole_stop_aggregator(void);

/* PC가 보낸, 멤버(가상ID target_id) 대상 프레임을 해당 멤버로 중계.
 * 성공 0. 아그리게이터가 아니거나 대상 멤버가 없으면 음수. */
int ble_multirole_forward(uint8_t target_id, const uint8_t *frame, uint16_t len);

/* 현재 READY(특성까지 확보) 상태 멤버 수. */
uint8_t ble_multirole_member_count(void);
