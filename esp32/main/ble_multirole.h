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

#include <stdbool.h>
#include <stdint.h>

/* BLE 허브 승격: 멤버 스캔·연결 시작(link_count = 본인 포함 총 N).
 *
 * 고정형/비고정형은 기획서 7.5 [새 방식]대로 "저장 노드ID 유무"로 스스로 판정한다
 * (별도 플래그·전환 명령 없음):
 *   - 자기 node_id == 0 (비고정형 초기구성) : 아무 R큐브나 연결 순서대로 붙이고
 *     가상ID 2,3,…을 배정한다.
 *   - 자기 node_id != 0 (고정형 재연결)     : 광고이름의 NN≥1인 큐브만 붙이고
 *     그 NN을 그대로 가상ID로 쓴다(연결 순서 무관). */
void ble_multirole_start_aggregator(uint8_t link_count, uint8_t group_mode);

/* edge central 시작(기획서 7.4-3·4 [BLE 멤버]).
 *
 * 독립로봇유닛에는 BLE 허브 큐브가 없다. ECF=1 큐브가 저장된 멤버 맵을 보고
 * CMF=0(BLE)인 멤버 노드ID들에 직접 BLE scan → 다중 연결한다.
 *   - 연결 순서는 전원 켜는 순서와 무관하며 스캐너인 edge central이 정한다.
 *   - 각 멤버는 광고이름의 노드ID(RCUBEROBOT.GG.NN)로 식별·매핑된다.
 *   - 맵에 없는 노드ID는 무시하고, 빠진 노드는 계속 스캔하며 기다린다.
 * BLE 멤버가 하나도 없으면 아무것도 하지 않고 false를 돌려준다. */
bool ble_multirole_start_edge(void);

/* 아그리게이터/edge central 해제: 모든 멤버 연결 종료·스캔 중지·상태 초기화. */
void ble_multirole_stop_aggregator(void);

/* PC가 보낸, 멤버(가상ID target_id) 대상 프레임을 해당 멤버로 중계.
 * 성공 0. 아그리게이터가 아니거나 대상 멤버가 없으면 음수. */
int ble_multirole_forward(uint8_t target_id, const uint8_t *frame, uint16_t len);

/* 브로드캐스트: 현재 READY인 모든 멤버로 프레임을 중계(target은 0xFE로 재기입).
 * 전송한 멤버 수를 반환. 아그리게이터가 아니면 음수. */
int ble_multirole_broadcast(const uint8_t *frame, uint16_t len);

/* 현재 READY(특성까지 확보) 상태 멤버 수. */
uint8_t ble_multirole_member_count(void);
