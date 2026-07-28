/*
 * can_transport — R큐브 CAN(TWAI) 전송 계층
 * ------------------------------------------------------------------
 * CMF=1(CAN) 큐브가 부팅 시 사용하는 통신 경로. ESP32-S3 내장 TWAI 사용.
 * 핀맵(메인보드): IO4=CAN_TX, IO5=CAN_RX, IO6=CAN_STB(Low=정상), IO7=CAN_TERM_EN(High=120Ω ON).
 *
 * 프레임: shared-protocol 29비트 확장 ID
 *   [28:26]Priority [25:18]OpCode [17]MULTI [16]FLAG [15:8]SrcId [7:0]DstId
 *   데이터필드(≤8B) = 표준프레임의 Property+Value(= payload).
 *
 * 동작:
 *   - can_transport_init(): STB 정상, 자가종단(node_id==term_id면 TERM_EN ON),
 *     TWAI 설치·시작, RX 태스크 + 하트비트 태스크 기동. 부팅 시 NodeAnnounce 발행.
 *   - 수신 프레임은 표준프레임으로 재구성해 rcube_cmd_on_frame()으로 디스패치(BLE와 공용).
 *     명령 회신(CmdAck 등)은 CAN으로 나가도록 rcube_cmd 응답 콜백을 CAN으로 교체한다.
 *
 * ※ 개발보드엔 CAN 트랜시버가 없어 실제 버스 통신은 메인보드에서만 검증된다.
 *   (트랜시버/버스 없으면 TX가 ACK를 못 받아 오류로 남지만 컨트롤러 초기화는 정상.)
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* CAN 전송 계층 초기화 + 태스크 기동. node_id/term_id는 rcube_config 값. */
esp_err_t can_transport_init(uint8_t node_id, uint8_t term_id);

/* 임의 CAN 프레임 송신(저수준). 성공 ESP_OK.
 * len이 8을 넘으면 확장 규격 §5 멀티프레임(MULTI=1)으로 자동 분할한다
 * (최대 RCUBE_CAN_REASSEMBLY_MAX=446바이트). 호출부는 분할을 신경 쓰지 않는다. */
esp_err_t can_transport_send(uint8_t priority, uint8_t op_code,
                             uint8_t dst, const uint8_t *data, uint16_t len);

/* edge central(ECF=1)이 CAN 멤버를 기다리기 시작한다(기획서 7.4-4 [CMF=1 멤버]).
 *
 * 저장된 멤버 맵에서 CMF=CAN인 노드ID들을 기대 목록으로 잡고, 각 노드의
 * 부팅 알림(NodeAnnounce) 또는 하트비트를 받으면 발견 처리한다. 노드ID 순서대로
 * 발견되지 않아도 되며, 전부 모이면 완료 로그·멜로디를 낸다.
 * CAN 멤버가 없으면 아무것도 하지 않고 false를 돌려준다. */
bool can_transport_start_edge(void);

/* edge central이 기대하는 CAN 멤버 중 현재까지 발견된 수 / 전체 수. */
uint8_t can_transport_edge_found(void);
uint8_t can_transport_edge_expected(void);
