/*
 * rcube_mission — 미션코드 컨테이너·저장소·시퀀서 (기획서 8장 / 확장 규격 §2.5)
 * ------------------------------------------------------------------
 * TYPE=1(데이터 테이블)까지 구현한다. 기획서 12.4가 "데이터 테이블 → C 구현,
 * 파이썬 없이 항상 동작"이라고 못 박았으므로 이것이 기본 실행기다. 파이썬(TYPE=3)은
 * 컨테이너의 type 필드만 늘리면 얹을 수 있도록 열어 둔다.
 *
 * ★ 저장은 파일시스템 없이 storage 파티션의 슬롯 2개를 직접 쓴다.
 *   미션은 불투명한 바이트 덩어리라 디렉터리가 필요 없고, 파일시스템을 얹으면
 *   마운트 실패·조각화 같은 새 실패 모드만 늘어난다. 업로드는 항상 비활성 슬롯에
 *   쓰고 CRC를 통과했을 때만 활성 슬롯 포인터를 바꾼다 — 업로드 중 전원이 끊겨도
 *   기존 미션이 살아남는다.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define RCUBE_MISSION_MAGIC      "RCMS"
#define RCUBE_MISSION_VER        1u
#define RCUBE_MISSION_HDR_LEN    32u
#define RCUBE_MISSION_NAME_LEN   12u

/* 컨테이너 type */
#define RCUBE_MISSION_TYPE_TABLE 1u   /* 키프레임 배열(구현) */
#define RCUBE_MISSION_TYPE_BYTECODE 2u/* 예약 */
#define RCUBE_MISSION_TYPE_MPY   3u   /* 예약 — MicroPython .mpy */

/* flags */
#define RCUBE_MISSION_FLAG_REPEAT 0x01u

/* TYPE=1 키프레임 레코드(8바이트 고정폭). CAN 데이터필드와 폭이 같다. */
#define RCUBE_MISSION_REC_LEN 8u
#define RCUBE_MISSION_KIND_ANGLE 0u
/* KIND=1 부저 톤. value = [31:16] 주파수Hz(0=쉼) | [15:0] 길이ms.
 * 모터 없이도 유닛 전체의 동시 동작을 눈·귀로 검증할 수 있어, 독립로봇유닛
 * 브링업(7.4-6)의 첫 미션으로 쓴다. */
#define RCUBE_MISSION_KIND_TONE  1u
#define RCUBE_MISSION_TONE_FREQ(v) ((uint16_t)(((uint32_t)(v) >> 16) & 0xFFFFu))
#define RCUBE_MISSION_TONE_DUR(v)  ((uint16_t)((uint32_t)(v) & 0xFFFFu))

/* 슬롯 1개 크기. storage 파티션(9.9MB)에 2개를 잡는다. */
#define RCUBE_MISSION_SLOT_SIZE (512u * 1024u)

/* 실행 상태(F3 회신의 state) */
typedef enum {
    RCUBE_MISSION_NONE = 0,
    RCUBE_MISSION_LOADED = 1,
    RCUBE_MISSION_RUNNING = 2,
} rcube_mission_state_t;

/* MissionControl(0xF9) action */
#define RCUBE_MISSION_RUN     0x01u
#define RCUBE_MISSION_STOP    0x02u
#define RCUBE_MISSION_PAUSE   0x03u
#define RCUBE_MISSION_RESUME  0x04u

/* 파티션 확인 + 활성 슬롯 로드. app_main 초기화에서 1회. */
esp_err_t rcube_mission_init(void);

/* ---- 업로드 (F0/F1/F2) ---- */
uint8_t rcube_mission_begin(uint8_t type, uint32_t total_len, uint32_t crc32,
                            uint32_t unit_sig);
uint8_t rcube_mission_chunk(uint16_t seq, const uint8_t *data, uint16_t len);
uint8_t rcube_mission_commit(void);

/* ---- 조회·삭제 (F3/F4) ---- */
/* out에 [state][헤더 32B] 33바이트를 채운다. 반환값은 채운 길이. */
uint8_t rcube_mission_info(uint8_t *out, uint8_t cap);
uint8_t rcube_mission_delete(void);

/* ---- 실행 (F9) ---- */
uint8_t rcube_mission_control(uint8_t action);
rcube_mission_state_t rcube_mission_state(void);

/* 이 유닛의 서명. 컨테이너 unit_sig와 대조한다(0이면 검사 생략). */
uint32_t rcube_mission_unit_sig(void);

/* ---- 연결 완료 → 자동 실행 (기획서 7.4-6) ----
 * edge central의 한 통신 분기(cmf: RCUBE_MEMBER_BLE / RCUBE_MEMBER_CAN)가 담당 멤버를
 * 전부 연결했을 때 그 전송계층이 호출한다. 맵이 요구하는 분기가 모두 모이면 적재된
 * 미션을 스스로 Run 시킨다(한 번만). ECF=0이거나 미션이 없으면 아무 일도 하지 않는다. */
void rcube_mission_branch_ready(uint8_t cmf);
