/*
 * rcube_melody — 내장 멜로디 데이터 (부저 재생용)
 * ------------------------------------------------------------------
 * 데이터 출처: docs/큐브_멜로디_데이터.xlsx (음/길이만 참조).
 *   - NoteIdx(0~16) → piano_scale[17] 주파수(Hz): A3(220)~C6(1046).
 *   - Amplitude는 내장곡 전부 0.9 고정(부저에서 duty로 반영).
 *   - opcode/전송규약은 R큐브 shared-protocol 기준(이 파일은 데이터만).
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

/* NoteIdx → 주파수(Hz). 인덱스 0~16 유효. */
#define RCUBE_PIANO_SCALE_LEN 17
extern const uint16_t rcube_piano_scale[RCUBE_PIANO_SCALE_LEN];

/* 쉼표(무음) 표시용 NoteIdx. */
#define RCUBE_NOTE_REST 0xFF

/* 한 음: piano_scale 인덱스 + 지속시간(ms). */
typedef struct {
    uint8_t  note_idx;   /* 0~16, 또는 RCUBE_NOTE_REST */
    uint16_t dur_ms;
} rcube_note_t;

typedef struct {
    const rcube_note_t *notes;
    uint8_t             count;
    const char         *name;
} rcube_melody_t;

/* 내장 멜로디 식별자. */
typedef enum {
    RCUBE_MELODY_START = 0,        /* 부팅음(노드ID 미할당 큐브) */
    RCUBE_MELODY_BUTTON_PRESSED,   /* 버튼 눌림 */
    RCUBE_MELODY_LINK,             /* 큐브 1개 연결 */
    RCUBE_MELODY_LINK_COMPLETED,   /* 전체 연결 완료(유닛구성완료) */
    RCUBE_MELODY_LINK_WAIT,        /* 연결모드 진입(연결대기) — 기획서 5장 [소리 규칙] */
    RCUBE_MELODY_EDGE,             /* 연결모드 진입(edge central 전용 엣지 멜로디) */
    /* 노드ID 부팅음: ID1=C4, ID2=D4 … ID8=C5 를 0.5s 연주·0.5s 쉼·0.5s 연주.
     * 반드시 연속(NODE_1..NODE_8)이어야 한다 — rcube_melody_node_id()가 오프셋으로 쓴다. */
    RCUBE_MELODY_NODE_1,
    RCUBE_MELODY_NODE_2,
    RCUBE_MELODY_NODE_3,
    RCUBE_MELODY_NODE_4,
    RCUBE_MELODY_NODE_5,
    RCUBE_MELODY_NODE_6,
    RCUBE_MELODY_NODE_7,
    RCUBE_MELODY_NODE_8,
    RCUBE_MELODY_COUNT
} rcube_melody_id_t;

/* id에 해당하는 멜로디를 돌려준다. 범위를 벗어나면 NULL. */
const rcube_melody_t *rcube_melody(rcube_melody_id_t id);

/* 부팅음 선택: 노드ID(1~8)가 있으면 그 노드 멜로디, 0/범위밖이면 START. */
rcube_melody_id_t rcube_melody_node_id(uint8_t node_id);
