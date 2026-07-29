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
    /* 그룹번호 알림. 음이 런타임(그룹번호)에 따라 정해지므로 고정 테이블이 없고,
     * rcube_melody_group_notes()로 그때그때 만든다. rcube_melody()는 NULL을 준다. */
    RCUBE_MELODY_GROUP,
    RCUBE_MELODY_COUNT
} rcube_melody_id_t;

/* id에 해당하는 멜로디를 돌려준다. 범위를 벗어나거나 GROUP이면 NULL. */
const rcube_melody_t *rcube_melody(rcube_melody_id_t id);

/* ---- 그룹번호 알림음 (기획서 5장 [소리 규칙], 2026-07-29 재정의) --------
 * 그룹 LED 색상표와 같은 자릿수를 음으로 옮긴다.
 *   0 흰색 C4 · 1 Red D4 · 2 Green E4 · 3 Blue F4 · 4 Cyan G4
 *   5 Magenta A4 · 6 Yellow B4 · 7 Violet C5 · 8 Orange D5 · 9 연한Red E5
 * 즉 piano_scale 인덱스 = 자릿수 + 2.
 *
 * ※ 노드ID는 소리에 반영하지 않는다 — 노드ID는 ID 표시용 칼라 LED 전용이다.
 *   같은 값을 소리와 LED로 이중 표시하면 한쪽만 바뀌었을 때 어긋나기 때문이다. */
#define RCUBE_GROUP_NOTE_HI_MS 200   /* 십의 자리 */
#define RCUBE_GROUP_NOTE_LO_MS 600   /* 일의 자리 */
#define RCUBE_GROUP_NOTE_COUNT 2

/* 색 자릿수(0~9) → piano_scale 인덱스. */
uint8_t rcube_melody_digit_note(uint8_t digit);

/* 그룹번호(0~99)를 두 음으로 전개한다. buf는 최소 RCUBE_GROUP_NOTE_COUNT칸.
 * 만든 음 개수를 반환(공간이 모자라면 0). */
uint8_t rcube_melody_group_notes(uint8_t group_id, rcube_note_t *buf, uint8_t cap);
