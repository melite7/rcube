#include "rcube_melody.h"

/* buzzer_sound.c piano_scale[17] 그대로. A3(라)~C6(도). idx2=C4(기준 도). */
const uint16_t rcube_piano_scale[RCUBE_PIANO_SCALE_LEN] = {
    220, 247, 262, 294, 330, 349, 392, 440, 494,
    523, 587, 659, 698, 784, 880, 988, 1046,
};

/* ── START (부팅/시작음): B3, D4, B4 ──
 * xlsx '멜로디 데이터'는 0.5 / 0.5 / 1.0초(총 2초)인데, 재생 속도를 2배로 하라는
 * 요청(2026-07-30)에 따라 길이를 절반으로 줄였다(총 1초). 음과 순서는 그대로다.
 * ★ 따라서 이 항목은 xlsx와 의도적으로 다르다 — 대조 스크립트가 START를 불일치로
 *   표시하는 것이 정상이다. */
static const rcube_note_t START_NOTES[] = {
    {1, 250}, {3, 250}, {8, 500},
};

/* ── DISCONNECT (BLE 연결 끊김): START를 역순으로 ──
 * 연결됨(상행 B3→D4→B4)과 끊김(하행 B4→D4→B3)이 귀로 바로 구분된다.
 * 세 음 모두 0.25초로 균등하게 둔다(2026-07-30) — 끝음을 늘이지 않아 짧고 단호하다. */
static const rcube_note_t DISCONNECT_NOTES[] = {
    {8, 250}, {3, 250}, {1, 250},
};

/* ── BUTTON_PRESSED (버튼 눌림): E4, C4, F4, C4 ── */
static const rcube_note_t BUTTON_NOTES[] = {
    {4, 50}, {2, 80}, {5, 50}, {2, 80},
};

/* ── LINK (큐브 1개 연결): D4, C4, E4, C4, F4, C4 ── */
static const rcube_note_t LINK_NOTES[] = {
    {3, 50}, {2, 80}, {4, 50}, {2, 80}, {5, 50}, {2, 80},
};

/* ── LINKCOMPLETED (전체 연결완료): 하행 후 상행 스케일 20음 ── */
static const rcube_note_t LINK_COMPLETED_NOTES[] = {
    {7, 80}, {6, 80}, {5, 80}, {4, 80}, {3, 80},
    {2, 50}, {3, 50}, {4, 50}, {5, 50}, {6, 50},
    {7, 50}, {8, 50}, {9, 50}, {10, 50}, {11, 50},
    {2, 50}, {3, 50}, {4, 50}, {5, 50}, {6, 50},
};

/* ── LINKWAIT (연결모드 진입 = 연결대기): C4, G4 상행 2음 ──
 * 기획서 5장 [소리 규칙]의 "연결대기 멜로디". 버튼음(BUTTON)과 구분되도록 상행. */
static const rcube_note_t LINK_WAIT_NOTES[] = {
    {2, 120}, {6, 200},
};

/* ── EDGE (edge central 연결모드 진입): C4, E4, G4, C5 상행 아르페지오 ──
 * 연결대기(2음)와 뚜렷이 구분되도록 4음 상행으로 둔다. */
static const rcube_note_t EDGE_NOTES[] = {
    {2, 110}, {4, 110}, {6, 110}, {9, 220},
};

#define MELODY(arr, nm) { .notes = (arr), .count = (uint8_t)(sizeof(arr)/sizeof((arr)[0])), .name = (nm) }

static const rcube_melody_t MELODIES[RCUBE_MELODY_COUNT] = {
    [RCUBE_MELODY_START]          = MELODY(START_NOTES, "START"),
    [RCUBE_MELODY_BUTTON_PRESSED] = MELODY(BUTTON_NOTES, "BUTTON_PRESSED"),
    [RCUBE_MELODY_LINK]           = MELODY(LINK_NOTES, "LINK"),
    [RCUBE_MELODY_LINK_COMPLETED] = MELODY(LINK_COMPLETED_NOTES, "LINKCOMPLETED"),
    [RCUBE_MELODY_LINK_WAIT]      = MELODY(LINK_WAIT_NOTES, "LINKWAIT"),
    [RCUBE_MELODY_EDGE]           = MELODY(EDGE_NOTES, "EDGE"),
    [RCUBE_MELODY_DISCONNECT]     = MELODY(DISCONNECT_NOTES, "DISCONNECT"),
    /* RCUBE_MELODY_GROUP은 런타임 생성이라 테이블이 비어 있다(notes=NULL). */
};

const rcube_melody_t *rcube_melody(rcube_melody_id_t id)
{
    if (id < 0 || id >= RCUBE_MELODY_COUNT) {
        return NULL;
    }
    const rcube_melody_t *m = &MELODIES[id];
    return (m->notes != NULL) ? m : NULL;
}

uint8_t rcube_melody_digit_note(uint8_t digit)
{
    if (digit > 9) {
        digit = 0;
    }
    /* 0=흰색→idx2(C4) … 9=연한Red→idx11(E5). 그룹 LED 색상표와 같은 순서. */
    return (uint8_t)(digit + 2);
}

uint8_t rcube_melody_group_notes(uint8_t group_id, rcube_note_t *buf, uint8_t cap)
{
    if (buf == NULL || cap < RCUBE_GROUP_NOTE_COUNT) {
        return 0;
    }
    if (group_id > 99) {
        group_id = 99;
    }
    buf[0].note_idx = rcube_melody_digit_note((uint8_t)(group_id / 10));
    buf[0].dur_ms = RCUBE_GROUP_NOTE_HI_MS;
    buf[1].note_idx = rcube_melody_digit_note((uint8_t)(group_id % 10));
    buf[1].dur_ms = RCUBE_GROUP_NOTE_LO_MS;
    return RCUBE_GROUP_NOTE_COUNT;
}
