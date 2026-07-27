#include "rcube_melody.h"

/* buzzer_sound.c piano_scale[17] 그대로. A3(라)~C6(도). idx2=C4(기준 도). */
const uint16_t rcube_piano_scale[RCUBE_PIANO_SCALE_LEN] = {
    220, 247, 262, 294, 330, 349, 392, 440, 494,
    523, 587, 659, 698, 784, 880, 988, 1046,
};

/* ── START (부팅/시작음): B3, D4, B4 ── */
static const rcube_note_t START_NOTES[] = {
    {1, 500}, {3, 500}, {8, 1000},
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

/* ── 노드ID 부팅음 (기획서 5장: ID1 C4, ID2 D4 … 규칙) ──
 * piano_scale 인덱스 = 노드ID + 1 (ID1→idx2=C4 … ID8→idx9=C5).
 * "0.5초 연주, 0.5초 쉬고 0.5초 연주" 2회 연주. */
#define NODE_NOTES(idx) { {(idx), 500}, {RCUBE_NOTE_REST, 500}, {(idx), 500} }
static const rcube_note_t NODE1_NOTES[] = NODE_NOTES(2);   /* C4 */
static const rcube_note_t NODE2_NOTES[] = NODE_NOTES(3);   /* D4 */
static const rcube_note_t NODE3_NOTES[] = NODE_NOTES(4);   /* E4 */
static const rcube_note_t NODE4_NOTES[] = NODE_NOTES(5);   /* F4 */
static const rcube_note_t NODE5_NOTES[] = NODE_NOTES(6);   /* G4 */
static const rcube_note_t NODE6_NOTES[] = NODE_NOTES(7);   /* A4 */
static const rcube_note_t NODE7_NOTES[] = NODE_NOTES(8);   /* B4 */
static const rcube_note_t NODE8_NOTES[] = NODE_NOTES(9);   /* C5 */

#define MELODY(arr, nm) { .notes = (arr), .count = (uint8_t)(sizeof(arr)/sizeof((arr)[0])), .name = (nm) }

static const rcube_melody_t MELODIES[RCUBE_MELODY_COUNT] = {
    [RCUBE_MELODY_START]          = MELODY(START_NOTES, "START"),
    [RCUBE_MELODY_BUTTON_PRESSED] = MELODY(BUTTON_NOTES, "BUTTON_PRESSED"),
    [RCUBE_MELODY_LINK]           = MELODY(LINK_NOTES, "LINK"),
    [RCUBE_MELODY_LINK_COMPLETED] = MELODY(LINK_COMPLETED_NOTES, "LINKCOMPLETED"),
    [RCUBE_MELODY_LINK_WAIT]      = MELODY(LINK_WAIT_NOTES, "LINKWAIT"),
    [RCUBE_MELODY_EDGE]           = MELODY(EDGE_NOTES, "EDGE"),
    [RCUBE_MELODY_NODE_1]         = MELODY(NODE1_NOTES, "NODE1"),
    [RCUBE_MELODY_NODE_2]         = MELODY(NODE2_NOTES, "NODE2"),
    [RCUBE_MELODY_NODE_3]         = MELODY(NODE3_NOTES, "NODE3"),
    [RCUBE_MELODY_NODE_4]         = MELODY(NODE4_NOTES, "NODE4"),
    [RCUBE_MELODY_NODE_5]         = MELODY(NODE5_NOTES, "NODE5"),
    [RCUBE_MELODY_NODE_6]         = MELODY(NODE6_NOTES, "NODE6"),
    [RCUBE_MELODY_NODE_7]         = MELODY(NODE7_NOTES, "NODE7"),
    [RCUBE_MELODY_NODE_8]         = MELODY(NODE8_NOTES, "NODE8"),
};

const rcube_melody_t *rcube_melody(rcube_melody_id_t id)
{
    if (id < 0 || id >= RCUBE_MELODY_COUNT) {
        return NULL;
    }
    return &MELODIES[id];
}

rcube_melody_id_t rcube_melody_node_id(uint8_t node_id)
{
    if (node_id < 1 || node_id > 8) {
        return RCUBE_MELODY_START;   /* 미할당(비고정형) → 디폴트 켜짐 멜로디 */
    }
    return (rcube_melody_id_t)(RCUBE_MELODY_NODE_1 + (node_id - 1));
}
