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

#define MELODY(arr, nm) { .notes = (arr), .count = (uint8_t)(sizeof(arr)/sizeof((arr)[0])), .name = (nm) }

static const rcube_melody_t MELODIES[RCUBE_MELODY_COUNT] = {
    [RCUBE_MELODY_START]          = MELODY(START_NOTES, "START"),
    [RCUBE_MELODY_BUTTON_PRESSED] = MELODY(BUTTON_NOTES, "BUTTON_PRESSED"),
    [RCUBE_MELODY_LINK]           = MELODY(LINK_NOTES, "LINK"),
    [RCUBE_MELODY_LINK_COMPLETED] = MELODY(LINK_COMPLETED_NOTES, "LINKCOMPLETED"),
};

const rcube_melody_t *rcube_melody(rcube_melody_id_t id)
{
    if (id < 0 || id >= RCUBE_MELODY_COUNT) {
        return NULL;
    }
    return &MELODIES[id];
}
