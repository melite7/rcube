/*
 * motor_uart — 모터제어보드(CubeMars AK 시리즈) UART 링크
 * ------------------------------------------------------------------
 * 이 구간은 R큐브 프로토콜이 아니라 드라이버 벤더 규격이다.
 *   출처: docs/모터제어명령규격.pdf 4.3.2 (Serial Port Message Protocol)
 *   요약: docs/R큐브_프로토콜_확장_20260728.md 부록
 *
 *   [0xAA][LEN][ID][data…][CRC hi][CRC lo][0xBB]
 *     LEN = ID+data 길이(헤더·CRC·테일 제외)
 *     CRC = CRC-16/XMODEM (poly 0x1021, init 0x0000), 대상 = ID+data
 *
 * 계층 구분(기획서 11장): ESP32는 20~50Hz로 "목표+도달시간"만 내려보내고, 1kHz 이상
 * 서보 루프와 키프레임 사이 보간은 드라이버가 전담한다. 이 파일은 그 하행/상행
 * 프레이밍만 담당하며 주기·버퍼·T0는 motion_core의 몫이다.
 *
 * 핀맵(docs/ESP32_핀맵_펌웨어참조_20260724.md):
 *   IO17=MC_UART_TX  IO18=MC_UART_RX  IO8=MC_FAULT(입력)
 *   IO35=MC_GATE_EN  IO36=MC_BOOT0    IO37=MC_nRST(Active-Low)
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* ---- 드라이버 명령 ID (매뉴얼 COMM_PACKET_ID) ---- */
#define MOTOR_CMD_GET_VALUES     0x45u  /* 상태 1회 회신 */
#define MOTOR_CMD_SET_DUTY       0x46u  /* duty      ×100000 */
#define MOTOR_CMD_SET_CURRENT    0x47u  /* A         ×1000 */
#define MOTOR_CMD_SET_CUR_BRAKE  0x48u  /* A         ×1000 */
#define MOTOR_CMD_SET_RPM        0x49u  /* ERPM      ×1 */
#define MOTOR_CMD_SET_POS        0x4Au  /* 도(°)     ×1000000  ★ */
#define MOTOR_CMD_SET_HANDBRAKE  0x4Bu  /* A         ×1000 */
#define MOTOR_CMD_SET_DETECT     0x4Cu  /* 위치 10ms 주기 피드백 */
#define MOTOR_CMD_ROTOR_POSITION 0x57u  /* 위치 회신(상행) */
#define MOTOR_CMD_SET_POS_SPD    0x3Cu  /* 도 ×1000 + ERPM + 가속  ★ */
#define MOTOR_CMD_SET_POS_MULTI  0x3Du  /* 다회전 모드 */
#define MOTOR_CMD_SET_POS_SINGLE 0x3Eu  /* 단회전 모드 */
#define MOTOR_CMD_SET_POS_ORIGIN 0x40u  /* 원점 설정 */

/* ★ SET_POS는 도×1,000,000, SET_POS_SPD는 도×1,000으로 스케일이 1000배 다르다.
 *   벤더 문서 4.3.2.2의 5번과 7번이 실제로 그렇게 정의돼 있다. 섞어 쓰면 축이
 *   1000배로 튀므로 아래 빌더를 통해서만 호출한다. */

/* 드라이버가 올려주는 상태(GET_VALUES 0x45 회신에서 추출). */
typedef struct {
    bool     valid;            /* 한 번이라도 유효 프레임을 받았는가 */
    int64_t  updated_us;       /* 마지막 갱신 시각(esp_timer) */
    float    mos_temp_c;
    float    motor_temp_c;
    float    current_a;        /* 출력 전류 */
    float    input_voltage_v;
    int32_t  erpm;             /* 전기적 회전수 */
    float    position_deg;     /* 외부 루프 위치 */
    uint8_t  error_code;       /* 0=정상 */
} motor_status_t;

/* UART/GPIO 초기화. 부팅 시 게이트는 반드시 "모터 차단"으로 먼저 잡는다. */
esp_err_t motor_uart_init(void);

/* 모터 게이트(IO35). 안전상태=차단(false). */
void motor_uart_set_gate(bool enable);
bool motor_uart_gate_enabled(void);

/* IO8 FAULT 입력이 활성인지(모터보드가 폴트를 알림). */
bool motor_uart_fault_asserted(void);

/* ---- 하행 명령 ---------------------------------------------------- */

/* 위치-속도 루프(0x3C). 기획서 11장의 "목표 위치 + 도달시간" 키프레임에 대응한다.
 *   deg      : 목표 각도(도)
 *   erpm     : 목표 전기적 회전수. 0이면 드라이버 기본값
 *   acc_erpm : 가속도(ERPM/s). 0이면 드라이버 기본값 */
esp_err_t motor_uart_set_pos_spd(float deg, int32_t erpm, int32_t acc_erpm);

esp_err_t motor_uart_set_pos(float deg);          /* 위치 루프(0x4A) */
esp_err_t motor_uart_set_rpm(int32_t erpm);       /* 속도 루프(0x49) */
esp_err_t motor_uart_set_current(float amps);     /* 전류 루프(0x47) */
esp_err_t motor_uart_set_duty(float duty);        /* duty(0x46) */
esp_err_t motor_uart_set_origin(uint8_t mode);    /* 원점 설정(0x40) */
esp_err_t motor_uart_request_status(void);        /* GET_VALUES(0x45) */
esp_err_t motor_uart_set_detect(bool on);         /* 위치 10ms 피드백(0x4C) */

/* 안전 정지: 전류 0 + 게이트 차단. 폴트/E-Stop 경로에서 호출한다. */
esp_err_t motor_uart_safe_stop(void);

/* ---- 상행 ---------------------------------------------------------- */

/* 최신 상태 스냅샷. 아직 수신 전이면 valid=false. */
void motor_uart_get_status(motor_status_t *out);

/* 마지막 유효 수신 이후 경과 ms. 수신 이력이 없으면 INT64_MAX. */
int64_t motor_uart_silence_ms(void);

/* ---- 검증 ---------------------------------------------------------- */

/* 매뉴얼 4.4.2 예제 프레임으로 CRC·스케일 자체 검증. 실패 개수를 반환(0=정상).
 * 모터보드가 없어도 프레이밍이 맞는지 부팅 시 확인할 수 있다. */
int motor_uart_selftest(void);

/* 루프백 진단: IO17↔IO18을 점퍼로 물린 상태에서 프레임을 보내고 되돌아오는지 본다.
 * 모터보드 없이 UART 경로·파서를 검증하는 용도. 성공 시 true. */
bool motor_uart_loopback_test(void);
