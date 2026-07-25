# 앱 진입 컴포넌트. Phase가 진행되며 소스가 추가된다.
# (예정 컴포넌트: motor_uart, ble_transport, can_transport, protocol_core,
#  role_member, role_edge_central, motion_core 등 — components/ 아래로 분리)
#
# REQUIRES:
#   bt        : NimBLE 호스트/컨트롤러
#   nvs_flash : BLE PHY 캘리브레이션 저장
#   driver    : GPIO(BOOT 버튼)
# (led_strip 관리형 컴포넌트는 idf_component.yml 의존성으로 자동 포함)
idf_component_register(
    SRCS
        "main.c"
        "board_led.c"
        "ble_rcube.c"
        "ble_multirole.c"
        "rcube_cmd.c"
        "rcube_config.c"
        "rcube_melody.c"
        "rcube_buzzer.c"
        "rcube_status.c"
        "bmi088.c"
    INCLUDE_DIRS
        "."
        # 프로토콜 단일 소스(opcode/주소/결과코드). app 과 동일 헤더를 공유.
        "../../shared-protocol"
    REQUIRES
        bt
        nvs_flash
        driver
        spi_flash
        esp_hw_support
)
