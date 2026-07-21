# 앱 진입 컴포넌트. Phase가 진행되며 소스가 추가된다.
# (예정 컴포넌트: motor_uart, ble_transport, can_transport, protocol_core,
#  role_member, role_edge_central, motion_core 등 — components/ 아래로 분리)
idf_component_register(
    SRCS
        "main.c"
    INCLUDE_DIRS
        "."
)
