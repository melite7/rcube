# shared-protocol — R큐브 BLE/CAN 응용계층 단일 소스

OpCode(0xA0~0xFF), NACK ResultCode, 패킷 구조체, TargetId 규약(0x01~08 / 0xFE / 0xFF).
BLE와 CAN이 응용계층을 완전 공유하며, esp32 펌웨어와 app이 모두 이 정의를 참조한다.
프로토콜 표(xlsx)에서 스크립트로 생성하는 것을 원칙으로 한다(수기 이전 금지).
