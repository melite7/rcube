# shared-protocol — R큐브 BLE/CAN 응용계층 단일 소스

OpCode(0xA0~0xFF), NACK ResultCode, 패킷 구조체, TargetId 규약(0x01~08 / 0xFE / 0xFF).
BLE와 CAN이 응용계층을 완전 공유하며, `esp32` 펌웨어와 `app`이 모두 이 정의를 참조한다.

## ★ 자동 생성 — 직접 수정 금지

이 폴더의 코드 파일은 **두 소스에서 스크립트로 생성**한다.
손으로 값을 옮기지 않는다(로드맵 Phase 0 원칙 — 세 곳에 옮겨 적으면 불일치 버그).

| 소스 | 역할 |
|---|---|
| `docs/R큐브_프로토콜_*.xlsx` | 기준 문서(읽기전용). OpCode·ResultCode 원본 |
| `docs/R큐브_프로토콜_확장_*.md` | 이후 확정된 **신규 OpCode + 페이로드 규격** |

xlsx는 읽기전용 기준 문서라 손대지 않는다. 새 OpCode는 확장 문서의
`<!-- GEN:OPCODES -->` 표에 적으면 생성기가 병합한다. 이름·코드가 겹치면 생성이 실패한다
(조용한 덮어쓰기 금지). **페이로드 레이아웃은 코드로 생성되지 않으므로 확장 문서가 유일한
규격서**다 — 구현 전에 반드시 참조한다.

| 파일 | 내용 | 사용처 |
|---|---|---|
| `rcube_opcodes.h` | OpCode 60개 enum (`RCUBE_OP_*`) | esp32 |
| `rcube_resultcodes.h` | NACK ResultCode 18개 enum (`RCUBE_RC_*`) | esp32 |
| `rcube_protocol.h` | 표준 헤더 구조체 · 주소 규약 · CAN 29비트 ID 매크로 · 우선순위 | esp32 |
| `rcube_protocol.py` | 위 전부의 Python 등가물 (`OpCode`/`ResultCode`/`can_id`…) | app |

## 재생성 방법

프로토콜 xlsx를 수정한 뒤:

```
pip install openpyxl        # 최초 1회
python tools/gen_protocol.py           # 헤더/모듈 재생성
python tools/gen_protocol.py --check   # 생성 없이 파싱 결과만 확인
```

## 검증 (선택)

- C/C++: `gcc -std=c11 -Wall -Wextra -I. <test>.c` 로 구문 검증
- Python: `import rcube_protocol` 후 값 확인
- 두 언어의 `can_id()` 결과가 동일해야 한다(전송계층 무관 원칙).

## esp32에서 사용

`rcube_protocol.h` 하나만 include 하면 opcodes/resultcodes가 함께 딸려온다.
추후 `esp32/`에서 이 폴더를 IDF 컴포넌트로 참조하거나 include 경로에 추가한다.
