"""
melody_do_re_mi.py — 독립로봇유닛 브링업 미션 (기획서 7.4-6 검증용)

미션 내용:
    모든 큐브의 연결이 완료되면(= edge central이 이 미션을 시작하는 시점) 3초를
    기다렸다가, 유닛의 모든 큐브가 동시에
        0.25초 도 → 0.5초 쉼 → 0.25초 레 → 0.5초 쉼 → 0.25초 미
    를 연주한다.

모터 없이 부저만 쓰므로, "독립로봇유닛이 스스로 미션을 실행하고 edge central이 전
큐브를 동시에 제어한다"는 것만 따로 떼어 귀로 검증할 수 있다. 세 큐브의 소리가 한
소리로 들리면 성공이고, 어긋나 들리면 분배 경로(CAN/BLE)의 지연 문제다.

컴파일(파이썬 → 미션코드):
    cd app
    python -m rcube.mission missions/melody_do_re_mi.py -o missions/melody_do_re_mi.rcm
업로드: GUI에서 "독립로봇유닛" 체크 후 저장하면 이 파일을 리드 큐브(노드01)에 올린다.

쓸 수 있는 명령은 rcube/mission.py 상단 주석 참조(wait·play·at·angle·NODES).
"""

NAME = "DOREMI"

# 연결 완료 직후에는 유닛구성완료 멜로디가 아직 울리고 있다. 3초는 그 소리와 겹치지
# 않게 두는 간격이기도 하다.
wait(3.0)

play("도", 0.25)    # nodes를 안 주면 유닛의 모든 큐브가 동시에 낸다
wait(0.5)
play("레", 0.25)
wait(0.5)
play("미", 0.25)
