"""
chord_do_mi_sol.py — 독립로봇유닛 미션 2 (큐브별 다른 음 = 화음)

미션 내용:
    모든 큐브의 연결이 완료되면 3초를 기다렸다가,
        큐브1 = 도(C4 262Hz), 큐브2 = 미(E4 330Hz), 큐브3 = 솔(G4 392Hz)
    를 **동시에 1초** 동안 낸다.

앞선 melody_do_re_mi(전 큐브가 같은 음을 차례로)와 달리 큐브마다 음이 다르므로,
edge central이 각 큐브에 서로 다른 명령을 같은 시각에 분배하는지까지 귀로 확인된다.
세 음이 한 화음(도-미-솔 = C 장3화음)으로 들리면 성공이고, 한 음이라도 빠지거나
늦으면 그 큐브의 분배 경로 문제다.

컴파일(파이썬 → 미션코드):
    cd app
    python -m rcube.mission missions/chord_do_mi_sol.py --nodes 3 --cmf can \
           -o missions/chord_do_mi_sol.rcm
업로드: GUI에서 "독립로봇유닛" 체크 후 저장하면 이 파일을 리드 큐브(노드01)에 올린다.
"""

NAME = "DOMISOL"

wait(3.0)

chord({1: "도", 2: "미", 3: "솔"}, 1.0)
