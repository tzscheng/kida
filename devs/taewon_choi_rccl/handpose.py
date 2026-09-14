# DG-5F 손 사전 정의 자세(프리셋) 표.
#
# 값은 손 벤더 GUI의 "Pose #N > Target Joint"를 그대로 옮긴 **도(deg)** 단위이고,
# 관절 순서도 그 GUI와 같습니다. 이 순서는 vmaster / dg5.py / eio-dg5s가 공유하는
# 20관절 레이아웃(finger*4 + j)과 동일합니다:
#
#     0.. 3  thumb1..4     (Finger 1, 사람 표기 1..4번 관절)
#     4.. 7  index1..4     (Finger 2, 5..8번)
#     8..11  middle1..4    (Finger 3, 9..12번)
#    12..15  ring1..4      (Finger 4, 13..16번)
#    16..19  little1..4    (Finger 5, 17..20번)
#
# wire(=dg5.Controller가 받는 'joint'/'hold' 명령) 단위는 rad입니다. eio-dg5s.c가
# rad -> deg로 되돌려 MoveServoJoint()에 넣으므로, 여기 deg 값에 pi/180만 곱하면
# 손 GUI에서 본 자세가 그대로 재현됩니다.
#
# 좌/우는 별도로 적습니다. 미러는 단순 부호 반전이 아니라 엄지 쪽(0..4)만
# 뒤집히므로(dg5.Controller.home과 같은 패턴), 자동 변환하지 않고 벤더 GUI에서
# 각각 읽은 값을 그대로 둡니다.
#
# 새 자세를 추가하려면 벤더 GUI에서 Pose를 저장하고 Target Joint 20개를 아래
# 형식으로 붙여 넣기만 하면 됩니다. kida-gui.py의 콤보 상자가 자동으로 읽습니다.

from math import pi

DEG2RAD = pi / 180.0
NJ = 20

POSES = {
    # 관절 사전 정의.pdf — Pose #4 (1쪽: 왼손, 2쪽: 오른손)
    'pose4': {
        'left':  [0,  95, -35, -25,   5, 50, 10, 35,   0, 45, 25, 30,
                  0, 80, 70, 50,   0, 0, 85, 85],
        'right': [0, -95,  35,  25,  -5, 50, 10, 35,   0, 45, 25, 30,
                  0, 80, 70, 50,   0, 0, 85, 85],
    },
}


def names():
    """콤보 상자에 넣을 프리셋 이름 목록."""
    return list(POSES)


def deg(name, side):
    """프리셋 20관절 값 (deg). side는 'left' 또는 'right'."""
    v = POSES[name][side]
    if len(v) != NJ:
        raise ValueError('%s/%s: 관절 수가 %d개 (20개여야 함)' % (name, side, len(v)))
    return list(v)


def rad(name, side):
    """프리셋 20관절 값 (rad) — 그대로 'hold' 명령에 실어 보내면 됩니다."""
    return [x * DEG2RAD for x in deg(name, side)]
