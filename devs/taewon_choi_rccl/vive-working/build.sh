#!/bin/bash

# ============================================================
#  vive-working/ — vive 리팩터 작업본 (제자리 빌드)
# ============================================================
# 원본 vive/ 를 건드리지 않고 이 폴더에서 그대로 빌드/실행합니다.
#
#   ./build.sh                 -> vhand
#   MANUS_DIR=~/ManusSDK/3.0.1 ./build.sh
#
# vmaster 와 manus.h 는 vive/ 로 반영이 끝나서 여기엔 없습니다. 여기 남은 건
# 아직 작업 중인 vhand 뿐이고, 헤더는 -I../../../vive 로 원본을 씁니다.
#
# 이 폴더에 없는 헤더(manus.h, retarget.h)는 -I로 원본 vive/ 를 참조합니다.
# 쌍따옴표 include는 소스가 있는 디렉터리를 먼저 뒤지므로, 나중에 이 폴더에
# retarget.h 사본을 두면 그쪽이 자동으로 우선합니다(원본은 그대로).
# calib 은 manus.h 가 실행 파일 위치 기준으로 찾고(/proc/self/exe), logger 는 아직
# cwd 기준입니다. 둘 다 원본 vive/ 로 심볼릭 링크를 걸어 뒀습니다.

cd "$(dirname "$0")"

VIVE=../../../vive          # 원본 vive/ (retarget.h, openvr/, calib/, logger)

# ManusSDK 위치. vive-working은 원본 vive/ 보다 두 단계 깊고(devs/<이름>/), 이
# 체크아웃은 ~/kida가 아니라 ~/Desktop/kida라 ../ 가 다섯 개입니다
# (vive-working -> taewon_choi_rccl -> devs -> kida -> Desktop -> ~).
# 저장소를 옮겼으면 환경변수로 주세요: MANUS_DIR=~/ManusSDK/3.0.1 ./build.sh
MANUS_DIR="${MANUS_DIR:-../../../../../ManusSDK/3.0.1}"
if [ ! -d "$MANUS_DIR" ]; then
    echo "ManusSDK를 찾지 못했습니다: $MANUS_DIR" >&2
    echo "  MANUS_DIR=/path/to/ManusSDK/3.0.1 ./build.sh" >&2
    exit 1
fi
# rpath에는 절대경로를 박습니다. 상대 rpath는 실행 시 cwd 기준으로 풀려서
# 다른 디렉터리에서 ./vive-working/vmaster 를 띄우면 .so를 못 찾습니다.
MANUS_DIR="$(cd "$MANUS_DIR" && pwd)"
echo "MANUS_DIR=$MANUS_DIR"

OPENVR_LIB_DIR="${OPENVR_LIB_DIR:-$HOME/.local/share/Steam/steamapps/common/SteamVR/bin/linux64}"

#glove-only master for retargeting work (no OpenVR: it never touches a tracker)
g++ -O2 -g -std=c++17 -W -Wall -o vhand vhand.cpp \
    -I"$MANUS_DIR/include" -I. -I"$VIVE" \
    -L"$MANUS_DIR/lib" -lManusSDK_Integrated -Wl,-rpath,"$MANUS_DIR/lib" \
    -lzmq -lncurses -pthread
