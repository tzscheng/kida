#!/bin/bash

# ============================================================
#  vive/ — Vive tracker + Manus teleop master
# ============================================================
# Runnable standalone (cd vive && ./build.sh). Resolves its own dir so the
# ../../ paths below (fgx/manus, fg/dev/manus) stay relative to kida's parent
# regardless of the caller's CWD.

cd "$(dirname "$0")"

MANUS_DIR="../../fgx/manus/ManusSDK"
OPENVR_LIB_DIR="${OPENVR_LIB_DIR:-$HOME/.local/share/Steam/steamapps/common/SteamVR/bin/linux64}"

#vive + manus teleop master (zmq PUSH to slave/logger)
g++ -O2 -g -std=c++17 -W -Wall -o vmaster vmaster.cpp \
    -I"$MANUS_DIR/include" -I. \
    -L"$MANUS_DIR/lib" -lManusSDK_Integrated -Wl,-rpath,"$MANUS_DIR/lib" \
    -L"$OPENVR_LIB_DIR" -lopenvr_api -Wl,-rpath,"$OPENVR_LIB_DIR" \
    -lzmq -lncurses -pthread
