#!/bin/bash

# ============================================================
#  vive/ — Vive tracker + Manus teleop master
# ============================================================
# Runnable standalone (cd vive && ./build.sh). Resolves its own dir so the
# ../../ paths below (ManusSDK) stay relative to kida's parent regardless of
# the caller's CWD.

cd "$(dirname "$0")"

MANUS_DIR="${MANUS_DIR:-../../ManusSDK/3.1.1}"
OPENVR_LIB_DIR="${OPENVR_LIB_DIR:-$HOME/.local/share/Steam/steamapps/common/SteamVR/bin/linux64}"

#vive + manus teleop master (zmq PUSH to slave/logger). Same source, twice:
#   vmaster      detach ('a') remembers the pose, re-attach resumes there
#   vmaster-gui  detach keeps the home bias, so re-attach after 'i' (init)
#                does not snap the arm back to the pre-init pose.
#                kida-gui.py launches this one.
build() {   # $1 = output binary, $2.. = extra flags
    local out=$1; shift
    g++ -O2 -g -std=c++17 -W -Wall "$@" -o "$out" vmaster.cpp \
        -I"$MANUS_DIR/include" -I. \
        -L"$MANUS_DIR/lib" -lManusSDK_Integrated -Wl,-rpath,"$MANUS_DIR/lib" \
        -L"$OPENVR_LIB_DIR" -lopenvr_api -Wl,-rpath,"$OPENVR_LIB_DIR" \
        -lzmq -lncurses -pthread
}

build vmaster
build vmaster-gui -DVMASTER_GUI
