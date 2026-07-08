#!/bin/bash

# ============================================================
#  eio — CAN/UDP hardware bridges
# ============================================================

# -Wl,-Bsymbolic-functions: resolve intra-.so function calls against our own
# definitions first. Without it, reset() -> step() goes through the PLT and
# matches libc's step() (System V regex API) instead of ours, causing a
# strlen(NULL) segfault. The name 'step' is too generic — keep this flag.

#io
gcc eio-kida.c -W -Wall -shared -o eio/eio-kida.so -fPIC -I../fg/dev/myact -Wl,-Bsymbolic-functions

gcc eio-single.c -W -Wall -shared -o eio/eio-single.so -fPIC  -I../fg/dev/myact -Wl,-Bsymbolic-functions

#dg5 — needs the DGSDK installed under /usr/local; skipped (keeping the tracked
#      eio/eio-dg5 binary) on machines without it
if [ -d /usr/local/include/DGSDK ]; then
    gcc -W -Wall -o eio/eio-dg5 eio-dg5.c -I/usr/local/include/DGSDK -lDGSDK
else
    echo "dg5: DGSDK not found — skipping eio-dg5 build"
fi

# ============================================================
#  rs2/ — RealSense multicam streamer
# ============================================================
(
cd rs2

#multicam receiver
g++ -std=c++17 -O2 mreceiver.cpp -o mreceiver `pkg-config --cflags --libs opencv4` -lzstd -lzmq -lpthread

#multicam sender
g++ -O2 -std=c++17 -pthread msender.cpp -o msender $(pkg-config --cflags --libs opencv4) -lrealsense2 -lzmq -lzstd -lturbojpeg

#video recorder (single cam)
g++ videorec.cpp -o videorec $(pkg-config --cflags --libs realsense2 opencv4)
)

# ============================================================
#  vive/ — Vive tracker + Manus teleop master
# ============================================================
(
cd vive

MANUS_DIR="../../fgx/manus/ManusSDK"
OPENVR_LIB_DIR="${OPENVR_LIB_DIR:-$HOME/.local/share/Steam/steamapps/common/SteamVR/bin/linux64}"

#vive tracker -> UDP 6D pose streamer
g++ -O2 -g -std=c++17 -W -Wall -o vive-udp vive-udp.cpp \
    -I. \
    -L"$OPENVR_LIB_DIR" -lopenvr_api -Wl,-rpath,"$OPENVR_LIB_DIR" \
    -pthread

#vive + manus teleop master (zmq PUSH to slave/logger)
g++ -O2 -g -std=c++17 -W -Wall -o vmaster vmaster.cpp \
    -I"$MANUS_DIR/include" -I. -I../../fg/dev/manus \
    -L"$MANUS_DIR/lib" -lManusSDK_Integrated -Wl,-rpath,"$MANUS_DIR/lib" \
    -L"$OPENVR_LIB_DIR" -lopenvr_api -Wl,-rpath,"$OPENVR_LIB_DIR" \
    -lzmq -lncurses -pthread
)
