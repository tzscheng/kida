#!/bin/bash

# ============================================================
#  rs2/ — RealSense multicam streamer
# ============================================================
# Split out of kida/build.sh. Runnable standalone (cd rs2 && ./build.sh) or
# invoked by the top-level kida/build.sh. Resolves its own dir so CWD-agnostic.

cd "$(dirname "$0")"

#multicam receiver
g++ -std=c++17 -O2 mreceiver.cpp -o mreceiver `pkg-config --cflags --libs opencv4` -lzstd -lzmq -lpthread

#multicam sender
g++ -O2 -std=c++17 -pthread msender.cpp -o msender $(pkg-config --cflags --libs opencv4) -lrealsense2 -lzmq -lzstd -lturbojpeg

#video recorder (single cam)
g++ videorec.cpp -o videorec $(pkg-config --cflags --libs realsense2 opencv4)
