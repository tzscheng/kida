#!/bin/bash
# hand runner (UDP <-> CAN), see runner.c header for usage
cd "$(dirname "$0")"

gcc -W -Wall -O2 -o runner runner.c -lm

# simulation build: no CAN bus, software motor model (for runner-test / protocol checks)
#gcc -W -Wall -O2 -DRUNNER_SIM -o runner-sim runner.c -lm

# motor/encoder check tool with a curses screen (talks to the hand over CAN directly)
gcc -W -Wall -O2 -o monitor monitor.c -lncurses -lm

# simulation build: no CAN bus, for trying the screen
#gcc -W -Wall -O2 -DMONITOR_SIM -o monitor-sim monitor.c -lncurses -lm

# Manus glove -> runner teleoperation. The Manus SDK 3.1.1 (headers and
# libManusSDK_Integrated.so) is kept in ./manus-sdk; the binary looks for the
# library next to itself ($ORIGIN), so it runs from any directory.
g++ -O2 -g -std=c++17 -W -Wall -o manus manus.cpp \
    -Imanus-sdk/include \
    -Lmanus-sdk/lib -lManusSDK_Integrated -Wl,-rpath,'$ORIGIN/manus-sdk/lib' \
    -pthread

# simulation build: no glove, no Manus SDK, synthetic open/close motion (try with runner-sim)
#g++ -O2 -std=c++17 -W -Wall -DMANUS_SIM -o manus-sim manus.cpp -pthread
