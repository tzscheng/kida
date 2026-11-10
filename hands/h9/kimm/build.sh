#!/bin/bash
# hand runner (UDP <-> CAN), see runner.c header for usage
cd "$(dirname "$0")"

gcc -W -Wall -O2 -o runner runner.c -lm

# simulation build: no CAN bus, software motor model (for runner-test / protocol checks)
gcc -W -Wall -O2 -DRUNNER_SIM -o runner-sim runner.c -lm
