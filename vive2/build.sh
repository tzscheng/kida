#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Wpedantic -pthread \
    miniaml.cpp -I. -L. -lultimate-tracker \
    -Wl,-rpath,'$ORIGIN' -o minimal

printf 'Built: %s/minimal\n' "$PWD"
