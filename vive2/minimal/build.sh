#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

if [[ $# -ne 0 ]]; then
    printf 'Usage: ./build.sh\n' >&2
    exit 2
fi

"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Wpedantic -pthread \
    miniaml.cpp -I. -L. -lultimate-tracker \
    -Wl,-rpath,'$ORIGIN' -o minimal

printf 'Built: %s/minimal\n' "$PWD"
