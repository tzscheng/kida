#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

if [[ $# -gt 1 || ( $# -eq 1 && "$1" != "--test" ) ]]; then
    printf 'Usage: ./build.sh [--test]\n' >&2
    exit 2
fi

if ! pkg-config --exists glfw3 gl eigen3; then
    printf 'gui-test requires GLFW, OpenGL and Eigen headers (libglfw3-dev libgl1-mesa-dev libeigen3-dev).\n' >&2
    exit 1
fi
read -r -a graphics_flags <<< "$(pkg-config --cflags --libs glfw3 gl)"
read -r -a eigen_flags <<< "$(pkg-config --cflags eigen3)"
python3 generate_kida_model.py
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Wpedantic -pthread \
    gui-test.cpp -I. -L. -lultimate-tracker "${graphics_flags[@]}" "${eigen_flags[@]}" \
    -Wl,-rpath,'$ORIGIN' -o gui-test
printf 'Built: %s/gui-test\n' "$PWD"

if [[ "${1:-}" == "--test" ]]; then
    "${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -Wpedantic \
        test_arm_ik.cpp "${eigen_flags[@]}" -o test_arm_ik
    ./test_arm_ik
fi
