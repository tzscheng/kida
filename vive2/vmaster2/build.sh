#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

if [[ $# -gt 1 || ( $# -eq 1 && "$1" != "--test" ) ]]; then
    printf 'Usage: ./build.sh [--test]\n' >&2
    exit 2
fi

manus_dir="${MANUS_SDK_DIR:-$PWD/../../../ManusSDK/3.1.1}"
manus_dir="$(realpath -- "$manus_dir")"
if [[ ! -f "$manus_dir/include/ManusSDK.h" || ! -f "$manus_dir/lib/libManusSDK_Integrated.so" ]]; then
    printf 'Manus SDK missing. Set MANUS_SDK_DIR to the SDK root (include/ and lib/).\n' >&2
    exit 1
fi
pkg-config --exists libzmq ncurses
read -r -a system_flags <<< "$(pkg-config --cflags --libs libzmq ncurses)"
common_flags=(-std=c++17 -O2 -g -Wall -Wextra -Wpedantic -pthread -I. -I"$manus_dir/include")
link_flags=(-L. -lultimate-tracker -L"$manus_dir/lib" -lManusSDK_Integrated
    -Wl,-rpath,'$ORIGIN' -Wl,-rpath,"$manus_dir/lib" "${system_flags[@]}")

"${CXX:-g++}" "${common_flags[@]}" vmaster2.cpp "${link_flags[@]}" -o vmaster2
printf 'Built: %s/vmaster2\n' "$PWD"

if [[ "${1:-}" == "--test" ]]; then
    "${CXX:-g++}" "${common_flags[@]}" test_vmaster2.cpp "${link_flags[@]}" -o test_vmaster2
    ./test_vmaster2
fi
