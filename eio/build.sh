#!/bin/bash

# ============================================================
#  eio — CAN/UDP hardware bridges
# ============================================================

cd "$(dirname "$0")"

# -Wl,-Bsymbolic-functions: resolve intra-.so function calls against our own
# definitions first. Without it, reset() -> step() goes through the PLT and
# matches libc's step() (System V regex API) instead of ours, causing a
# strlen(NULL) segfault. The name 'step' is too generic — keep this flag.

#io
gcc eio-kida.c -W -Wall -shared -o eio-kida.so -fPIC -I. -Wl,-Bsymbolic-functions

gcc eio-single.c -W -Wall -shared -o eio-single.so -fPIC  -I. -Wl,-Bsymbolic-functions

#dg5f / dg5s — need the DGSDK installed under /usr/local; skipped (keeping the
#      tracked binaries) on machines without it. Two separate binaries because
#      the -M and -S are distinct SDK models (DG_MODEL_DG_5F_* vs _5F_S_*).
#
#      g++, not gcc: DGSDK >= 2.0.0 declares `noexcept` on its exported
#      prototypes (DGSDK.h:37) inside the `extern "C"` block, so the header only
#      parses as C++. The sources are plain C and compile clean either way; only
#      the 1.7.2-era header was C-compatible. Reverting to gcc will fail with
#      "expected declaration specifiers before 'noexcept'".
if [ -d /usr/local/include/DGSDK ]; then
    g++ -W -Wall -o eio-dg5f eio-dg5f.c -I/usr/local/include/DGSDK -lDGSDK
    g++ -W -Wall -o eio-dg5s eio-dg5s.c -I/usr/local/include/DGSDK -lDGSDK
else
    echo "dg5f/dg5s: DGSDK not found — skipping eio-dg5f / eio-dg5s build"
fi

# rs2/ (RealSense multicam) and vive/ (Vive + Manus teleop) are built
# separately by their own scripts:
#   cd ../rs2  && ./build.sh
#   cd ../vive && ./build.sh
