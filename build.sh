#!/bin/bash

# ============================================================
#  eio — CAN/UDP hardware bridges
# ============================================================

# -Wl,-Bsymbolic-functions: resolve intra-.so function calls against our own
# definitions first. Without it, reset() -> step() goes through the PLT and
# matches libc's step() (System V regex API) instead of ours, causing a
# strlen(NULL) segfault. The name 'step' is too generic — keep this flag.

#io
gcc eio-kida.c -W -Wall -shared -o eio/eio-kida.so -fPIC -I. -Wl,-Bsymbolic-functions

gcc eio-single.c -W -Wall -shared -o eio/eio-single.so -fPIC  -I. -Wl,-Bsymbolic-functions

#dg5f — needs the DGSDK installed under /usr/local; skipped (keeping the tracked
#      eio/eio-dg5f binary) on machines without it
if [ -d /usr/local/include/DGSDK ]; then
    gcc -W -Wall -o eio/eio-dg5f eio-dg5f.c -I/usr/local/include/DGSDK -lDGSDK
else
    echo "dg5f: DGSDK not found — skipping eio-dg5f build"
fi

# rs2/ (RealSense multicam) and vive/ (Vive + Manus teleop) are built
# separately by their own scripts — this top-level build handles eio only:
#   cd rs2  && ./build.sh
#   cd vive && ./build.sh
