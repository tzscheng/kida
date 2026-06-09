#!/bin/bash

# -Wl,-Bsymbolic-functions: resolve intra-.so function calls against our own
# definitions first. Without it, reset() -> step() goes through the PLT and
# matches libc's step() (System V regex API) instead of ours, causing a
# strlen(NULL) segfault. The name 'step' is too generic — keep this flag.

#io
gcc eio-kida.c -W -Wall -shared -o eio/eio-kida.so -fPIC -I../dev/can -Wl,-Bsymbolic-functions

gcc eio-single.c -W -Wall -shared -o eio/eio-single.so -fPIC  -I../dev/can -Wl,-Bsymbolic-functions

#dg5
#gcc -W -Wall -o eio/eio-dg5 eio-dg5.c -I/usr/local/include/DGSDK -lDGSDK
