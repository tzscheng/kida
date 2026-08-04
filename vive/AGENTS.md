# AGENTS.md

This file provides guidance to Codex and Claude Code when working with code in this repository.

## Build

No Make/CMake. This dir's own `build.sh` is standalone. It builds:

- `vmaster` from `vmaster.cpp`

```bash
./build.sh              # build vive tools here directly (runnable from anywhere)
```

OpenVR is resolved through `OPENVR_LIB_DIR` (default SteamVR path). Manus SDK is
resolved from `../../ManusSDK/3.0.1` (i.e. `~/ManusSDK/3.0.1`, outside the
repo). Both are baked into rpaths; rebuild after moving dependencies. There are no tests, linter, or formatter config.

## Run

SteamVR must already be running.

- `~/ongoing/dev/vive/vive-udp [-tN]`: minimal tracker pose UDP streamer to `127.0.0.1:6634`.
  Format: `T<idx> px py pz qx qy qz qw`; default poll period is 10 ms.
- `./vmaster -tN [-gN] [-l] [-n]`: C++ teleop master. `-t`: 0=kida-left,
  1=kida-right, 2=kida-both, 5=gos10. `-g`: 0=H9, 1=DG5F-M, 2=DG5F-S. `-l` spawns
  `./logger`; `-n` skips Manus glove calibration.
- `./logger -tN [-gN] [-a cpu-list]` records HDF5 episodes and pins itself to
  CPUs `6-8` by default. Override with `-a 6,7,8` or another CPU list.

(The Python sibling `tests/vmaster.py` and its `pymanus` module were retired to
`~/Desktop/manus-backup/`.)

Hot keys in `vmaster`: `/` randomize, `r` rest, `a` toggle attached, `h` home,
`i` init+home, `c` toggle log, `z`/`x` jaw, `q`/ESC quit.

`player` (Python script, uv shebang) is offline HDF5 inspection/playback; it is
unrelated to live operation.

## Wire Contract

`vmaster` sends commands over ZMQ PUSH:

- slave: `ipc:///dev/shm/default`
- logger: `ipc:///dev/shm/logger`

Commands are ASCII. Arm and hand payloads are comma-separated for multi-part
targets; detached sides are sent as `none` so receivers keep a stable frame
shape. See `NOTES.md` for the detailed data flow and coordinate transform.

## Manus / Retargeting

The `q[40]` hand layout is part of the wire contract:

- left hand: `q[0..19]`
- right hand: `q[20..39]`

Retarget coefficients live in `vmaster.cpp`/`retarget.h`; do not repack this
layout without coordinating slave/logger consumers. Calibration files are read
from the local `calib/` copy (originals in `../../fg/dev/manus/calib/`); `-n` skips calibration.

## Conventions

- Comments are mixed English/Korean; preserve surrounding style.
- The `tflag` (`-t0`, `-t1`, etc.) is forwarded verbatim to `./logger`;
  logger and master must agree on meanings.
- Detailed transform, Manus callback, UI, and archive notes live in `NOTES.md`.
