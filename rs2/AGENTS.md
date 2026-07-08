# AGENTS.md

This file provides guidance to Codex and Claude Code when working with code in this repository.

## Build

There is no Make/CMake. The `rs2` section of the repo-root `../build.sh` holds the canonical g++ invocations:

```bash
(cd .. && ./build.sh)   # rs2 section of the merged kida build.sh
```

Active outputs are `mreceiver`, `msender`, and `videorec`. Required system libs:
`librealsense2`, `opencv4`, `libzmq3-dev`, `libzstd-dev`, `libturbojpeg-dev`.
There are no tests, linter, or formatter config in this tree.

## Run

- `./msender <name[:type]> [name[:type] ...] [-d] [-q jpeg_quality] [-z zstd_level]`
  streams one RealSense pipeline per token. `:type` is `r`/`rgb` (default) or
  `d`/`depth`. The full token is used as the ZMQ endpoint name.
- `./mreceiver <name[:type]> [name[:type] ...]` views RGB, depth, or tact lidar
  streams. Decode hints: `:r`/`:rgb`, `:d`/`:depth`, `:l`/`:lidar`.
- `./videorec [-d] [-s segment_seconds]` records one RGB camera to segmented
  `record_<timestamp>.mp4`.

Legacy global `-D`/`-d` depth flags are rejected; use per-stream `:d`.

## Sharp Edges

- Endpoint naming is asymmetric: `msender cam1:d` binds
  `ipc:///dev/shm/cam1:d`, while `mreceiver cam1:d` connects to
  `ipc:///dev/shm/cam1`. RGB with no suffix still pairs normally.
- `msender` maps logical names to physical cameras by sorted serials and sorted
  full tokens, not command-line order. Physically swapping cameras can change
  assignment.
- `cam1:r` and `cam1:d` are different tokens and require two cameras.
- Lidar is receiver-only here; the publisher is the shared `start` launcher
  (fg repo, backed by tact). The receiver
  expects fixed 320x240 raw float32 lidar frames.
- Sender shutdown and cold-start `execv` recovery are load-bearing; see
  `PROTOCOL.md` before changing thread or ZMQ lifecycle code.

## Architecture Details

Detailed transport, cold-start recovery, serial mapping, and encoding notes live
in `PROTOCOL.md`.

## Conventions

- Comments in the source are mixed Korean/English; preserve surrounding style.
- `msender` uses `getopt`; `mreceiver` uses a hand-rolled parser.
- `IMG_RGB=1`, `IMG_DEPTH=2` are kept in sync between sender and receiver.
  `IMG_LIDAR=3` is receiver-only.
- `rs2::log_to_console(RS2_LOG_SEVERITY_WARN)` is enabled in the sender. USB 2
  fallback warnings are the first thing to check for missing-frame symptoms.
