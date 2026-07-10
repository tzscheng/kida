# AGENTS.md

This file provides guidance to Codex and Claude Code when working with code in this repository.

## Build

There is no Make/CMake. The canonical g++ invocations live in this dir's own
`build.sh`. It is standalone; build rs2 tools here directly:

```bash
./build.sh              # build rs2 tools (runnable from anywhere)
```

Active outputs are `mreceiver`, `msender`, and `videorec`. Required system libs:
`librealsense2`, `opencv4`, `libzmq3-dev`, `libzstd-dev`, `libturbojpeg-dev`.
There are no tests, linter, or formatter config in this tree.

## Run

- `./msender <name[:type]> [name[:type] ...] [-d] [-a cpu-list] [-q jpeg_quality] [-z zstd_level]`
  streams one RealSense pipeline per token. `:type` is `r`/`rgb` (default) or
  `d`/`depth`. The suffix is a mode hint only; `name` is used as the ZMQ
  endpoint. By default, the sender pins itself to CPUs `3-5` before camera
  worker threads are launched; override with `-a 3,4,5` or another CPU list.
- `./mreceiver <name[:type]> [name[:type] ...]` views RGB, depth, or tact lidar
  streams. Decode hints: `:r`/`:rgb`, `:d`/`:depth`, `:l`/`:lidar`.
- `./videorec [-d] [-s segment_seconds]` records one RGB camera to segmented
  `record_<timestamp>.mp4`.

Legacy global `-D`/`-d` depth flags are rejected; use per-stream `:d`.

## Sharp Edges

- `msender` and `mreceiver` both parse `name[:type]` as endpoint `name` plus a
  per-stream mode hint. `msender cam1:d` binds `ipc:///dev/shm/cam1`, and
  `mreceiver cam1:d` connects to the same endpoint using depth decode.
- `msender` maps logical names to physical cameras by sorted serials and sorted
  endpoint names, not command-line order. Physically swapping cameras can change
  assignment.
- `cam1:r` and `cam1:d` conflict because they target the same endpoint with
  different stream types.
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
