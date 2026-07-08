# RealSense ZMQ Protocol Notes

This file holds the details behind `msender` and `mreceiver`. `AGENTS.md` keeps
only the build/run and sharp-edge summary.

## Transport

`msender` and `mreceiver` communicate through Unix-domain ZMQ sockets at
`ipc:///dev/shm/<token>`.

On `msender`, the token is the full positional arg verbatim: `cam1:d` binds
`ipc:///dev/shm/cam1:d`. RGB vs depth is picked from the same `:type` suffix,
and the suffix is not stripped from the endpoint path.

On `mreceiver`, `name:type` connects to `ipc:///dev/shm/<name>` and the suffix
only selects the decoder. Consequence: depth streams from new-style `msender`
do not pair with `./mreceiver foo:d` unless the sender also binds `foo` or the
receiver is changed to use the full token as endpoint. RGB without a hint
(`./msender foo` and `./mreceiver foo`) remains compatible.

Both sides set `ZMQ_CONFLATE=1`. The sender also sets `ZMQ_LINGER=0`.

Shutdown ordering is load-bearing: SIGINT flips `g_run=false`, then
`zmq_ctx_shutdown(zctx)` wakes workers blocked in `zmq_send`, then worker
threads are joined, sockets close, and finally `zmq_ctx_term` runs.

## Cold-Start Recovery

`msender` self-recovers if a camera worker fails to receive its first frame
within `FIRST_FRAME_DEADLINE_MS` (3000 ms). It sets `g_request_exec=true`,
signals shutdown, then the main thread calls `execv("/proc/self/exe", g_argv)`
after normal cleanup.

This exists because in-process `hardware_reset` plus `pipe.start` retry is not
reliable for D400-series cold-start failures where libusb state is stale. Before
exec, the worker attempts `hardware_reset()` and sleeps 4 seconds for USB
reenumeration. `g_argv` must preserve original argv for this path.

The runtime loop emits a 5 second heartbeat and a one-shot stall warning after
first-frame success; these are diagnostics, not failure paths.

## Logical Name To Serial Mapping

`msender` does not pin logical names to serials. It enumerates devices, sorts
serials ascending, sorts unique full positional tokens alphabetically, and zips
the two lists.

Examples:

- `./msender top side`: lower serial becomes `side`, higher becomes `top`.
- `./msender cam1:d cam0:r`: lower serial becomes `cam0:r`.

Input order affects display tile layout when `-d` is enabled, but not serial
assignment. Stable token-to-camera binding would need new code.

Tokens are deduped by full-string match. `cam1:r` and `cam1:d` are distinct and
would require two physical cameras.

## Encoding

- RGB: 640x480 BGR8 to TurboJPEG (`TJSAMP_420`, `TJFLAG_FASTDCT`), default
  quality 80. Receiver decodes via `cv::imdecode`.
- Depth from `msender`: 640x480 Z16 to zstd level 1 by default.
- Depth/lidar in `mreceiver`: zstd-decompressed into a per-stream buffer.
  `W*H*4` means float32 meters; `W*H*2` means RealSense Z16 mm converted to
  meters.
- Lidar uses the same display path as depth but masks negative no-return pixels
  to black.

`WIDTH`/`HEIGHT`/`FPS` are duplicated in `msender.cpp`, `mreceiver.cpp`, and
`videorec.cpp`; keep them in sync. `LIDAR_WIDTH`/`LIDAR_HEIGHT` are
`mreceiver.cpp` only and must match tact YAML lidar `res`.

## videorec

`videorec.cpp` uses `rs2::pipeline` directly and writes segmented mp4 files. It
does not touch ZMQ. Do not share code with `msender.cpp` unless that coupling is
intentional.
