# Vive Teleop Notes

This file holds detailed implementation notes for `vmaster` and related tools.
`AGENTS.md` keeps build/run and wire-contract basics.

## vmaster Data Flow

Three inputs feed a single roughly 30 Hz teleop loop, which sends two ZMQ PUSH
streams:

```text
Vive trackers --(OpenVR background app)--+
Manus gloves  --(SDK callbacks, bg thr)--+--> main loop --> ipc:///dev/shm/default
keyboard      --(ncurses getch)----------+              --> ipc:///dev/shm/logger
```

Both PUSH sockets set `ZMQ_IMMEDIATE=1`, `ZMQ_CONFLATE=1`, `ZMQ_LINGER=0` and
`connect()` to processes that `bind`. Sends use `ZMQ_DONTWAIT`; `EAGAIN` is
dropped like the Python original.

Commands are ASCII and comma-separated for multi-part commands:

- arm side: `task <6 or 12 or 7 floats>` or `none`
- hand side for `-g` with `-t` in `{0,1,2}`: `joint <N floats>` or `none`
- combined: `<arm>, <hand>` if both sides exist

Types:

- `-t0`/`-t1`: 6-float task for one arm
- `-t2`: 12-float task, left then right
- `-t5`: 7-float gos task, pose plus `jawpos`

Attach flags gate sending. `ee_attach` captures current `_xyz` as an offset to
avoid jumps. `hand_attach` is plain on/off. `r` and `i` reset both attach flags.

## Coordinate Transform

Per frame, per tracker:

```text
T01 = trans(1.2,0,0) * rot_z(pi/2) * rot_x(pi/2)
T12 = OpenVR pose (tracker -> HMD-base)
T32:
  left:  rot_x( pi/2) * rot_z(pi/2)
  right: rot_x(-pi/2) * rot_z(pi/2)
  gos:   rot_x( pi)   * rot_z(pi/2)
T03 = T01 * T12 * inv(T32)
xyz = scale * (T03[:3,3] + offset[slot]) + bias[slot]
rpy = rotation_to_euler_xyz(T03[:3,:3])
```

Bias/scale are per mode in the loop. Euler conversion matches
`tact.rotation_to_euler` from the Python original: extrinsic XYZ with explicit
gimbal-lock branch.

## Manus Retargeting

`OnManusErgonomics` writes a 20-float-per-side snapshot under `g_manus_mutex`.
The main loop snapshots it and runs `RetargetSide` to fill `q[40]`:

- left hand: `q[0..19]`
- right hand: `q[20..39]`

Retarget formulas are gripper-specific affine maps in `vmaster.cpp`. The
relevant slice length is 9 for H9 and 20 for DG5F.

Calibration files live in the local `calib/` copy (originals in `../../fg/dev/manus/calib/`). The Manus thread waits until both
glove IDs are known, then uploads calibration once. `-n` skips this.

`vmaster.py` uses `../../fgx/manus/pymanus` (built by fg's `dev/manus`), which
mirrors the Manus section of `vmaster.cpp` and reuses the local `retarget.h`
(vendored from fg `dev/manus`).

## ncurses UI

`InitUi` redirects stdout/stderr into a pipe and drains it into the bottom log
pane. Prints before `InitUi` go to the real terminal intentionally. Before UI
init, `vmaster` validates `./logger` when `-l` is set and exits if it is missing
or dies immediately.

## Archive

`_/` contains the Python original. It consumed glove data over UDP from a
separate Manus daemon; the C++ port pulls glove data directly through the Manus
SDK. Treat it as historical reference.
