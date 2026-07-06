# AGENTS.md

This file provides guidance to Codex and Claude Code when working with code in this repository.

## What this is

`kida` is a control project for a 14-DOF dual-arm manipulator (two 7-DOF arms sharing a torso `root`). It is not a standalone framework — it sits on top of the sibling `tact` toolkit (`../tact`) for kinematics/dynamics, simulation, and rendering, and on `../h9` or the local `dg5.py` gripper agent. `h9.py` and the generic `start` are symlinks into sibling repos; treat symlinks as read-only here. See `../tact/AGENTS.md` for the framework details.

This project provides:

- `kida.py` — `agent` class for the full dual-arm (14 joints, `n_u=14`, `n_y=42`).
- `single.py` — `agent` class for one arm + hand (`n_u=7`, `n_y=21`), used when running just the left or right side.
- `kida.run` / `single.run` — executable runners (uv shebang) that replace the generic `tact/extras/start`. Unlike `start`, these wire up an arm agent **plus** one or two hand agents (`h9` or `dg5`) and concatenate the `u`/`y` vectors themselves.
- `eio-kida.c` / `eio-single.c` → `eio/eio-kida.so` / `eio/eio-single.so` — C shared libs that drive the real arms over CAN (per-arm pthread on cores 1, 2) and the DG-5 hands over UDP (`127.0.0.1:6660` left, `:6661` right). `htype=1` forks `eio/eio-dg5` (symlink to `../../dg5/eio-dg5`); `htype=0` is H9-style hands handled inside the Python loop.
- `yml/kida.yml`, `yml/kida-left.yml`, `yml/kida-right.yml` — robot models for the dual arm and per-arm variants. Gripper YAMLs (`dg5-*.yml`, `h9-*.yml`) are symlinks into the gripper repos.
- `usrsample.py` — reference ZMQ client. `_/sample.py` is an older UDP-based version and `rcvpp.py` is a minimal proprio subscriber.

## Build / run

```bash
./build.sh                                  # gcc → eio/eio-kida.so and eio/eio-single.so
                                            # needs myactcan.h from ../dev/myact
```

Dual-arm:
```bash
./kida.run -g 0                              # tact sim, H9 hands
./kida.run -g 1                              # tact sim, DG-5 hands
./kida.run -g 1 -x [-b]                      # real hardware (eio/eio-kida.so); -b uses
                                            # actuator built-in position control instead of torque
./kida.run -g 1 -v                           # verbose: print TCP pose each step
```

Single arm (one side at a time):
```bash
./single.run -t 0 -g 0                       # left arm + H9 hand, sim
./single.run -t 1 -g 1 -x                    # right arm + DG-5 hand, real
```

Flags shared by both runners: `-x` real hardware (loads `eio/eio-*.so`), `-b` actuator built-in position controller, `-v` verbose, `-g 0|1` gripper (h9|dg5). `kida.run` also has `-m` (MuJoCo, currently commented out). `single.run` requires `-t 0|1` for left/right.

The runners pin to CPU 0 (`sched_setaffinity({0})`); the C side pins arm threads to cores 1 and 2.

## IPC contract

These runners do **not** use `tact/extras/start`'s logging/dispatch. They do their own ZMQ binding:

- PULL `ipc:///dev/shm/default` — commands. Top-level words `quit`/`reset` are handled in the runner; otherwise the message is split on `,` into up to three substrings (arm, hand1, hand2 for `kida.run`; arm, hand for `single.run`) and each is forwarded to that agent's `msgproc`.
- PUB CONFLATE `ipc:///dev/shm/proprio` — `y.astype(float32).tobytes()`. Cycle: every 2 sim ticks (~120 Hz) on real hardware, every 33 ticks (~30 Hz) in sim.
- PUB CONFLATE `ipc:///dev/shm/<cam_name>` — JPEG frames per camera from `env.get_camera_name()` (typically `headcam`, `leftcam`, `rightcam`). Sim only; on real hardware camera frames come from a separate `rs2/msender` process.

`usrsample.py` shows the full client pattern (PUSH commands + SUB proprio + SUB headcam, with `cv2.imshow` for the JPEG stream) and documents the proprio layout: `14 + 14 + 14` arm joints (pos / vel / current) followed by `3×20` per hand (pos / vel / current), totalling 162 float32 values for dual-arm + two hands.

## Agent state machine (kida.py / single.py)

Both `agent` classes share the same `msgproc(w) → shift(state)` + `update(y) → u` shape. Recognised commands:

- `joint <q...>` — move to joint target (clipped to per-joint limits).
- `task <x...>` — move to task-space target (`6d` per TCP, clipped to a workspace box).
- `init` / `rest` — staged ramps via `init1 → init2 → home` and the reverse.
- `home` / `home-task` — go to a precomputed home pose (joint or task variant).
- `joint-loop` / `task-loop` — cycle through 4 waypoints continuously (8·`rate` per cycle).
- `gcomp` — pure gravity compensation (torque mode only).
- `free` — accepted only when controller is in torque mode (`not self.has_pd`).
- `scan` — pulls a camera frame (headcam) and discards it; placeholder.

`rate` is the controller's tick rate (Hz) used for trajectory durations. Both runners pass `rate=240` to all four controllers (kida/single/h9/dg5) regardless of mode — this matches real-HW eio pacing exactly. In sim, frameskip is *hardcoded*: `1` if `-x` (real), `4` otherwise. So sim runs physics at 1 kHz (`env.dt=0.001`) but controllers are called every 4 steps (≈250 Hz), with last `(tau, q_ref, qd_ref)` held between (ZOH — same shape as real HW eio holds last command between Python updates). The 4 % wall-clock mismatch (sim trajectories finish 4 % faster than nominal because controllers think they're at 240 Hz but actually run at 250 Hz) is accepted; not worth branching `rate` per mode. See `kida.run` / `single.run` for the wiring. The branch in `update()` switches between two control paths per state, driven by `self.has_pd = env.has_pd` (set once at controller `__init__`):

- **internal PD** (cmode==1 on eio, or sim with native PD): emit position references via `q_ref`; for task-space, emit IK results.
- **external PID** (cmode==0 / torque-mode hardware): `pid.update(...) + m.gravity(q)` for joint targets, or `jtc.update(...) + m.gravity(q) + sk*(rq - q)` for task targets — the `sk*(rq - q)` term is a soft postural spring biasing the elbow.

The `-b` flag is the single source of truth: it sets both eio's `cmode=1` (so the firmware runs onboard PD) and `cenv.has_pd=True` (so the controller emits q_ref) in a single argv parse. Without `-b` both stay at 0/False. See `kida.run` / `single.run` for the wiring.

## Conventions specific to kida

- `tact.Model.ik(...)` is called at construction for `home`, `q_d1..q_d4` — these depend on the gripper YAML being importable (the runner imports `h9` or `dg5` based on `-g`).
- For dual-arm IK, frames are always `{'tcp1':'6d', 'tcp2':'6d'}`. For single-arm it is `{'tcp':'6d'}`. Don't mix.
- `single.py` uses `y_sign = +1` (left) / `-1` (right) to mirror task-space targets. The yml filename suffix (`-left` / `-right`) is what sets it — name yml files accordingly.
- `eio-kida.c` is hard-coded for `kt = {1.4, 1.4, 1.3, 1.3, 1.3, 1.9, 1.9}` (joints 1–5 detuned because their current I-gain is zeroed) and for joint-direction sign vectors per arm. If you change either the motors or the YAML joint axes, both `kt` and `dir` need updating.
- The C `step()` busy-waits with `usleep(3000)` to target ~240 Hz. There is no separate clock; control rate is set by that sleep.
- **`step` symbol collides with libc's System V regex API** (`char *step(const char *, const char *)` from `<regexp.h>`). ctypes `cdll.step` from Python still finds our `step()` via dlsym, but any *internal* `reset() -> step()` call in the .so goes through the PLT and was resolving to libc's `step()`, which then called `regexec(NULL, ...)` and segfaulted in `strlen(NULL)`. Worked around with `-Wl,-Bsymbolic-functions` in `build.sh` — that flag makes intra-.so function calls bind to the .so's own definitions. If you add a new C backend or remove that linker flag, the bug will resurface. Long-term fix is to rename `step`/`reset`/`init`/`finish` to namespaced symbols (`eio_step` etc.) and update `tact.CEnv` to match; the same generic-name risk exists for any other backend (`tact/extras/mjenv.so`, `chenv.so`) that exports `step`.

## Subdirectories

- `_/` — scratch/archive (older UDP-based `sample.py`, `act/` mini-ACT transformer experiment, `rcvpp.py`). Don't edit unless asked.
- `eio/` — built C shared libs and the `eio-dg5` symlink to `../../dg5/eio-dg5`.
- `yml/` — kida-specific YAMLs plus symlinks to gripper YAMLs in sibling repos. `yml/_` is empty scratch.
