# AGENTS.md

This file provides guidance to Codex and Claude Code when working with code in this repository.

## What this is

`kida` is a control project for a 14-DOF dual-arm manipulator (two 7-DOF arms sharing a torso `root`). It lives at `/home/ubuntu/kida` as an independent repo, but it is not a standalone framework — it sits on top of the sibling `tact` toolkit (developed in its own repo `pytact` at `github.com/tzscheng/pytact`, checked out at `../pytact` for reference) which it consumes as a **versioned PyPI release** (`pytact>=0.1.0a1` in `pyproject.toml`, from <https://pypi.org/project/pytact/>) for kinematics/dynamics, simulation, and rendering, and on the local `h9.py` or `dg5f.py` gripper agent (vendored copies, see below). There is deliberately **no** `[tool.uv.sources]` override for the local `../pytact` clone: edits there never leak into kida — publish a new pytact release and bump the version pin (then `uv sync`) to pick it up. `h9.py`, the generic `utils/start`, and the gripper YAMLs under `yaml/` are local snapshot copies of their fg originals (`../fg/h9.py`, `../fg/start`, `../fg/h9/yml/`) — they do not auto-track fg updates; re-copy when the originals change. See the `pytact` source at `../pytact` for the framework details (that repo no longer tracks its own `AGENTS.md`).

This project provides:

- `kida.py` — `agent` class for the full dual-arm (14 joints, `n_u=14`, `n_y=42`).
- `single.py` — `agent` class for one arm + hand (`n_u=7`, `n_y=21`), used when running just the left or right side.
- `kida-run` / `single-run` — executable runners (uv shebang) that replace the generic `fg/start`. Unlike `start`, these wire up an arm agent **plus** one or two hand agents (`h9` or `dg5f`) and concatenate the `u`/`y` vectors themselves.
- `eio/eio-kida.c` / `eio/eio-single.c` → `eio/eio-kida.so` / `eio/eio-single.so` — C shared libs that drive the real arms over CAN (per-arm pthread on cores 1, 2) and the DG-5 hands over UDP (`127.0.0.1:6660` left, `:6661` right). `htype=1` forks `eio/eio-dg5f` (tracked binary); `htype=0` is H9-style hands handled inside the Python loop.
- `yaml/kida.yaml`, `yaml/kida-left.yaml`, `yaml/kida-right.yaml` — robot models for the dual arm and per-arm variants. Gripper YAMLs (`dg5f-*.yaml`, `h9-*.yaml`) are local files (h9 ones copied from `../fg/h9/yml/`).
- `usrsample.py` — reference ZMQ client. `rcvpp.py` is a minimal proprio subscriber.

## Build / run

```bash
cd eio  && ./build.sh                       # gcc → eio/eio-kida.so, eio/eio-single.so
                                            # (uses local myactcan.h; -I.)
cd rs2  && ./build.sh                        # rs2/{msender,mreceiver,videorec} (separate)
cd vive && ./build.sh                        # vive/vmaster (separate; local retarget.h)
```

Dual-arm:
```bash
./kida-run -g 0                              # tact sim, H9 hands
./kida-run -g 1                              # tact sim, DG-5 hands
./kida-run -g 1 -x [-b]                      # real hardware (eio/eio-kida.so); -b uses
                                            # actuator built-in position control instead of torque
./kida-run -g 1 -v                           # verbose: print TCP pose each step
```

Single arm (one side at a time):
```bash
./single-run -t 0 -g 0                       # left arm + H9 hand, sim
./single-run -t 1 -g 1 -x                    # right arm + DG-5 hand, real
```

Flags shared by both runners: `-x` real hardware (loads `eio/eio-*.so`), `-b` actuator built-in position controller, `-v` verbose, `-g 0|1` gripper (h9|dg5f). `kida-run` also has `-m` (MuJoCo, currently commented out). `single-run` requires `-t 0|1` for left/right.

The runners pin to CPU 0 (`sched_setaffinity({0})`); the C side pins arm threads to cores 1 and 2.

## IPC contract

These runners do **not** use `fg/start`'s logging/dispatch. They do their own ZMQ binding:

- PULL `ipc:///dev/shm/default` — commands. Top-level words `quit`/`reset` are handled in the runner; otherwise the message is split on `,` into up to three substrings (arm, hand1, hand2 for `kida-run`; arm, hand for `single-run`) and each is forwarded to that agent's `msgproc`.
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

`rate` is the controller's tick rate (Hz) used for trajectory durations. Both runners pass `rate=240` to all four controllers (kida/single/h9/dg5f) regardless of mode — this matches real-HW eio pacing exactly. In sim, frameskip is *hardcoded*: `1` if `-x` (real), `4` otherwise. So sim runs physics at 1 kHz (`env.dt=0.001`) but controllers are called every 4 steps (≈250 Hz), with last `(tau, q_ref, qd_ref)` held between (ZOH — same shape as real HW eio holds last command between Python updates). The 4 % wall-clock mismatch (sim trajectories finish 4 % faster than nominal because controllers think they're at 240 Hz but actually run at 250 Hz) is accepted; not worth branching `rate` per mode. See `kida-run` / `single-run` for the wiring. The branch in `update()` switches between two control paths per state, driven by `self.has_pd = env.has_pd` (set once at controller `__init__`):

- **internal PD** (cmode==1 on eio, or sim with native PD): emit position references via `q_ref`; for task-space, emit IK results.
- **external PID** (cmode==0 / torque-mode hardware): `pid.update(...) + m.gravity(q)` for joint targets, or `jtc.update(...) + m.gravity(q) + sk*(rq - q)` for task targets — the `sk*(rq - q)` term is a soft postural spring biasing the elbow.

The `-b` flag is the single source of truth: it sets both eio's `cmode=1` (so the firmware runs onboard PD) and `cenv.has_pd=True` (so the controller emits q_ref) in a single argv parse. Without `-b` both stay at 0/False. See `kida-run` / `single-run` for the wiring.

## Conventions specific to kida

- `tact.Model.ik(...)` is called at construction for `home`, `q_d1..q_d4` — these depend on the gripper YAML being importable (the runner imports `h9` or `dg5f` based on `-g`).
- For dual-arm IK, frames are always `{'tcp1':'6d', 'tcp2':'6d'}`. For single-arm it is `{'tcp':'6d'}`. Don't mix.
- `single.py` uses `y_sign = +1` (left) / `-1` (right) to mirror task-space targets. The yml filename suffix (`-left` / `-right`) is what sets it — name yml files accordingly.
- `eio/eio-kida.c` is hard-coded for `kt = {1.4, 1.4, 1.3, 1.3, 1.3, 1.9, 1.9}` (joints 1–5 detuned because their current I-gain is zeroed) and for joint-direction sign vectors per arm. If you change either the motors or the YAML joint axes, both `kt` and `dir` need updating.
- The C `step()` busy-waits with `usleep(3000)` to target ~240 Hz. There is no separate clock; control rate is set by that sleep.
- **`step` symbol collides with libc's System V regex API** (`char *step(const char *, const char *)` from `<regexp.h>`). ctypes `cdll.step` from Python still finds our `step()` via dlsym, but any *internal* `reset() -> step()` call in the .so goes through the PLT and was resolving to libc's `step()`, which then called `regexec(NULL, ...)` and segfaulted in `strlen(NULL)`. Worked around with `-Wl,-Bsymbolic-functions` in `eio/build.sh` — that flag makes intra-.so function calls bind to the .so's own definitions. If you add a new C backend or remove that linker flag, the bug will resurface. Long-term fix is to rename `step`/`reset`/`init`/`finish` to namespaced symbols (`eio_step` etc.) and update `tact.CEnv` to match; the same generic-name risk exists for any other backend (`../pytact/extras/mjenv.so`, `chenv.so`) that exports `step`.

## Subdirectories

- `eio/` — C bridge sources, `myactcan.h`, `build.sh`, built shared libs, and the `eio-dg5f` helper binary (tracked; a regular file, not a symlink).
- `rs2/` — RealSense multicam streamer (`msender`/`mreceiver`/`videorec`; moved here from fg `dev/rs2`, own `AGENTS.md`). Built separately by its own `rs2/build.sh`.
- `vive/` — Vive tracker + Manus teleop master (`vmaster`; moved here from fg `dev/vive`, own `AGENTS.md`). Built separately by its own `vive/build.sh`; needs ManusSDK (`../../fgx/manus`) and SteamVR. Legacy/test helpers live in `vive/tests/`.
- `yaml/` — kida-specific YAMLs plus gripper YAMLs (h9 ones are snapshot copies from `../fg/h9/yml/`).
- `utils/` — temporary/experimental tools; contents change freely, don't rely on them.
