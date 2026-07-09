# KIDA

KIDA는 14-DOF dual-arm manipulator와 양손 gripper를 시뮬레이션 또는 실제 하드웨어에서 구동하는 프로젝트입니다. `kida-run`/`single-run` 실행기가 ZeroMQ command를 받고, proprioception과 camera frame을 publish합니다.

## 설치 및 빌드

clone한 repo 루트에서 실행합니다.

```bash
uv sync          # .venv 생성 (numpy, zmq, opencv, pytact(PyPI) 등)
cd eio && ./build.sh   # eio/ hardware bridge (rs2/·vive/는 각 폴더에서 별도 빌드)
```

**eio**·**rs2**·**vive**는 각 폴더의 별도 `build.sh`로 **따로** 빌드합니다:

- **eio** — `eio/eio-kida.c`/`eio/eio-single.c` → `eio/eio-*.so` (실제 하드웨어 CAN/UDP bridge). `eio-dg5f`는 DGSDK가 설치된 머신에서만 재빌드됩니다. → `cd eio && ./build.sh`
- **rs2** — RealSense multicam 도구 (`rs2/msender`, `rs2/mreceiver`, `rs2/videorec`). → `cd rs2 && ./build.sh`
- **vive** — Vive tracker + Manus teleop (`vive/vmaster`; ManusSDK·SteamVR 필요). → `cd vive && ./build.sh`

## 실행

시뮬레이터 실행:

```bash
./kida-run
```

다른 터미널에서 예제 client 실행:

```bash
uv run python usrsample.py
```

명령을 직접 보낼 때 (`utils/zmqmsg`는 kida 로컬 복사본):

```bash
./utils/zmqmsg init, home, home
./utils/zmqmsg rest
./utils/zmqmsg quit
```

## 실행 옵션

Dual-arm:

```bash
./kida-run                 # DG5F hand 시뮬레이터
./kida-run -g 0            # H9 hand 시뮬레이터
./kida-run -g 1 -l         # DG5F hand headless 시뮬레이터
./kida-run -g 1 -x         # 실제 KIDA + DG5F hand
./kida-run -g 1 -x -b      # 실제 KIDA, actuator built-in position controller 사용
./kida-run -g 1 -v         # TCP pose verbose 출력
```

Single-arm:

```bash
./single-run -t 0 -g 1     # left arm + DG5F hand 시뮬레이터
./single-run -t 1 -g 1     # right arm + DG5F hand 시뮬레이터
./single-run -t 0 -g 0 -x  # left arm + H9 hand 실제 하드웨어
./single-run -t 1 -g 1 -x  # right arm + DG5F hand 실제 하드웨어
```

공통 옵션:

| 옵션 | 의미 |
|---|---|
| `-g 0` | H9 hand |
| `-g 1` | DG5F hand |
| `-x` | 실제 하드웨어 backend 사용 |
| `-b` | 실제 하드웨어에서 actuator built-in PD/position mode 사용 |
| `-l` | 시뮬레이터 rendering window 없이 실행 |
| `-v` | controller debug/pose 출력 |
| `-d <file>` | `<step_count> <command>` 형식 dispatch file 실행 |
| `-t 0` | `single-run`에서 left arm 선택 |
| `-t 1` | `single-run`에서 right arm 선택 |

## 실제 하드웨어 준비

CAN interface를 먼저 올립니다 (helper는 kida 로컬 `can-up`). 장치명은 환경에 맞게 선택합니다.

```bash
./can-up 0
./can-up 1
```

DG5F hand를 사용할 때는 `kida-run -x` 또는 `single-run -x`가 `eio/eio-dg5f`를 사용합니다. RealSense camera stream은 별도 프로세스로 실행합니다.

```bash
rs2/msender
```

Vive tracker나 Manus glove를 함께 쓸 때는 `vive/steamvr-run`, `vive/vmaster`, `vive/logger`, `vive/player`, `vive/calib/`을 사용합니다. 필요한 경우 해당 센서 프로세스를 별도 터미널에서 실행합니다.

종료 후 CAN interface를 내릴 때는 `sudo ip link set can<N> down`을 사용합니다.

## ZeroMQ 통신

`kida-run`과 `single-run`은 같은 endpoint를 사용합니다. 같은 PC에서는 IPC, 다른 PC에서는 TCP로 접속합니다.

| 채널 | 로컬 endpoint | TCP endpoint | 방향 | 내용 |
|---|---|---|---|---|
| command | `ipc:///dev/shm/default` | `tcp://<robot-ip>:5555` | client -> runner | `init`, `joint`, `task`, `rest`, `quit` 등 |
| proprio | `ipc:///dev/shm/proprio` | `tcp://<robot-ip>:5556` | runner -> client | float32 proprioception |
| headcam | `ipc:///dev/shm/headcam` | `tcp://<robot-ip>:5557` | runner/camera -> client | JPEG image |
| leftcam | `ipc:///dev/shm/leftcam` | `tcp://<robot-ip>:5558` | runner/camera -> client | JPEG image |
| rightcam | `ipc:///dev/shm/rightcam` | `tcp://<robot-ip>:5559` | runner/camera -> client | JPEG image |

`usrsample.py`에서 `SERVER = None`이면 로컬 IPC를 사용합니다. 원격 PC에서 접속할 때는 `SERVER = '<robot-ip>'`로 바꿉니다.

`single-run` 실행 시 wrist camera는 `ipc:///dev/shm/wristcam`으로 publish되고, YAML port 설정상 TCP `5558`을 사용합니다.

## Command 형식

Dual-arm command는 comma로 arm, left hand, right hand 명령을 나눕니다.

```text
init, home, home
joint <arm 14 values>, <left hand 20 values>, <right hand 20 values>
task <left tcp 6 values> <right tcp 6 values>, home, home
rest
quit
```

Single-arm command는 arm, hand 두 부분입니다.

```text
init, home
joint <arm 7 values>, <hand 20 values>
task <tcp 6 values>, home
rest
quit
```

대표 command:

| command | 의미 |
|---|---|
| `init` | 초기 자세로 들어 올린 뒤 home pose로 이동 |
| `home` | home pose로 이동 |
| `joint ...` | joint-space target |
| `task ...` | task-space TCP target |
| `rest` | 팔을 내려 정지 자세로 이동 |
| `gcomp` | gravity compensation |
| `free` | torque mode에서 자유 상태 |
| `reset` | runner counter reset |
| `quit` | runner 종료 |

## Proprioception layout

Dual-arm `kida-run`의 proprioception은 총 162개 `float32`입니다.

| 구간 | 개수 | 내용 |
|---|---:|---|
| arm position | 14 | dual-arm joint position |
| arm velocity | 14 | dual-arm joint velocity |
| arm current | 14 | dual-arm joint current |
| left hand position | 20 | left hand joint position |
| left hand velocity | 20 | left hand joint velocity |
| left hand current | 20 | left hand joint current |
| right hand position | 20 | right hand joint position |
| right hand velocity | 20 | right hand joint velocity |
| right hand current | 20 | right hand joint current |

실제 하드웨어에서는 약 120 Hz, 시뮬레이터에서는 약 30 Hz로 publish합니다.

## 디렉터리 구조

| 경로 | 설명 |
|---|---|
| `kida-run`, `kida.py` | Dual-arm 실행기와 controller |
| `single-run`, `single.py` | 좌/우 한쪽 arm + hand 실행기와 controller |
| `dg5f.py` | DG5F hand gripper agent |
| `h9.py` | H9 hand agent (`../fg/h9.py` 복사본) |
| `usrsample.py` | ZeroMQ client 예제. command 송신, proprioception/camera 수신 |
| `utils/start` | generic runner 복사본 |
| `utils/zmqmsg`, `utils/udpmsg` | command 송신 helper |
| `utils/rcvpp.py` | proprioception subscriber |
| `utils/tacplot.py`, `utils/tacview.py` | DG5F tactile viewer |
| `yaml/kida.yaml` | Dual-arm robot model |
| `yaml/kida-left.yaml`, `yaml/kida-right.yaml` | 좌/우 single-arm model |
| `yaml/h9-left.yaml`, `yaml/h9-right.yaml` | H9 hand model (`../fg/h9/yml/` 복사본) |
| `yaml/dg5f-left.yaml`, `yaml/dg5f-right.yaml` | DG5F hand model |
| `yaml/desk1.yaml` | 시뮬레이션 desk scene |
| `eio/eio-kida.c`, `eio/eio-single.c`, `eio/eio-dg5f.c` | hardware bridge 소스 |
| `eio/myactcan.h` | MyActuator CAN helper header |
| `eio/build.sh` | eio 빌드 스크립트 |
| `eio/eio-kida.so`, `eio/eio-single.so` | 실제 dual/single-arm backend shared library |
| `eio/eio-dg5f` | DG5F hand hardware bridge |
| `rs2/` | RealSense multicam 송수신/녹화 도구 (`msender`, `mreceiver`, `videorec`) |
| `vive/` | Vive tracker + Manus teleop 도구 (`vmaster`, `logger`, `player`, `steamvr-run`, `calib/`, `tests/`) |
| `pyproject.toml`, `uv.lock` | uv 환경 (`pytact` PyPI 패키지 포함) |
