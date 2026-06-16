# KIDA 배포 패키지

KIDA는 14-DOF dual-arm manipulator와 양손 gripper를 시뮬레이션 또는 실제 하드웨어에서 구동하는 실행 패키지입니다. `kida`/`single` 실행 파일이 ZeroMQ command를 받고, proprioception과 camera frame을 publish합니다.

이 README는 배포 패키지를 압축 해제한 뒤 실행하는 방법을 기준으로 합니다.

## 설치 및 실행

배포받은 PC에서 압축을 풉니다.

```bash
tar -xzf kida.tar.gz
cd kida
```

Python client 예제를 사용하려면 uv 의존성을 설치합니다.

```bash
uv sync
```

시뮬레이터 실행:

```bash
./kida
```

다른 터미널에서 예제 client 실행:

```bash
uv run python usrsample.py
```

명령을 직접 보낼 때:

```bash
./zmqmsg init, home, home
./zmqmsg rest
./zmqmsg quit
```

## 실행 파일

| 파일 | 용도 |
|---|---|
| `kida` | Dual-arm + 양손 실행기. 기본은 시뮬레이터, `-x`는 실제 하드웨어 |
| `single` | 좌/우 한쪽 arm + hand 실행기 |
| `usrsample.py` | ZeroMQ client 예제. command 송신, proprioception/camera 수신 |
| `zmqmsg` | command channel로 문자열을 보내는 CLI 도구 |
| `msender` | RealSense camera frame 송신기 |
| `mreceiver` | camera/message 수신 테스트 도구 |
| `up` / `down` | CAN interface 활성화/비활성화 스크립트 |

## KIDA 실행 옵션

Dual-arm:

```bash
./kida                 # DG5 hand 시뮬레이터
./kida -g 0            # H9 hand 시뮬레이터
./kida -g 1 -l         # DG5 hand headless 시뮬레이터
./kida -g 1 -x         # 실제 KIDA + DG5 hand
./kida -g 1 -x -b      # 실제 KIDA, actuator built-in position controller 사용
./kida -g 1 -v         # TCP pose verbose 출력
```

Single-arm:

```bash
./single -t 0 -g 1     # left arm + DG5 hand 시뮬레이터
./single -t 1 -g 1     # right arm + DG5 hand 시뮬레이터
./single -t 0 -g 0 -x  # left arm + H9 hand 실제 하드웨어
./single -t 1 -g 1 -x  # right arm + DG5 hand 실제 하드웨어
```

공통 옵션:

| 옵션 | 의미 |
|---|---|
| `-g 0` | H9 hand |
| `-g 1` | DG5 hand |
| `-x` | 실제 하드웨어 backend 사용 |
| `-b` | 실제 하드웨어에서 actuator built-in PD/position mode 사용 |
| `-l` | 시뮬레이터 rendering window 없이 실행 |
| `-v` | controller debug/pose 출력 |
| `-d <file>` | `<step_count> <command>` 형식 dispatch file 실행 |
| `-t 0` | `single`에서 left arm 선택 |
| `-t 1` | `single`에서 right arm 선택 |

## 실제 하드웨어 준비

CAN interface를 먼저 올립니다. 장치명은 환경에 맞게 선택합니다.

```bash
./up 0
./up 1
```

DG5 hand를 사용할 때는 `kida -x` 또는 `single -x`가 `eio/eio-dg5`를 사용합니다. RealSense camera stream은 별도 프로세스로 실행합니다.

```bash
./msender
```

종료 후 CAN interface를 내립니다.

```bash
./down 0
./down 1
```

## ZeroMQ 통신

`kida`와 `single`은 같은 endpoint를 사용합니다. 같은 PC에서는 IPC, 다른 PC에서는 TCP로 접속합니다.

| 채널 | 로컬 endpoint | TCP endpoint | 방향 | 내용 |
|---|---|---|---|---|
| command | `ipc:///dev/shm/default` | `tcp://<robot-ip>:5555` | client -> runner | `init`, `joint`, `task`, `rest`, `quit` 등 |
| proprio | `ipc:///dev/shm/proprio` | `tcp://<robot-ip>:5556` | runner -> client | float32 proprioception |
| headcam | `ipc:///dev/shm/headcam` | `tcp://<robot-ip>:5557` | runner/camera -> client | JPEG image |
| leftcam | `ipc:///dev/shm/leftcam` | `tcp://<robot-ip>:5558` | runner/camera -> client | JPEG image |
| rightcam | `ipc:///dev/shm/rightcam` | `tcp://<robot-ip>:5559` | runner/camera -> client | JPEG image |

`usrsample.py`에서 `SERVER = None`이면 로컬 IPC를 사용합니다. 원격 PC에서 접속할 때는 `SERVER = '<robot-ip>'`로 바꿉니다.

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

Dual-arm `kida`의 proprioception은 총 162개 `float32`입니다.

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

## 포함 파일

| 경로 | 설명 |
|---|---|
| `yml/kida.yml` | Dual-arm robot model |
| `yml/kida-left.yml` | Left single-arm model |
| `yml/kida-right.yml` | Right single-arm model |
| `yml/h9-left.yml`, `yml/h9-right.yml` | H9 hand model |
| `yml/dg5-left.yml`, `yml/dg5-right.yml` | DG5 hand model |
| `yml/desk1.yml` | 시뮬레이션 desk scene |
| `eio/eio-kida.so` | 실제 dual-arm backend shared library |
| `eio/eio-single.so` | 실제 single-arm backend shared library |
| `eio/eio-dg5` | DG5 hand hardware bridge |
| `pyproject.toml`, `uv.lock` | Python client 예제 실행용 uv 환경 |
