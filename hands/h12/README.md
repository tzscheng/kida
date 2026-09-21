# H12 hand bench

H12는 Robotis X330 두 개와 CAN-FD 모터 열 개로 구성된 12자유도 핸드다.
이 디렉터리는 팔 없이 한쪽 H12를 시뮬레이션하거나 실제 하드웨어에서
제어하는 벤치 프로젝트이며, 왼손과 오른손을 모두 지원한다.

## 구성

| 파일 | 역할 |
| --- | --- |
| [run-h12](run-h12) | Python 실행기. tact 시뮬레이션 또는 `eio.so`로 한 손 제어 |
| [h12.py](h12.py) | 12관절 PD 위치제어, zero/home 궤적, 장갑 목표값 처리 |
| [eio.c](eio.c) → `eio.so` | 실제 작동이 검증된 CAN-FD/X330 하드웨어 브리지 |
| [basic.c](basic.c) → `basic` | `eio.c`를 직접 include하여 UDP 명령으로 12관절 PD 위치제어 |
| [check.c](check.c) → `check` | CAN 모터 10개의 개별 PWM 출력/엔코더 확인 |
| [sockcan.h](sockcan.h) | 로컬 SocketCAN 송수신 함수 |
| [yaml/h12-left.yaml](yaml/h12-left.yaml), [yaml/h12-right.yaml](yaml/h12-right.yaml) | 좌우 모델과 관절 위치·속도 feed 정의 |
| [build.sh](build.sh) | `check`, `basic`, `eio.so` 빌드 |
| `up-fd`, `down` | CAN 인터페이스 활성화/비활성화 |
| `latency.sh`, `usermod.sh` | USB serial latency와 dialout 권한 설정 보조 스크립트 |
| `_/` | 이전 코드/작업 보관 디렉터리 |

실행 경로는 다음과 같다.

```text
ZMQ 명령 → run-h12 → h12.Controller → tact.Env (시뮬레이션)
                                  → tact.CEnv → eio.so → X330 + CAN (실물)
UDP 명령 → basic의 PD 제어 → 포함된 eio.c → X330 + CAN (실물)
키보드   → check → CAN 10개 모터 (실물)
```

## 빌드와 환경

Python 경로는 kida 저장소의 uv 환경에서 PyPI 패키지 `pytact`를 사용한다.
로컬 `../pytact` 소스를 직접 가져오는 구성이 아니다. 최초 Python 환경 준비는
kida 루트에서 `uv sync`로 수행한다.

C 빌드에는 GCC, Linux SocketCAN, Dynamixel C SDK가 필요하다.
현재 SDK 헤더 경로는 `/usr/local/include/dynamixel_sdk`, 링크 라이브러리는
`libdxl_x64_c`다. 이 라이브러리를 런타임 링커에서도 찾을 수 있어야 한다.

```bash
cd /home/ubuntu/kida/devs/h12
bash build.sh
```

`build.sh`는 현재 작업 디렉터리를 기준으로 파일을 찾는다. 이 디렉터리에서
실행해야 한다. `basic`만 빌드하려면 다음을 사용한다.

```bash
gcc -W -Wall -o basic basic.c -lm -I/usr/local/include/dynamixel_sdk -ldxl_x64_c
```

`basic`에는 `eio.c` 구현이 컴파일 시 포함되므로 실행할 때 `eio.so`가 필요하지
않다. 위 명령처럼 컴파일 입력은 `basic.c` 하나로 지정한다. `eio.c`의 보정값이나
통신 코드를 수정한 뒤에는 `basic`을 다시 빌드해야 한다. Python 실물 실행에도
반영하려면 `eio.so`까지 다시 빌드하는 `bash build.sh`를 실행한다.

## 하드웨어 연결

CAN-FD 보드 5개가 모터를 두 개씩 담당하며 송신 ID는
`0x10`, `0x20`, `0x30`, `0x40`, `0x50`이다. X330은 USB serial 버스의
ID 1과 2를 사용한다. Dynamixel Protocol 2.0, baudrate 1 Mbps를 사용한다.

```bash
./up-fd 0       # can0: bitrate/dbitrate 1 Mbps, FD on
./down 0       # can0 비활성화
```

`up-fd`/`down`은 내부에서 `sudo ip link`를 호출한다. `sockcan.h`의 송신은
BRS를 끈 CAN-FD 프레임이며, 수신 timeout은 500 ms다.

USB 접근 권한이 필요하면 `bash usermod.sh`로 현재 사용자를 `dialout`에
추가한다. 새 그룹 적용에는 다시 로그인이 필요할 수 있다.
`bash latency.sh`는 **ttyUSB0**의 `latency_timer`를 1로 설정하므로,
다른 USB 포트를 쓰면 스크립트의 경로도 맞춰야 한다.
같은 하드웨어에 `run-h12 -x`, `basic`, `check`를 동시에 실행하지 않는다.

## 관절 순서와 하드웨어 보정

목표값은 rad, 속도는 rad/s다. 모든 12관절 명령은 아래 순서를 따른다.

| 인덱스 | YAML 관절 | 구동부 |
| --- | --- | --- |
| 0, 1 | thumb1, thumb2 | X330 ID 1, 2 |
| 2, 3 | thumb3, thumb4 | CAN 엄지 굽힘 |
| 4, 5 | index1, index2 | CAN 검지 |
| 6, 7 | middle1, middle2 | CAN 중지 |
| 8, 9 | ring1, ring2 | CAN 약지 |
| 10, 11 | little1, little2 | CAN 소지 |

`h12.Controller.n_u=12`, `n_y=24`이며 관측값은
`y = [q0 ... q11, qd0 ... qd11]`이다. 전류 피드백은 이 벡터에 포함되지 않는다.

`eio.c`를 기준으로 한 현재 보정값은 다음과 같다. `basic.c`는 `eio.c`를
직접 include하므로 보정값은 `eio.c`에서만 갱신한다.

| 항목 | 왼손 (`-t0`) | 오른손 (`-t1`) |
| --- | --- | --- |
| X330 encoder/motor 방향 | `[-1, +1]` | `[+1, -1]` |
| X330 영점 count | `[1486, 1540]` | `[1486, 2500]` |
| CAN encoder/motor 방향 | 10개 모두 `+1` | 10개 모두 `+1` |
| CAN 순열 | `[0, 1, 3, 2, 5, 4, 7, 6, 9, 8]` | 동일 |

X330 위치는 4096 counts/revolution, CAN 위치는 120000 counts/revolution으로
환산한다. 모터 위치 `m`에서 관절 위치 `q`를 구할 때 손가락 연동을 보정한다.

```text
q[0] = m[0], q[1] = m[1]
i = 2, 4, 6, 8, 10:
    q[i]   = m[i]
    q[i+1] = m[i+1] + 0.25*q[i]
```

따라서 `basic`의 목표값도 이 보정을 포함한 관절 위치다. 예전 10축
`basic.c`에서 사용하던 모터 위치와는 다르다.

## run-h12: 시뮬레이션과 Python 제어

```bash
./run-h12 -t0             # 왼손 시뮬레이션
./run-h12 -t1 -l          # 오른손 시뮬레이션, 렌더 창 생략
./run-h12 -t0 -x -c0 -u0  # 왼손 실물, can0 + /dev/ttyUSB0
./run-h12 -t1 -x -c0 -u0 -v
```

`-t`는 필수이며 `0=왼손`, `1=오른손`이다. `-c`와 `-u`는 기본값 0이고
실물 모드에서만 사용한다. `-u`는 `--tty`로도 지정할 수 있다.
`-v`는 Python 컨트롤러의 관절 각도를 degree로 출력한다. 현재 `eio.c`는
이 옵션과 별개로 매 스텝의 관절 위치를 rad로 출력한다.
실행기는 자신의 디렉터리로 이동하므로 절대경로로도 실행할 수 있다.

명령 수신은 ZMQ PULL `ipc:///dev/shm/default`이며, 한 메시지에 한 명령을
공백으로 구분해 보낸다. 팔/손을 쉼표로 나누는 kida-run 명령 형식은 사용하지 않는다.

| 명령 | 동작 |
| --- | --- |
| `zero` | zero 자세로 이동: 700 컨트롤러 tick, 이동평균 smoother 사용 |
| `home` | home 자세로 이동: 700 컨트롤러 tick, 이동평균 smoother 사용 |
| `test` | test 목표값으로 즉시 PD 제어 |
| `joint q0 ... q11` | 12관절 목표값으로 즉시 PD 제어; 장갑 스트림용 |
| `mcheck i` | 인덱스 `i`에 처음 100 tick 동안 출력 `0.4` 적용; 위치제어가 아님 |
| `reset` | 컨트롤러를 재생성하고 환경 reset |
| `quit` | `env.finish()` 후 종료 |

현재 `pid`와 `stop`은 `run-h12` 명령이 아니다. `xmanus`는 파서에는 있지만
`update()`에 대응 제어 분기가 없으므로 장갑 연동에는 `joint`를 사용한다.
`joint` 입력에는 길이/범위 검사가 없으므로 정확히 12개 값을 보낸다.

```text
zero = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
home = [0.4, 1.0, 0.6, 0.6, 0.5, 0.5, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6]
test = [0.3, 1.0, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

예를 들어 다른 터미널에서 ZMQ 명령을 보낼 수 있다.

```bash
uv run python - <<'PY'
import zmq
ctx = zmq.Context()
sock = ctx.socket(zmq.PUSH)
sock.connect('ipc:///dev/shm/default')
sock.send_string('zero')
sock.close()
ctx.term()
PY
```

실물에서는 `has_pd=False`로 외부 PD 출력을 `eio.so`에 전달한다. X330은
Goal Current, CAN 모터는 PWM을 사용한다. `eio.c`는 X330 operating mode를
변경하지 않으므로 X330이 전류제어 모드로 설정되어 있어야 한다.

게인은 전 관절 `kp=1.0`, X330 `kd=0.02`, CAN `kd=0.01`, 적분 게인 0이다.
실행기는 `rate=240`을 전달하지만 현재 궤적 길이는 tick 기준이며 실제 240 Hz
주기를 강제하지 않는다. frameskip은 시뮬레이션 4, 실물 1이고, `eio.c`의
속도 계산은 고정 `DT=0.002`를 사용한다.

`run-h12`에는 proprio PUB 소켓이 없다. YAML에서 선언한 tactile에 대한
PUB 지원은 있지만 현재 좌우 H12 YAML에는 tactile/camera 선언이 없다.

## basic: C 단독 12관절 위치제어

```bash
./basic -t0 -c0 -u0 -v  # 왼손, can0, /dev/ttyUSB0
./basic -t1 -c0 -u0 -v  # 오른손
./basic -h             # 하드웨어를 열지 않고 사용법 출력
```

`-t`는 필수, `-c`/`-u`의 기본값은 0이다. 시작 상태는 정지이며
`-v`는 50 제어 주기마다 모드와 12관절 위치를 rad로 추가 출력한다.
`eio.c` 자체의 관절 출력은 `-v`와 관계없이 매 주기 발생한다.

UDP 포트 **1234**에 한 datagram당 ASCII 명령 하나를 보낸다.

| 명령 | 동작 |
| --- | --- |
| `joint q0 ... q11` | 정확히 12개의 유한한 rad 목표값으로 PD 제어 |
| `stop` | X330 전류 0, CAN PWM 중립; 위치를 유지하는 명령이 아님 |
| `quit` | 출력 차단과 X330 torque disable 후 종료 |

```bash
python3 - <<'PY'
import socket
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.sendto(b'joint 0 0.5 0 0 0 0 0 0 0 0 0 0', ('127.0.0.1', 1234))
PY
```

목표값은 trajectory smoothing이나 관절 범위 clipping 없이 적용되며 다음
명령까지 유지된다. 잘못된 명령은 기존 목표값과 모드를 바꾸지 않는다.
`pid`, `zero`, `test`, `home`은 지원하지 않는다.

`basic.c`는 `#include "eio.c"`로 하드웨어 구현을 직접 포함하고
`init/step/finish`를 호출한다. UDP 명령 처리, 외부 PD, 주기 제어는
`basic.c`에 남아 있다. X330은 사전에 current mode(0)로 설정되어 있어야 하며,
초기화 시 operating mode를 자동 변경하지 않는다. 첫 `step`은 출력 0으로
초기 관절 위치를 읽는다.

| 설정 | X330 2관절 | CAN 10관절 |
| --- | --- | --- |
| `kp` | `1.0` | `0.3` |
| `kd` | `0.02` | `0.005` |
| 적분 게인 | 0 | 0 |
| 출력 제한 | `motor_dir * 1000 * tau`, raw register 값 ±600 | normalized voltage ±1, PWM `0..65535` |

X330 게인은 `h12.py`, CAN 게인은 이전 `basic.c` 값을 사용한다. 전류 레지스터
스케일과 제한은 `eio.c`와 동일하며, ±600은 코드의 raw 명령값이다.
제어 루프는 2 ms를 목표로 하고 속도는 `eio.c`의 고정 `DT=0.002`로 계산한다.
`step` 통신 오류, quit, SIGINT/SIGTERM 시 `finish()`로 정리한다.
`finish()`는 CAN 중립 PWM 송수신 후 X330 torque를 disable하고 포트를 닫는다.
초기화 오류 처리는 `eio.c`의 `init()`을 따르며 내부에서 바로 종료할 수 있다.

## check: CAN 모터 개별 확인

```bash
./check -c0
```

위치제어 대신 선택한 CAN 모터에 직접 PWM을 가한다. X330은 다루지 않는다.

| 키 | 동작 |
| --- | --- |
| `1` … `9`, `0` | CAN 모터 10개 중 하나 선택; 출력 단계 0으로 초기화 |
| `[` / `]` | 출력 단계 감소/증가, 범위 `-8..8` |
| Space | 출력 단계 0 |
| `q` | 중립 PWM 전송 후 종료, 통신 주파수 출력 |

단계당 duty 변화량은 `0x1000`, 중립은 `0x8000`이다. 모터 map/direction은
`check.c`에 별도로 들어 있으며 좌우 선택 옵션은 없다.

## Manus 연동

`/home/ubuntu/ongoing/dev/manus/manus-out -g3`은 H12용 12관절 `joint` 명령을
만든다. 기본 ZMQ 출력은 `run-h12`의 `ipc:///dev/shm/default`에 연결할 수 있다.
`manus-out`은 자신의 `calib/`를 참조하므로 해당 디렉터리에서 실행한다.

ZMQ 메시지에는 좌우 구분자가 없고 감지된 각 장갑의 명령을 같은 endpoint로
보내므로, 한 손 벤치에 연결할 때는 의도한 장갑의 데이터만 전달되도록 해야 한다.
`manus-out -u`는 `L/R` 접두사와 UDP 포트 6633을 사용하므로 `basic`의
포트 1234/`joint` 프로토콜에 직접 호환되지 않는다.

## 검증 범위

`eio.c`를 직접 include하는 `basic.c`는 경고를 오류로 처리한 컴파일과
하드웨어를 열지 않는 도움말/인자 검증을 확인했다. 변경 후 전체 실물 제어
루프는 아직 벤치 검증 전이다.
