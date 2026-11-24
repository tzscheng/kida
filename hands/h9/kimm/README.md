# kimm — Hand Runner (h9)

KIMM(생기원) 로봇핸드 사용설명서(`261110_핸드1_사용설명`)에서 말하는 **Hand Runner** 구현이다.
사용자 프로그램은 핸드 펌웨어와 직접 통신하지 않고 runner에 UDP 문자열 명령을 보내고,
runner가 CAN 버스, PWM duty 제어 루프, 설정 파일(`conf.txt`)을 담당한다.
핸드 한 대당 runner 한 개를 띄우며, 여러 대면 UDP 포트를 다르게 해서 여러 개 띄운다.

```
사용자 프로그램 --(UDP 명령)--> runner --(CAN)--> 핸드
사용자 프로그램 <--(UDP 상태 100Hz)-- runner
```

이 폴더는 **단독으로 동작한다.** 폴더 밖의 코드나 파일을 참조하지 않으므로 폴더만 복사해서 쓰면 된다.

## 파일

| 파일 | 설명 |
| --- | --- |
| `runner.c` | Hand Runner 소스 (단일 파일). CAN 송수신, 엔코더→관절각 변환, PD 제어, 궤적 생성, UDP 프로토콜 |
| `runner` | 실물용 빌드 (SocketCAN 사용) |
| `runner-sim` | 시뮬레이션 빌드 (`-DRUNNER_SIM`, `build.sh`의 주석 처리된 줄). CAN 없이 소프트웨어 모터 모델로 동작, 프로토콜/제어 루프 테스트용 |
| `runner-test` | curses 대화형 UDP 테스트 도구 (Python) |
| `monitor.c` / `monitor` | 모터/엔코더 점검 및 영점 읽기 도구 (curses 화면, CAN 직접 사용) |
| `manus.cpp` / `manus` | Manus 장갑 → runner teleoperation (아래 Manus 장갑 절) |
| `manus-sdk/` | Manus SDK 3.1.1 (헤더, `libManusSDK_Integrated.so` 약 112 MB). `manus`의 빌드와 실행에 쓴다. `.so`는 git에 올라가지 않으므로 폴더를 옮길 때 직접 복사한다 |
| `calib/` | Manus 장갑 calibration 파일 (`.mcal`) |
| `conf.txt` | 핸드별 설정 파일 |
| `build.sh` | `runner`, `monitor`, `manus` 빌드 (`runner-sim`, `monitor-sim`, `manus-sim` 줄은 주석 처리) |
| `can-up` | CAN 인터페이스 활성화 (`./can-up 0` → `can0`, 1 Mbps) |

## 빌드

```bash
./build.sh
```

필요한 것 (Linux, SocketCAN):

- 빌드: `gcc`, ncurses 개발 패키지 (`monitor`용. Ubuntu: `sudo apt install build-essential libncurses-dev`)
- `manus`: `g++`, 그리고 SDK가 쓰는 시스템 라이브러리 `libusb-1.0`, `libudev`, `libz`, `libcap` (Ubuntu: `sudo apt install libusb-1.0-0 libudev1 zlib1g libcap2`, 보통 이미 깔려 있다)
- `runner-test`: `python3` (3.7 이상, 표준 라이브러리만 사용)
- `can-up`: `ip` (iproute2), `sudo` 권한

## 실행

```bash
# 1. CAN 활성화 (환경에 맞게)
./can-up 0                         # = sudo ip link set can0 type can bitrate 1000000; sudo ip link set can0 up

# 2. runner 실행 (conf.txt가 있는 이 폴더에서)
./runner L2                        # 핸드 이름은 옵션 없이 인자로 준다 (포트 등은 ./runner L2 -p 10000)
```

| 옵션 | 설명 | 기본값 |
| --- | --- | --- |
| `<핸드 이름>` | `conf.txt`의 항목 이름 (필수, 옵션 없이 주는 인자) | |
| `-p` | 명령 수신 UDP 포트 | `10000` |
| `-c` | SocketCAN 채널 번호 (`canN`) | 0 |
| `-f` | 설정 파일 경로 | `conf.txt` |
| `-m` | 핸드 모델 | `h9` |
| `-s` | 상태 송신 UDP 포트 (127.0.0.1) | 24000 |
| `-v` | 관절각을 초당 10회 출력 | |

핸드 두 대 예시:

```bash
./runner L2 -p 10000 -c 0 &
./runner R2 -p 10001 -c 1 &
```

두 runner 모두 상태를 24000 포트로 보내며, 상태 메시지의 첫 단어(핸드 이름)로 구분한다.

## UDP 프로토콜

문자열 기반이며 `Head + Tail` 구조다. 한 datagram에 명령 하나.

### 송신 (사용자 프로그램 → runner, `-p` 포트)

| Head | Tail | 설명 |
| --- | --- | --- |
| `jointpos` | 관절 목표각 9개 (degree) | 예: `jointpos 10 20 30.0 0.0 -10 0 -25.8 0 30` |
| `home` | 없음 | home 자세로 이동 (2초) |
| `zero` | 없음 | zero 자세로 이동 (2초) |
| `quit` | 없음 | runner 종료 |

관절 순서: `thumb1 thumb2 thumb3 index1 index2 middle1 middle2 ring1 ring2`, 양수가 굽힘 방향.
인자 개수나 형식이 틀리면 명령을 무시하고 stderr에 이유를 출력한다.
목표각은 -60 ~ 120도로 클램프된다 (`runner.c`의 `h9_qmin_deg` / `h9_qmax_deg`).

### 수신 (runner → 사용자 프로그램, 127.0.0.1:24000)

명령과 무관하게 100 Hz로 전송한다.

| Head | Tail |
| --- | --- |
| 핸드 이름 | 현재 관절각 9개 (degree) |

예: `L2 10.00 20.00 30.00 0.00 10.00 0.00 20.00 0.00 30.00`

## 동작 개요

- **제어 루프**: 250 Hz (`DT 0.004`). 매 주기 CAN ID 3개(ID 하나에 모터 3개·엔코더 3개. R 핸드 0x10 0x20 0x30, L 핸드 0x10 0x20 0x40)로 duty 프레임을 보내고 엔코더 값을 받은 뒤 PD 제어 (`kp 2.0`, `kd 0`, 오차 단위 rad, 출력 -1~1 정규화 duty).
- **궤적**: 이동평균 스무더. `home`/`zero`는 500 tick(2초), 단발 `jointpos`는 오차 크기에 비례한 짧은 시간이라 사실상 계단 명령에 가깝다.
  추종 중에 새 명령이 오면 측정각이 아니라 직전 기준각에서 이어 간다 (측정각에서 다시 시작하면 그 순간 오차와 duty가 0으로 떨어진다).
- **연속 명령 (teleoperation)**: `jointpos`가 0.2초 안에 이어서 오면 연속 명령으로 보고, 기준각을 명령 주기(30 Hz면 약 33 ms)에 걸쳐 새 목표까지 직선으로 옮긴다. 그만큼의 지연이 생기는 대신 기준각이 계단 모양이 되지 않는다.
- **시작 상태**: 명령이 올 때까지 duty 0 (모터 free). 첫 명령부터 위치를 유지한다.
- **엔코더 턴 카운트**: 12비트 단일회전 엔코더이므로 시작 시 "영점에서 모터 반바퀴 이내"가 되도록 턴 카운트를 초기화한다. 핸드가 영점 근처에서 시작한다는 전제.
- **CAN 응답 확인**: 응답은 요청 ID + 1로 온다 (0x10 → 0x11, 0x20 → 0x21, 0x30 → 0x31, 0x40 → 0x41).
  `runner`와 `monitor`는 그 ID의 프레임만 해당 요청의 엔코더 값으로 받고, 타임아웃(0.5초) 뒤 늦게 도착한 다른 요청의 응답은 버린다.
- **CAN 실패**: 교환이 실패하면 모터를 풀고(idle) 계속 재시도하며, 새 명령이 오면 다시 추종한다.
- **종료**: `quit`, Ctrl-C, SIGTERM 모두 duty 0을 보낸 뒤 종료한다.

## conf.txt

한 줄에 핸드 하나. `runner`와 `monitor`가 같은 파일을 읽는다.

```
이름  타입(L/R)  pulley0  pulley1  couple0  couple1  영점9개(엔코더 raw, 관절 순서)
L2    L          0.422    0.250    0.75     0.75     590 1516 1277 1326 2884 1991 2751 2287 2426
```

- `pulley0`: 모터각 → 관절각 비율 (thumb1을 뺀 8개 관절). `pulley1`: thumb1(엄지 CMC)의 비율.
- `couple0`: 같은 손가락의 앞 관절이 다음 관절에 더해지는 비율. `couple1`: thumb1이 thumb2, thumb3에 더해지는 비율.

영점 값은 핸드를 zero 자세에 두고 `./monitor <이름>`의 `MAG` 행에서 읽은 엔코더 raw 값이다 (아래 monitor 절).

## 테스트

### 시뮬레이션

`runner-sim`은 기본 빌드에 들어 있지 않다. `build.sh`의 주석 처리된 줄을 풀거나 직접 빌드한다.

```bash
gcc -W -Wall -O2 -DRUNNER_SIM -o runner-sim runner.c -lm
./runner-sim L2 -p 10000           # CAN 없이 동작
```

### runner-test (대화형)

```bash
./runner-test                      # 명령 포트 기본 10000 (-p 로 변경)
./runner-test L2 -p 10000          # 핸드가 여러 대일 때 목표/오차 행을 표시할 핸드
./runner-test -p 10000 -s 24001    # runner를 -s 24001 로 띄웠을 때
```

- 위쪽: 핸드별 현재각, 수신 Hz, 마지막 패킷 경과 시간. 1초 이상 끊기면 `NO STATUS`.
- 마지막 명령의 목표각(`target`)과 오차(`error`) 행.
- 가운데: 송신/오류 로그. 아래: 입력줄 (위/아래 화살표로 이전 명령, 좌/우 화살표·Home·End로 커서를 옮겨 중간 수정, Delete·Backspace로 삭제).
- `jointpos`는 보내기 전에 숫자 개수와 형식을 검사한다.
- `exit` 또는 Ctrl-C는 runner-test만 종료한다. runner까지 끝내려면 `quit`을 먼저 보낸다. `clear`는 로그 삭제.
- 24000 포트를 점유하므로 사용자 프로그램과 동시에 쓰려면 runner와 runner-test 모두 `-s`로 다른 포트를 지정한다.

### monitor (모터/엔코더 점검)

관절 하나를 고정 duty로 구동하면서 전체 관절의 엔코더 raw 값과 관절각을 curses 화면으로 보여준다.
CAN을 직접 쓰므로 **같은 핸드의 runner와 동시에 실행하지 않는다.**

```bash
./monitor L2                       # ./monitor L2 -c 1 -f conf.txt, -n L2 로 줘도 된다
```

| 키 | 동작 |
| --- | --- |
| `1`~`9` | 구동할 관절 선택 (level은 0으로) |
| `[` / `]` | level -1 / +1 (-8~8, 1단계 = duty 0x1000) |
| space | level 0 (모터 free) |
| `q`, Ctrl-C | 종료 (duty 0을 보낸 뒤) |

- 관절별 `DUTY`(송신 duty), `MAG`(엔코더 raw), `ZPOS`(영점 기준 카운트), `DEG`(관절각). 선택한 관절 열이 반전 표시된다.
- zero 자세에서 읽은 `MAG` 행이 `conf.txt`의 영점 9개다.
- 사용 방법(키)은 화면 위쪽에 항상 표시된다.
- CAN 응답이 없으면 화면에 `CAN ERROR`와 원인을 표시하고 마지막 값을 유지한 채 계속 시도한다. 버린 프레임 수는 `stray frames dropped`로 표시된다.
- 하드웨어 없이 화면만 볼 때는 `build.sh`의 `monitor-sim` 줄(`-DMONITOR_SIM`)로 빌드한다.

### 단발 명령 (bash)

별도 도구 없이 bash의 `/dev/udp`로 datagram 하나를 보낼 수 있다.

```bash
printf 'jointpos 10 20 30 0 -10 0 -25.8 0 30' > /dev/udp/127.0.0.1/10000
printf 'quit' > /dev/udp/127.0.0.1/10000
```

## Manus 장갑 teleoperation

`manus`는 Manus 장갑의 관절각을 h9 관절 9개로 바꿔 runner에 `jointpos` 명령으로 보낸다 (30 Hz).
runner는 핸드당 하나이므로 왼손과 오른손 장갑은 서로 다른 포트로 보낸다.

```
장갑 --(Manus SDK)--> manus --(UDP jointpos)--> runner --(CAN)--> 핸드
```

```bash
./runner L2 -p 10000 -c 0 &
./runner R2 -p 10001 -c 1 &
./manus                            # 왼손 -> 10000, 오른손 -> 10001
./manus -r 0                       # 왼손만 (0 = 그 쪽은 보내지 않음)
```

| 옵션 | 설명 | 기본값 |
| --- | --- | --- |
| `-l` | 왼손 runner의 명령 포트 (0 = 보내지 않음) | `10000` |
| `-r` | 오른손 runner의 명령 포트 (0 = 보내지 않음) | `10001` |
| `-a` | runner의 IP 주소 | `127.0.0.1` |
| `-d` | 장갑 calibration 파일 폴더 | `calib` |
| `-n` | calibration 파일을 읽지 않음 | |
| `-v` | 보내는 메시지를 모두 출력 | |

- **시작**: 장갑 데이터가 들어오는 즉시 장갑 자세를 그대로 보낸다. 시작 동작이 따로 없으므로 첫 명령에서 핸드가 장갑 자세로 빠르게 움직인다.
- **장갑 데이터가 끊기면** (1초 이상) 아무것도 보내지 않고, runner는 마지막 자세를 유지한다. 다시 들어오면 바로 장갑 자세를 따라간다.
- **종료** (Ctrl-C): 더 보내지 않으며 핸드는 마지막 자세를 유지한다. 되돌리거나 끝내려면 runner에 `zero` / `quit`을 보낸다.
- **Manus SDK**: `manus-sdk/`에 들어 있어 따로 설치할 필요가 없다. `manus`는 자기 옆의 `manus-sdk/lib`에서 라이브러리를 찾으므로 어느 폴더에서 실행해도 된다 (단, `calib/`는 실행한 폴더 기준이므로 다른 곳에서 실행하면 `-d`로 준다). 새 PC에서는 Manus 동글의 USB 권한(udev)과 라이선스가 따로 필요하다.
- SDK가 실행한 폴더에 빈 `log.txt`를 만든다. 지워도 된다.
- **calibration**: `calib/`의 파일 이름은 `manus.cpp`의 `kLeftCalib` / `kRightCalib`에 고정되어 있다. 장갑을 끼는 사람이 바뀌면 그 사람의 `.mcal`로 바꾼다.
- **retarget 값** (장갑 관절 → h9 관절의 gain, offset)은 `manus.cpp`의 `RetargetH9` 표에 있다.
- 장갑 없이 시험할 때는 `build.sh`의 `manus-sim` 줄(`-DMANUS_SIM`)로 빌드한다. 손을 4초 주기로 쥐었다 펴는 가상 데이터를 보내므로 `runner-sim`과 함께 쓸 수 있다.

## 다른 핸드 추가

`runner.c`의 `hand_driver_t` 인터페이스(`init` / `exchange` / `finish`, 관절 수, home 자세, 클램프, 게인)를 구현한
드라이버 블록을 추가하고 `drivers[]` 테이블에 등록하면 `-m <모델>`로 선택할 수 있다.
UDP 프로토콜, 궤적 생성, 제어 루프는 공용이다.

## 실물 첫 시험 시 권장 순서

1. 핸드를 zero 자세 근처에 두고 전원/CAN 연결 (PC 쪽 120Ω 종단 저항 필요).
2. `./runner L2 -v`로 띄워 초기 관절각 출력이 0 근처인지 확인.
3. `runner-test`로 `zero` → `home` → 작은 `jointpos` 순으로 시험.
4. 필요하면 `h9_qmin_deg` / `h9_qmax_deg`, `kp`, `pose_ticks`를 조정.
