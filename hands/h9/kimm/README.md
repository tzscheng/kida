# kimm — Hand Runner (h9)

KIMM(생기원) 로봇핸드 사용설명서(`261110_핸드1_사용설명`)에서 말하는 **Hand Runner** 구현이다.
사용자 프로그램은 핸드 펌웨어와 직접 통신하지 않고 runner에 UDP 문자열 명령을 보내고,
runner가 CAN 버스, PWM duty 제어 루프, 설정 파일(`conf.txt`)을 담당한다.
핸드 한 대당 runner 한 개를 띄우며, 여러 대면 UDP 포트를 다르게 해서 여러 개 띄운다.

```
사용자 프로그램 --(UDP 명령)--> runner --(CAN)--> 핸드
사용자 프로그램 <--(UDP 상태 100Hz)-- runner
```

상위 `h9/` 폴더의 `eio.c`, `h9.py`, `run-h9`와 같은 역할이지만 Python/ZMQ 없이
단일 C 파일로 독립 동작한다. 그쪽 파일과는 코드를 공유하지 않는다.

## 파일

| 파일 | 설명 |
| --- | --- |
| `runner.c` | Hand Runner 소스 (단일 파일). CAN 송수신, 엔코더→관절각 변환, PD 제어, 궤적 생성, UDP 프로토콜 |
| `runner` | 실물용 빌드 (SocketCAN 사용) |
| `runner-sim` | 시뮬레이션 빌드 (`-DRUNNER_SIM`). CAN 없이 소프트웨어 모터 모델로 동작, 프로토콜/제어 루프 테스트용 |
| `runner-test` | curses 대화형 UDP 테스트 도구 (Python) |
| `conf.txt` | 핸드별 설정 파일 (`h9/conf.txt` 복사본) |
| `build.sh` | `runner`, `runner-sim` 빌드 |

## 빌드

```bash
./build.sh
```

의존성은 gcc와 libm뿐이다. `runner-test`는 상위 `kida` 저장소의 uv 환경(파이썬 표준 라이브러리만 사용)으로 실행된다.

## 실행

```bash
# 1. CAN 활성화 (환경에 맞게)
sudo ip link set can0 up type can bitrate 1000000

# 2. runner 실행 (conf.txt가 있는 이 폴더에서)
./runner -p 10000 -n L2
```

| 옵션 | 설명 | 기본값 |
| --- | --- | --- |
| `-p` | 명령 수신 UDP 포트 (필수) | |
| `-n` | 핸드 이름 = `conf.txt`의 항목 이름 (필수) | |
| `-c` | SocketCAN 채널 번호 (`canN`) | 0 |
| `-f` | 설정 파일 경로 | `conf.txt` |
| `-m` | 핸드 모델 | `h9` |
| `-s` | 상태 송신 UDP 포트 (127.0.0.1) | 24000 |
| `-v` | 관절각을 초당 10회 출력 | |

핸드 두 대 예시:

```bash
./runner -p 10000 -n L2 -c 0 &
./runner -p 10001 -n R2 -c 1 &
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

- **제어 루프**: 250 Hz (`DT 0.004`). 매 주기 CAN 보드 3개에 duty 프레임을 보내고 엔코더 값을 받은 뒤 PD 제어 (`kp 2.0`, `kd 0`, 오차 단위 rad, 출력 -1~1 정규화 duty).
- **궤적**: 이동평균 스무더. `home`/`zero`는 500 tick(2초), `jointpos`는 오차 크기에 비례한 짧은 시간이라 사실상 계단 명령에 가깝다. 모두 `h9.py`와 같은 규칙.
- **시작 상태**: 명령이 올 때까지 duty 0 (모터 free). 첫 명령부터 위치를 유지한다.
- **엔코더 턴 카운트**: 12비트 단일회전 엔코더이므로 시작 시 "영점에서 모터 반바퀴 이내"가 되도록 턴 카운트를 초기화한다. 핸드가 영점 근처에서 시작한다는 전제.
- **CAN 실패**: 교환이 실패하면 모터를 풀고(idle) 계속 재시도하며, 새 명령이 오면 다시 추종한다.
- **종료**: `quit`, Ctrl-C, SIGTERM 모두 duty 0을 보낸 뒤 종료한다.

## conf.txt

한 줄에 핸드 하나. `h9/conf.txt`와 같은 형식이며 **복사본**이므로 영점을 바꿀 때 어느 쪽을 쓰는지 주의한다.

```
이름  타입(L/R)  엄지pulley  손가락pulley  couple0  couple1  영점9개(엔코더 raw, 관절 순서)
L2    L          0.422       0.250         0.75     0.75     590 1516 1277 1326 2884 1991 2751 2287 2426
```

영점 값은 핸드를 zero 자세에 두고 `h9/check`로 읽은 엔코더 raw 값이다.

## 테스트

### 시뮬레이션

```bash
./runner-sim -p 10000 -n L2        # CAN 없이 동작
```

### runner-test (대화형)

```bash
./runner-test -p 10000             # 명령 포트
./runner-test -p 10000 -n L2       # 핸드가 여러 대일 때 목표/오차 행을 표시할 핸드
./runner-test -p 10000 -s 24001    # runner를 -s 24001 로 띄웠을 때
```

- 위쪽: 핸드별 현재각, 수신 Hz, 마지막 패킷 경과 시간. 1초 이상 끊기면 `NO STATUS`.
- 마지막 명령의 목표각(`target`)과 오차(`error`) 행.
- 가운데: 송신/오류 로그. 아래: 입력줄 (위/아래 화살표로 이전 명령).
- `jointpos`는 보내기 전에 숫자 개수와 형식을 검사한다.
- `exit` 또는 Ctrl-C는 runner-test만 종료한다. runner까지 끝내려면 `quit`을 먼저 보낸다. `clear`는 로그 삭제.
- 24000 포트를 점유하므로 사용자 프로그램과 동시에 쓰려면 runner와 runner-test 모두 `-s`로 다른 포트를 지정한다.

### udpmsg (단발 명령)

```bash
../../../utils/udpmsg -p 10000 jointpos 10 20 30 0 -10 0 -25.8 0 30
../../../utils/udpmsg -p 10000 quit
```

## 다른 핸드 추가

`runner.c`의 `hand_driver_t` 인터페이스(`init` / `exchange` / `finish`, 관절 수, home 자세, 클램프, 게인)를 구현한
드라이버 블록을 추가하고 `drivers[]` 테이블에 등록하면 `-m <모델>`로 선택할 수 있다.
UDP 프로토콜, 궤적 생성, 제어 루프는 공용이다.

## 실물 첫 시험 시 권장 순서

1. 핸드를 zero 자세 근처에 두고 전원/CAN 연결 (PC 쪽 120Ω 종단 저항 필요).
2. `./runner -p 10000 -n L2 -v`로 띄워 초기 관절각 출력이 0 근처인지 확인.
3. `runner-test`로 `zero` → `home` → 작은 `jointpos` 순으로 시험.
4. 필요하면 `h9_qmin_deg` / `h9_qmax_deg`, `kp`, `pose_ticks`를 조정.
