# KIDA

KIDA platform runner — 14-DOF dual-arm manipulator 시뮬레이터/실로봇 구동 플랫폼.
시뮬레이션과 실제 하드웨어를 동일한 ZeroMQ 인터페이스로 제어합니다.

## Quick start

```bash
uv sync               # 의존성 설치 (pyproject.toml / uv.lock 기준)
./kida                # 시뮬레이터 실행
python usrsample.py   # 예제 클라이언트 실행 (다른 터미널에서)
```

## 통신 구조

`kida`(서버)와 사용자 코드(클라이언트)는 ZeroMQ로 메시지를 주고받습니다.
로컬이면 IPC(`ipc:///dev/shm/...`), 원격이면 TCP(`tcp://<host>:<port>`)를 씁니다.

| 채널 | 엔드포인트(로컬 / TCP) | 방향 | 내용 |
|---|---|---|---|
| command  | `default` / `5555`  | client → kida | 로봇 명령 (init/joint/task/rest/quit 등) |
| proprio  | `proprio` / `5556`  | kida → client | 관절 상태 162×float32 (실로봇 120Hz, 시뮬 30Hz) |
| headcam  | `headcam` / `5557`  | kida → client | 머리 카메라 RGB (640×480@30Hz, JPEG) |
| leftcam  | `leftcam` / `5558`  | kida → client | 좌측 카메라 |
| rightcam | `rightcam` / `5559` | kida → client | 우측 카메라 |

## Files

### 실행 파일 / 바이너리

| 파일 | 설명 |
|---|---|
| `kida`     | 메인 플랫폼 러너. `./kida` → 시뮬레이터 실행, `./kida -x` → 실제 하드웨어 구동 |
| `mreceiver`| ZeroMQ 메시지 수신 테스트용 바이너리 |
| `msender`  | ZeroMQ 메시지 송신 테스트용 바이너리 |
| `eio/eio-dg5`       | 외부 I/O 모듈 — DG5 하드웨어용 실행 파일 |
| `eio/eio-kida.so`   | 외부 I/O 플러그인 — kIDA 플랫폼용 공유 라이브러리 |
| `eio/eio-single.so` | 외부 I/O 플러그인 — 단일 장치(single)용 공유 라이브러리 |

### 스크립트

| 파일 | 설명 | 사용법 |
|---|---|---|
| `up`          | CAN 통신 채널 활성화 (bitrate 1Mbps) | `./up 0` → CAN 채널 0 up |
| `down`        | CAN 통신 채널 비활성화                 | `./down 0` → CAN 채널 0 down |
| `usrsample.py`| 사용자 예제 클라이언트. proprio/카메라 수신 + init→joint→task→rest 동작 시퀀스 데모 | `python usrsample.py` |
| `zmqmsg`      | 커맨드 채널로 메시지를 직접 보내는 CLI 도구 (단발/콘솔/kbhit 모드) | `./zmqmsg <msg>` , `./zmqmsg -k` |

### 설정 (`yml/`)

로봇·환경의 시뮬레이션 모델 정의 (물성/관절/뷰 등).

| 파일 | 대상 |
|---|---|
| `kida.yml`       | 14-DOF dual-arm manipulator (전체) |
| `kida-left.yml`  | 7-DOF 좌측 팔 |
| `kida-right.yml` | 7-DOF 우측 팔 |
| `h9-left.yml`    | A9 핸드 (좌) |
| `h9-right.yml`   | A9 핸드 (우) |
| `dg5-left.yml`   | DG5 핸드 (좌) |
| `dg5-right.yml`  | DG5 핸드 (우) |
| `desk1.yml`      | 테스트 환경 (책상/물체) |

### 프로젝트 메타

| 파일 | 설명 |
|---|---|
| `pyproject.toml`  | Python 프로젝트 정의 및 의존성 (numpy, pyzmq, opencv, diffusers 등) |
| `uv.lock`         | uv 잠금 파일 (의존성 버전 고정) |
| `.python-version` | Python 버전 지정 |

## 명령 예시 (command 채널)

```
init, home, home                              # 초기화 + 양손 home 포즈
joint <팔 14>, <좌손 20>, <우손 20>           # 관절공간(joint-space) 명령
task  <좌팔 6D> <우팔 6D>, home, home         # 작업공간(task-space) 명령
rest                                          # 팔 내리기
quit                                          # 종료
```

자세한 사용 예시는 `usrsample.py` 참고.
