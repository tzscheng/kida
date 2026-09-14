# vive-working — 손 리타게팅 확인 도구

`vive/retarget.h` 계수를 팔 없이 확인하기 위한 파일들입니다. 원본 `vive/` 를
건드리지 않고 **이 폴더에서 그대로 빌드·실행**합니다. 작업이 끝나 본가지에
반영할 때만 아래 "원본에 반영"을 따르면 됩니다.

## 왜 필요한가

`vive/vmaster`는 `main()`에서 OpenVR을 초기화하고 트래커를 못 찾거나 개수가
`-t`와 안 맞으면 exit 3/4로 종료합니다. 즉 손 관절 매핑만 보고 싶어도 SteamVR과
Vive 트래커 전체를 켜야 했습니다. 여기 있는 두 개가 그 결합을 끊습니다.

- `vhand` — 글러브 전용 마스터. 글러브 → `retarget.h` → 손 `joint` 명령, 끝.
  OpenVR을 링크하지 않으므로 SteamVR이 필요 없습니다.
- `hand-run` — 손 YAML만 로드하는 tact 시뮬레이터. 팔도 IK도 없습니다.

## 파일

| 파일 | 놓을 자리 | 내용 |
|---|---|---|
| `vhand.cpp` | `vive/vhand.cpp` | 글러브 전용 마스터 (신규, **아직 여기서 작업 중**). |

`manus.h` 와 `vmaster.cpp` 는 **`vive/` 로 반영이 끝나서 여기엔 없습니다.**
`build.sh` 가 `-I../../../vive` 로 원본 `manus.h` 를 씁니다. 다시 여기서 고칠 일이
생기면 사본을 두면 그쪽이 자동으로 우선합니다(쌍따옴표 include 규칙).
| `build.sh` | `vive/build.sh` | 기존 것 + `vhand` 빌드 한 덩어리. 제자리 빌드용으로 include/`MANUS_DIR` 경로를 고쳤습니다 (아래 참고). |
| `hand-run` | **저장소 루트** (`kida/hand-run`) | 손만 띄우는 시뮬레이터 (신규). 여기서는 `run-hand` 래퍼로 실행합니다. |
| `run-hand` | (작업용, 반영 대상 아님) | `hand-run`을 저장소 루트 기준으로 돌려주는 래퍼. |
| `calib`, `logger` | (심볼릭 링크) | 원본 `vive/` 것을 가리킵니다. 프로필 폴더(`p1/`, `p2/`)는 `vive/calib/` 에 있습니다 — 캘리브레이션 데이터는 작업본이 아니라 저장소 본체에 두는 게 맞습니다. |

### 제자리 실행을 위해 손본 것

- **헤더** — `vhand.cpp`는 `manus.h`와 `retarget.h`를 씁니다. 둘 다 이 폴더에
  없으므로 `build.sh`가 `-I../../../vive`로 원본을 참조합니다. 쌍따옴표 include는 소스가 있는 디렉터리를 먼저 뒤지므로,
  `retarget.h` 계수를 이 폴더에서 만지고 싶으면 사본을 여기 두면 그쪽이
  자동으로 우선합니다 (원본은 그대로).
- **`MANUS_DIR`** — `../../../../../ManusSDK/3.0.1`. 원본 `vive/build.sh`의
  `../../` 은 저장소가 `~/kida`에 있고 `vive/` 가 루트 바로 아래라는 전제인데,
  이 폴더는 `devs/<이름>/` 아래라 두 단계 더 깊고 체크아웃도 `~/Desktop/kida`라
  한 단계가 더 붙습니다. `MANUS_DIR=...` 로 덮어쓸 수 있고, 없으면 바로
  실패합니다(조용히 엉뚱한 곳을 보지 않도록). rpath에는 절대경로를 박아서
  다른 디렉터리에서 띄워도 `.so`를 찾습니다.
- **`calib`, `logger`** — 원본 `vive/` 것으로 심볼릭 링크를 걸어 뒀습니다.
  `calib` 은 `manus.h` 가 **실행 파일 위치 기준**으로 찾으므로(`/proc/self/exe`),
  여기 링크가 있으면 어느 cwd 에서 띄워도 잡힙니다. `vmaster -l` 의 `./logger` 는
  아직 cwd 기준이라 그쪽은 이 폴더에서 띄워야 합니다.
- **`run-hand`** — `hand-run`은 `dg5`/`h9`를 import하고 `yaml/`을 상대경로로
  읽어서 저장소 루트에서 돌아야 합니다. 게다가 `uv run`은 cwd에서 위로 올라가며
  가장 가까운 `pyproject.toml`을 찾는데, 이 폴더 위에는 pytact가 없는
  `devs/taewon_choi_rccl/pyproject.toml`이 있어서 여기서 바로 실행하면 import부터
  실패합니다. `run-hand`가 루트로 `cd` + `PYTHONPATH`를 얹어 줍니다
  (cwd만 옮기면 `sys.path[0]`이 스크립트 디렉터리라 `dg5`를 못 찾습니다).

## 캘리브레이션 프로필 (`-pN`) — 기존 `-n` 대체

`-n`(캘리브 건너뛰기)이 없어지고 `-pN` 이 들어왔습니다. `calib/pN/{left,right}.mcal`
을 글러브에 올립니다. 사람별 표는 `calib/README.md` 에 있고, `manus.h` 상단 주석에도
같은 표를 달아 뒀습니다.

| | |
|---|---|
| `-p0` 또는 무플래그 | 아무것도 안 올림 (**예전 `-n` 이 이것**) |
| `-p1` | 이동혁 (RCCL) — 기존 `donghyuk*Metaglove.mcal` |
| `-p2` | 최태원 |
| `-p3~` | 폴더 만들면 잡힘 |

- `-n` 은 이제 인식되지 않습니다 → usage 띄우고 종료(조용한 동작 변경 방지).
- 없는 번호는 **바로 종료**합니다(`exit 64`). 업로드는 글러브가 붙은 뒤 백그라운드
  스레드에서 일어나므로, 인자 파싱 직후에 파일 존재만 미리 확인합니다. 이게 없으면
  `-p3` 오타가 조용히 "직전에 올린 프로필 그대로"로 흘러갑니다.
- **`p0` 은 초기화가 아닙니다.** `CoreSdk_SetGloveCalibration()` 이 글러브 하드웨어에
  쓰기 때문에, p0 은 *직전 실행에서 올린 값이 남아있는* 상태입니다. 그래서 기동 시
  LOG 패널에 어느 프로필로 도는지(p0 이면 경고를) 반드시 찍습니다.
- `calib` 은 **실행 파일 옆**에서 찾습니다(cwd 무관). `vive/vmaster` 는
  `vive/calib/`, 여기 `vhand` 는 `vive-working/calib`(→ `vive/calib`) 입니다.
  다른 곳에 두려면 `KIDA_CALIB_DIR=/path/to/calib`.

## 쓰는 법

```bash
./build.sh                      # vhand
./run-hand -g 2 -t 2 &          # 손만 있는 시뮬 (DG-5F-S 양손)
./vhand -g 2 -t 2 -p1           # 글러브 → retarget.h → 위 시뮬 (이동혁 프로필)
```

`retarget.h`를 고치면 `./build.sh` 후 `vhand`만 다시 띄우면 됩니다.
시뮬레이터는 계속 돌려둬도 됩니다.

## 원본에 반영

작업이 끝나면 이 폴더 기준으로:

```bash
cp vhand.cpp  ../../../vive/
cp hand-run   ../../../
```

`manus.h` / `vmaster.cpp` / `calib/` 는 이미 `vive/` 에 반영돼 있습니다.

주의할 것 셋:

- `build.sh` 는 제자리 빌드용 경로가 들어 있으므로 **그대로 복사하지 마세요**.
  `vive/build.sh` 에는 `vhand` 빌드 블록만 옮기면 됩니다.
- `vive/build.sh` 에 `vhand` 빌드 블록을 추가해야 합니다.

`vhand` 화면은 글러브 원본 `_q[20]`과 리타게팅된 `q`를 위아래로 나란히 보여줍니다.
계수를 바꾸면 손이 움직이기 전에 숫자로 먼저 확인됩니다. 키: `a` attach,
`h` home, `z` zero, `d` deg/rad 전환, `p` 현재 자세를 도(deg)로 로그에 덤프,
`q`/ESC 종료.

`-g`/`-t`는 양쪽이 같아야 합니다 — `-t`가 명령 문자열의 콤마 파트 수를 정하고
`hand-run`이 그 개수로 자르기 때문입니다.

`vhand -A`를 쓰면 맨 앞에 `none` 팔 파트를 붙여 보냅니다. 이건 `kida-run` /
`single-run`이 파싱하는 형식이라, 같은 글러브 스트림으로 팔은 그대로 둔 채 전체
로봇 시뮬레이터의 손만 움직여 볼 수 있습니다.

## 옵션

`vhand [-tN] [-gN] [-A] [-pN] [-e endpoint]`

| 옵션 | 의미 |
|---|---|
| `-t 0\|1\|2` | 왼손 / 오른손 / 양손 (기본 2) |
| `-g 0\|1\|2` | H9 / DG-5F-M / DG-5F-S (기본 2) |
| `-A` | 맨 앞에 `none` 팔 파트를 붙여 `kida-run`/`single-run`으로 보냄 |
| `-pN` | `calib/pN/` 프로필 업로드 (기본 0 = 안 올림). 위 절 참고 |
| `-e` | slave endpoint (기본 `ipc:///dev/shm/default`) |

`hand-run [-gN] [-tN] [-s M] [-v] [-l]` — `-g`/`-t`는 위와 같고, `-s`는 양손
배치 간격(m, 기본 0.25), `-l`은 렌더 창 없이 실행. `-x`/`-b`/`-d`는 없습니다:
실물 손 경로는 `eio-kida.so`가 포크하는 `eio-dg5f`/`eio-dg5s` 안에 있어서 팔
러너를 거쳐야 합니다.

`hand-run`은 `kida-run`과 같은 IPC 엔드포인트(`ipc:///dev/shm/default`,
`.../proprio`)를 bind하므로 둘을 동시에 띄울 수 없습니다. proprio는 팔 블록이
빠진 레이아웃입니다: DG-5F 손 하나당 float32 60개(pos/vel/current), H9는 18개
(pos/vel).

## 튜닝할 때 주의: `.mcal` 캘리브레이션

`calib/p1/{left,right}.mcal`(원본 `vive/calib/donghyuk*Metaglove.mcal`)은
**특정 사람 손 기준**의 Manus
글러브 캘리브레이션 프로파일입니다 (gzip으로 눌린 JSON — 손가락별 `proportions`,
`mcpPosition`, `fingerCurve`, `minMaxRange`, `sensorRotationOffset` 등).
`manus::LoadOneCalib`이 이 바이트를 그대로 `CoreSdk_SetGloveCalibration()`에
넣습니다. 코일 원시 신호를 `retarget.h`가 받는 관절각으로 바꾸는 게 이 파일이라,
**`retarget.h` 계수는 이 캘리브레이션에서만 유효합니다.** 착용자가 바뀌거나
재캘리브레이션하면 계수를 다시 봐야 합니다. 즉 **계수는 프로필과 짝**입니다 —
p1 계수를 p2로 켜고 만지면 헛수고입니다.

튜닝 중에는 `vhand`와 실운용 `vmaster`에 **같은 `-pN`을 주세요.** 다른 번호로
띄우면(특히 한쪽만 `-p0`) 서로 다른 입력에 계수를 맞추게 됩니다.

## 검증한 것

- `./build.sh`로 이 폴더에서 제자리 빌드 — `vhand` 가 `vive/manus.h` 를 참조해
  `-W -Wall` 경고 0으로 통과. `ldd`로 ManusSDK 링크 확인, 다른 디렉터리에서
  띄워도 `.so`를 찾는 것까지 확인.
- `vive/` 쪽도 `MANUS_DIR=~/ManusSDK/3.0.1 ./build.sh` 로 `vmaster` 재빌드 통과.
  `-p9` → `exit 64`, `-n` → usage 후 종료 확인.
- `./run-hand -l -g 2 -t 2` 정상 기동 (`import tact`/`dg5` 해결됨).
- `./calib/*.mcal` 이 cwd 기준으로 보이는 것 확인.
- `vhand` 실행 → Manus SDK 초기화, 동글 연결(`sdk:1`), SDK 로그가 LOG 패널로 들어감.
- `hand-run`을 이 저장소의 `dg5.py`/`yaml/`로 구동해 `home`, `joint`,
  `none` 접두 명령 모두 확인 (proprio 120 float32).
- 실제 글러브를 낀 상태의 리타게팅 결과는 확인하지 못했습니다 (글러브 미착용,
  `glove L:0 R:0`).
