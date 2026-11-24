# Ultimate Tracker + Manus vmaster2

`kida/vive/vmaster.cpp`의 원격조작 기능을 VIVE Ultimate Tracker SDK로 옮긴 프로그램입니다.
원본 `kida/vive/`는 수정하지 않았습니다. SteamVR, OpenVR, 베이스 스테이션 없이 동작합니다.
이 프로그램은 `gui-test` 뷰어와 달리 기존 KIDA 제어기에 실제 원격조작 명령을 전송합니다.

## 보존한 기능과 파일

- `-t0` 왼팔, `-t1` 오른팔, `-t2` 양팔, `-t5` GOS10 모드
- Manus SDK 직접 수신, 기존 보정 파일, H9 / DG5F-M / DG5F-S / H12 손가락 매핑
- 팔·손 연결/해제, 홈·초기화·휴식, 로깅, jaw 조작과 ncurses UI
- 약 30Hz ZMQ PUSH, 기존 ASCII `task` / `joint` / `none` 형식과 제어 명령
- 데이터 소켓은 최신 프레임 유지(CONFLATE), 제어 소켓은 별도 큐로 순서대로 전송

복사한 파일: `vmaster2.cpp`, `retarget.h`, `logger`, `player`, `calib/*.mcal`.
`vmaster2.cpp`의 트래커 입력·좌표 변환과 필요한 주변 코드를 이 폴더에서 수정했습니다.
`logger` 사본에는 테스트/원격 구성용 `--command-endpoint` 옵션만 추가했습니다.
`retarget.h`, `player`, 보정 파일은 원본과 같습니다.

`ultimate_input.h`가 SDK 콜백 수신·유효성 검사·트래커 선택을 담당합니다.
`ultimate_tracker.h`, `libultimate-tracker.so`는 `../sdk/`를 가리키는 상대 심볼릭 링크입니다.
`vive/`의 헤더나 OpenVR 파일을 런타임에 참조하지 않습니다.

## 빌드

```bash
cd /home/ubuntu/kida/vive2/vmaster2
./build.sh
```

필요 항목: C++17 컴파일러, Bash, pkg-config, libzmq/ncurses 개발 패키지,
Ultimate SDK와 Manus SDK 3.1.1. 현재 PC에는 모두 설치되어 있습니다.
기본 Manus SDK 경로는 `../../../ManusSDK/3.1.1`입니다. 다른 경로는 다음처럼 지정합니다.

```bash
MANUS_SDK_DIR=/path/to/ManusSDK/3.1.1 ./build.sh
```

빌드는 호출한 작업 디렉토리와 무관하게 이 폴더에 `vmaster2`를 만듭니다.
Ultimate SDK는 실행 파일 옆 링크로 찾고, Manus 라이브러리 경로는 빌드할 때 RPATH에 기록합니다.
`-g`를 생략한 팔 전용 모드에서도 이 빌드의 실행 파일에는 Manus 공유 라이브러리가 필요합니다.
이 모드에서는 Manus 장치를 초기화하거나 손 명령을 보내지 않습니다.

## 실행

다른 Ultimate SDK 서버(`gui-test`, `minimal`, `tracker_demo` 등)를 종료하고 동글을 연결합니다.
이미 페어링·맵 설정이 된 트래커를 사용합니다. 이 프로그램에는 맵 설정 UI가 없습니다.

팔 두 개만 제어:

```bash
./vmaster2 -t2 --left-id 0 --right-id 1
```

Manus와 DG5F-S 손까지 제어:

```bash
./vmaster2 -t2 -g2 --left-id 0 --right-id 1
```

기존과 같이 보정 생략은 `-n`, logger 자동 실행은 `-l`입니다.
KIDA logger는 손 종류가 필요하므로 `-l`을 쓸 때에는 원본처럼 `-g`도 지정합니다.

```bash
./vmaster2 -t2 -g2 -n -l --left-id 0 --right-id 1
```

오른팔 하나에 SDK ID 0을 사용하는 예:

```bash
./vmaster2 -t1 --right-id 0
```

`-t5`는 `--left-id` 또는 `--left-serial` 쪽 트래커를 사용합니다.
원본처럼 KIDA 이동 스케일은 1, GOS10은 2입니다.

기본 좌우 ID는 0/1입니다. 트래커가 3개 이상 연결되어 있어도 지정한 트래커만 사용합니다.
시리얼로 지정할 수도 있습니다. 같은 쪽에 ID와 시리얼을 함께 지정하지 마세요.

```bash
./vmaster2 -t2 --left-serial YOUR_LEFT_SERIAL --right-serial YOUR_RIGHT_SERIAL
```

ID로 선택해도 처음 연결된 기기의 시리얼을 고정합니다. 재연결 후 다른 ID로 나타나면
같은 시리얼을 찾아 연결하며, 다른 기기가 기존 슬롯을 차지해도 팔을 넘겨주지 않습니다.
`1`/`2` 키로 지정한 좌/우 트래커의 LED를 확인할 수 있습니다.

프로그램은 팔과 손이 모두 연결 해제된 상태로 시작합니다.
필요한 트래커가 모두 `TRACKING`이면 `a`를 눌러 팔을 연결하고, `b`를 눌러 손을 연결합니다.
첫 팔 연결은 KIDA home TCP `(0.30, ±0.23, -0.40)`, 방향 0/0/0을 기준으로 합니다.
제어기는 그 자세에 있다고 가정하며 실제 로봇 상태를 구독해 맞추는 기능은 원본처럼 없습니다.

## 좌표계와 연결 기준

Ultimate SDK는 이미 X 전방 / Y 왼쪽 / Z 위쪽 좌표계입니다.
원본의 OpenVR Y-up 변환과 Tracker 3.0 장착 회전을 그대로 적용하지 않습니다.

```text
T_map_to_robot = Rz(yaw)
T_current = T_map_to_robot * T_ultimate
offset = -position_at_attach
R_offset = transpose(R_at_attach) * R_last_output
position_out = scale * (position_current + offset) + position_last_output
R_out = R_current * R_offset
```

`a` 연결 시 현재 손목의 위치와 장착 방향을 기준으로 잡고, 이후 이동·회전을 반영합니다.
트래커를 특정 RPY로 맞추거나 `gui-test`의 C 보정을 할 필요는 없습니다.
위치는 미터, 전송되는 RPY는 **라디안**, extrinsic XYZ입니다.

맵의 수평 축과 사용자가 바라보는 정면이 다르면 `--yaw-deg`로 SDK→로봇 회전을 지정하세요.
예를 들어 90도는 SDK +X 이동을 로봇 +Y 이동으로 변환합니다.
이 값은 양손목/몸통에서 자동 추정하지 않습니다.

```bash
./vmaster2 -t2 --left-id 0 --right-id 1 --yaw-deg 90
```

원본의 `-b1`/`-b2`는 CLI 호환을 위해 유지했습니다. Ultimate에는 베이스 스테이션이 없으므로
각각 yaw 0도 / +30도 프리셋으로 해석하며, `--yaw-deg`가 있으면 그 값이 우선합니다.

`Ready`와 실제 추적 가능 여부를 구분합니다. `Ready` + pose-state Good(2),
유효한 위치/쿼터니언, rotation-only/setup-required 해제, 0.3초 이내 수신을 모두 요구합니다.
pose-state 이름은 기존 Ultimate SDK 뷰어/디버그의 매핑을 기준으로 합니다.
필요한 트래커 하나라도 추적을 잃거나 재위치 추정/재연결이 감지되면 팔을 자동 해제합니다.
다시 `TRACKING`이 된 뒤 사용자가 `a`로 재연결합니다. 손 연결은 독립적으로 유지합니다.
해제는 팔 목표 전송을 멈추는 동작이며 로봇 제어기에는 마지막 명령이 남습니다.

## 키와 통신

| 키 | 기능 |
|---|---|
| a | 팔 연결/해제; 현재 위치·방향을 기준으로 캡처 |
| b | Manus 손 연결/해제 |
| h | `home` 전송, 기준 위치·방향을 home으로 초기화 |
| i | `init, home[, home]` 전송, 팔·손 해제 |
| r | `rest, home[, home]` 전송, 팔·손 해제 |
| / | `randomize` 전송 |
| c | logger `log-on` / `log-off` |
| z / x | GOS10 jaw 0.10 / 0.02 |
| 1 / 2 | 좌/우 트래커 LED 식별 |
| q | slave와 logger에 `quit` 전송 후 master 종료 |
| Esc | logger 종료 후 master 종료; slave에는 `quit`을 보내지 않음 |
| Ctrl+C / SIGTERM / 트래커 버튼 세 번 | master와 자체 SDK 서버 종료; 자동 실행한 logger도 종료 요청 |

기본 전송 대상은 원본과 같습니다.

```text
slave : ipc:///dev/shm/default
logger: ipc:///dev/shm/logger
```

수신 제어기가 bind하고 master가 connect합니다. 필요하면 URI를 지정할 수 있습니다.
예를 들어 기존 `kida-run`의 원격 명령 포트는 `--slave-endpoint tcp://ROBOT_IP:5555`로 연결합니다.
`--logger-endpoint`도 변경 가능하며 `-l`로 띄운 logger 사본에 같은 값을 전달합니다.
logger의 proprio/카메라 구독 주소와 HDF5 구조는 원본 그대로입니다.
logger·보정 파일은 실행 파일 폴더에서 찾습니다. HDF5와 SDK 로그의 출력 위치는 작업 디렉토리입니다.
KIDA `.venv/bin/python`이 있으면 logger에 그 인터프리터를 사용하고, 없으면 원래 shebang을 사용합니다.
`player`는 복사된 HDF5 조회/재생 도구입니다.

## 명령 전송 없이 입력 확인

```bash
./vmaster2 -t2 --left-id 0 --right-id 1 --dry-run
```

`--dry-run`에서는 SDK 입력과 UI/키 조작을 유지하되 ZMQ 소켓 생성·전송을 생략합니다.
전송할 명령은 로그 패널에 표시합니다. 이 모드에서는 `-l`로 logger를 실행하지 않습니다.
`--seconds 10`처럼 종료 시간을 지정할 수도 있습니다.
ncurses용 실제 터미널이 필요하며 상태/로그가 충분히 보이도록 터미널을 넓혀 사용하세요.

## 검증

```bash
./build.sh --test
```

테스트는 SDK 서버나 Manus 장치를 시작하지 않습니다.
합성 입력으로 추적 상태·시리얼 고정·재위치 추정, 위치/쿼터니언 변환,
연결 시 연속성, 80가지 명령 조합과 네 종류의 손 매핑을 확인합니다.
ZMQ 테스트는 전용 `inproc://` 주소를 사용하므로 실제 slave/logger와 연결하지 않습니다.
실제 로봇·Manus 장치를 사용한 종단 검증은 별도로 필요합니다.
