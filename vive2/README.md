# VIVE Ultimate Tracker: native pose receiver and KIDA 7-DOF GUI

세 프로그램 모두 HTC SDK 서버를 직접 시작하고 `ITrackerSubscriber::Update()`
콜백으로 연결된 트래커의 위치·자세를 받습니다.
별도 `tracker_demo`, 뷰어, Python 프로그램이나 원래 SDK 저장소는 필요 없습니다.

| 프로그램 | 용도 |
|---|---|
| `minimal` | 트래커별 최신 위치·자세를 터미널에 출력 |
| `gui-test` | 손목 트래커 2개로 KIDA 구조의 7자유도 양팔을 제어 |
| `vmaster2` | Ultimate Tracker + Manus로 기존 KIDA/GOS 제어기에 원격조작 명령 전송 |

`vmaster2`는 [vmaster2/README.md](vmaster2/README.md)의 빌드·실행 안내를 사용하세요.
`vive/vmaster.cpp`의 기능을 보존해 별도로 이식한 프로그램으로, 화면용 `gui-test`와 달리 제어기에 명령을 보냅니다.

## 폴더 구조

```text
vive2/
├── sdk/
│   ├── libultimate-tracker.so        # SDK 원본
│   └── ultimate_tracker.h           # SDK 원본
├── minimal/
│   ├── build.sh
│   ├── miniaml.cpp
│   ├── minimal                      # 빌드 결과
│   ├── libultimate-tracker.so -> ../sdk/libultimate-tracker.so
│   └── ultimate_tracker.h -> ../sdk/ultimate_tracker.h
├── gui-test/
│   ├── build.sh
│   ├── gui-test.cpp
│   ├── gui-test                         # 빌드 결과
│   ├── arm_ik.h / kida_ik.h / kida_model_data.h
│   ├── generate_kida_model.py
│   ├── test_arm_ik.cpp / test_kida_reference.py
│   ├── third_party/stb_easy_font.h
│   ├── libultimate-tracker.so -> ../sdk/libultimate-tracker.so
│   └── ultimate_tracker.h -> ../sdk/ultimate_tracker.h
├── vmaster2/                        # Ultimate + Manus 원격조작, 자체 build.sh
└── README.md
```

각 프로그램 폴더의 SDK 링크는 `../sdk/`를 가리키는 상대 심볼릭 링크입니다.
기존 SDK 실행 로그도 `sdk/`로 옮겼습니다. 새 로그는 실행 시 작업 디렉토리에 생성될 수 있습니다.
아래 명령은 `kida/vive2/`를 작업 디렉토리로 하는 기준입니다.

## 빌드와 실행

`minimal` 빌드에는 Bash와 C++17을 지원하는 `g++`가 필요합니다.
`gui-test`는 추가로 `pkg-config`, Python 3, GLFW/OpenGL 개발 라이브러리, Eigen 3 헤더가 필요합니다.
KIDA 모델 원본이 바뀌어 헤더를 재생성할 때에는 Python의 PyYAML도 필요합니다.
현재 PC에는 그래픽 의존성이 설치되어 있습니다. CMake나 make 없이 빌드합니다.
`sdk/`의 `ultimate_tracker.h`와 `libultimate-tracker.so`는 이 PC의
x86-64용 HTC SDK v0.0.1 헤더·라이브러리입니다.

```bash
./minimal/build.sh
./minimal/minimal

./gui-test/build.sh
./gui-test/gui-test
```

각 `build.sh`는 어느 작업 디렉토리에서 호출해도 해당 스크립트 폴더에 실행 파일을 만듭니다.
`minimal/build.sh`는 `minimal/minimal`만, `gui-test/build.sh`는 `gui-test/gui-test`만 빌드합니다.
다른 컴파일러는 `CXX=clang++ ./gui-test/build.sh`처럼 지정하세요.
각 폴더 안에서는 `./build.sh` 후 `./minimal` 또는 `./gui-test`로 실행하면 됩니다.

VIVE Wireless Dongle을 연결하고 이미 페어링한 트래커의 전원을 켜세요.
이 프로그램을 실행하기 전 기존 `tracker_demo` 등 동글을 사용하는 서버를 종료하세요.
종료는 **Ctrl+C** 또는 연결된 트래커 버튼을 빠르게 세 번 누르면 됩니다.
프로그램 종료 시 `StopTrackerServer()`를 호출하며, 본체 전원은 별도로 끕니다.

일정 시간만 실행하려면:

```bash
./minimal/minimal --seconds 10
```

## 양손목으로 양팔 제어

```bash
./gui-test/gui-test --left-id 0 --right-id 1 --height 1.75
```

`--height`는 배경 인체의 키이며 단위는 미터입니다. 기본값은 1.75입니다.
KIDA 팔의 실제 링크 길이와 양쪽 베이스 간격은 키와 무관하게 고정됩니다.
`--scale 0.7`처럼 지정하면 손목 이동 10cm가 목표점 이동 7cm가 됩니다. 기본값은 1로 1:1입니다.
회전에는 이동 스케일이 적용되지 않습니다.
다른 트래커 번호를 쓰면 `--left-id`와 `--right-id`를 바꾸세요.

1. 트래커를 왼손목과 오른손목에 단단히 부착하고 전원을 켭니다.
2. 화면에서 양쪽 상태가 `TRACKING`이 될 때까지 기다립니다.
   `Ready`여도 실제 포즈 상태가 `Good`가 아니면 보정과 제어를 허용하지 않습니다.
   `Good=2`, `Calibrating=3`은 SDK 뷰어/디버그의 이름 매핑을 기준으로 합니다.
3. 키보드 **1 / 2**로 왼쪽/오른쪽으로 지정된 트래커의 LED를 확인합니다.
   **L / R**로 사용할 ID를 순환하거나 **S**로 좌우를 바꿀 수 있습니다.
4. 정면을 보고 서서 화면의 대기 자세처럼 **팔꿈치를 굽혀 양손을 몸 앞에 둡니다**.
   양손목은 어깨 너비 정도로 벌리고 같은 높이로 유지합니다.
   **엄지는 위로, 손바닥은 서로 마주 보게** 하여 화면의 손 방향과 맞춥니다.
5. 그래픽 창을 선택하고 **C**를 눌러 보정합니다.
   양손목의 수평 방향으로 정면을 정하고, 현재 손목 위치·장착 각도를 각각 저장합니다.
   이 자세가 KIDA의 home TCP 자세에 대응합니다. 기존 T자세 보정에서 변경되었습니다.
6. 손목을 움직이거나 돌리면 양팔의 실제 7개 관절이 함께 움직이며 TCP 위치·방향을 맞춥니다.
   C를 다시 누르면 현재 손목 자세를 새 기준으로 삼아 KIDA home 자세로 돌아갑니다.

각 트래커 패널의 `POS`는 SDK 좌표계의 위치 X/Y/Z(미터), `ORI R/P/Y`는
같은 좌표계에서의 방향 roll/pitch/yaw(도)입니다. 방향은 쿼터니언을
extrinsic XYZ Euler 각도(`R = Rz(yaw) Ry(pitch) Rx(roll)`)로 변환해 표시합니다.
실제 입력 모드에서는 보정·스무딩 전의 트래커 값이며, 데모에서는 합성 입력의 화면상 값을 표시합니다.

몸통·어깨·머리·다리는 고정되어 있으므로 제어 중에는 몸통의 위치와 방향을 유지하세요.
수평 손목 간 거리가 20cm 미만이거나 양손 높이 차이가 10cm를 넘으면 보정을 거부합니다.
보정은 실행 중 메모리에 보관되며, 프로그램을 다시 실행하면 C로 다시 보정합니다.
서버/동글을 사용하는 두 프로그램을 동시에 실행하지 마세요.

| 입력 | 동작 |
|---|---|
| C | 팔꿈치를 굽힌 대기 자세로 보정, KIDA home으로 복귀 |
| L / R | 왼손/오른손에 사용할 트래커 번호 순환, 보정 초기화 |
| S | 좌우 트래커 교환, 보정 초기화 |
| 1 / 2 | 왼손/오른손 트래커 LED 식별 |
| A | 실제 관절 회전축과 TCP 좌표축 표시 전환 |
| 마우스 왼쪽 드래그 / 휠 | 카메라 회전 / 확대·축소 |
| Home | 카메라 초기화 |
| Esc / 창 닫기 / Ctrl+C | 자체 서버와 프로그램 종료 |

팔은 `kida/yaml/kida.yaml`의 좌우 `link1..7` / `link8..14`, `tcp1` / `tcp2`를 사용합니다.
관절 위치·축 방향·링크 길이·TCP 오프셋·원통 형상을 그대로 읽어 C++ 헤더에 저장합니다.
각 관절은 YAML 고정 변환 뒤의 local Z축을 중심으로 회전하며 좌우 각각 독립된 7자유도입니다.
양쪽 첫 관절의 간격은 292mm이며, 모델 전체를 배경 인체의 어깨 높이에 평행 이동해 표시합니다.
손 모양과 몸통은 시각화용이며 KIDA의 실제 그리퍼 모델은 포함하지 않습니다.
손 형상의 방향은 `kida-run`의 좌우 손 장착 회전(intrinsic XYZ `[0,90,-90]` / `[0,90,90]`)과
손 YAML 좌표계에 맞춥니다. 모든 관절이 0이면 손가락은 아래, 손등은 좌우 바깥쪽을 향합니다.
시작 화면과 C로 돌아가는 home 자세는 모든 관절이 0인 자세와 다릅니다.
USB를 사용하지 않고 영점 자세만 확인하려면 `./gui-test/gui-test --zero-pose`를 실행하세요.

`gui-test/generate_kida_model.py`가 `kida/yaml/kida.yaml`과 `kida/kida.py`를 읽어
`gui-test/kida_model_data.h`를 생성합니다.
빌드할 때 원본이 변경되었으면 재생성합니다. 실행할 때는 YAML/Python/pytact가 필요 없습니다.
`vive2/` 전체를 옮겨도 상대 SDK 링크와 포함된 모델 헤더로 다시 빌드할 수 있습니다.
관절 제한은 `kida.py`의 `joint` 명령 clamp 값을 사용합니다. YAML에는 기계적 제한이 명시되어 있지 않습니다.
J1/J2/J3/J5/J6/J7은 -1.5~1.5rad, J4는 -1.0~2.5rad입니다.
초기 자세는 `init2`에서 시작해 `home_task`의 TCP 위치·방향을 수치 IK로 맞춰 구합니다.

손목 위치 3축과 방향 3축을 함께 목표로 두고, 감쇠 최소제곱 IK와 관절 제한을 적용합니다.
이전 프레임의 관절각을 다음 풀이의 시작점으로 사용하며 남는 자유도는 home 자세 쪽으로 유도합니다.
손목 트래커만으로 사람의 팔꿈치 방향까지 측정할 수는 없습니다.
추가 트래커나 팔꿈치 방향 입력 없이 사람 팔꿈치의 모든 자세를 그대로 복제하는 것은 아닙니다.

각 패널의 `Q1-4`, `Q5-7`은 해당 팔의 실제 관절각(도)이며 오른팔은 YAML의 link8..14 순서입니다.
`IK ERROR`는 목표와 실제 TCP의 위치 오차(mm)와 방향 오차(도)입니다.
오차가 8mm 또는 약 4.6도를 넘으면 `IK LIMIT / TARGET NOT REACHED`와 주황색 목표점을 표시합니다.
관절 제한·도달 범위·국소해 때문에 목표를 맞추지 못할 수 있으며 충돌 검사/회피는 포함하지 않습니다.
이 프로그램은 화면의 가상 모델만 제어하며 실제 KIDA에 명령을 보내지 않습니다.

추적이 끊기거나 마지막 데이터가 0.3초보다 오래되면 해당 팔의 마지막 목표를 유지합니다.
35ms 시정수의 스무딩을 적용하며, 렌더링은 화면 동기화 주기로 동작합니다.
트래커 데이터 수신은 렌더링과 독립된 SDK 콜백에서 계속 이루어집니다.

현재 화면은 별도 모델 다운로드가 필요 없는 절차적 인체 형상입니다.
AMASS 영상과 동일한 SMPL/SMPL-H 메시를 사용하려면 해당 인체 모델 파일과 스키닝 구현이 추가로 필요합니다.
이번 버전은 양팔 제어만 구현하며, 발목·허리를 추가한 5개 트래커 전신 제어는 후속 작업입니다.

### 트래커 없는 데모와 테스트

```bash
./gui-test/gui-test --demo
./gui-test/gui-test --zero-pose
./gui-test/build.sh --test
```

`--demo`는 USB 서버를 시작하지 않고 KIDA의 도달 가능한 합성 TCP 위치·자세로 양팔을 움직입니다.
테스트는 7개 관절의 Jacobian 수치 미분 대조, 240개 6D IK 목표, 2,000프레임 연속 추종,
관절 제한, 도달 불가능 목표, 보정 좌표·장착 방향 변환을 확인합니다.
설치된 pytact와 실제 YAML을 사용한 독립 FK 대조도 실행할 수 있습니다:

```bash
../.venv/bin/python gui-test/test_kida_reference.py
```

이 검사는 무작위 양팔 자세 100개의 TCP 변환행렬과 손 형상의 장착 방향을 비교합니다.
손 방향은 `kida-run`에서 실제 장착 회전을 읽어 독립적으로 대조하며 영점의 손등 방향도 확인합니다.
제한 시간 실행과 테스트용 PPM 화면 저장도 가능합니다.

```bash
./gui-test/gui-test --demo --seconds 3 --screenshot /tmp/gui-test.ppm
```

## 출력과 데이터 처리

샘플은 아래 CSV 형식으로 출력합니다. HTC SDK의 시작·상태 로그도 같은 터미널에
표시되므로 전체 표준 출력을 그대로 저장하면 순수 CSV 파일은 아닙니다.

```text
id,serial,timestamp_us,state,pose_state,rotation_only,setup_required,x_m,y_m,z_m,qx,qy,qz,qw
```

- 위치: 미터. X 전방, Y 왼쪽, Z 위쪽.
- 자세: quaternion `(qx, qy, qz, qw)`.
- `timestamp_us`: SDK 기준 마이크로초 시각. Unix epoch 시각은 아닙니다.
- `pose_state`: `Flags & TrackerFlag_PoseStateMask` 원시값. 공개 헤더에는 각 값의 이름이 정의되어 있지 않습니다.
- `state=Ready`만으로 정상 추적을 보장하지 않습니다. 추적 상태와 위치 갱신도 확인하세요.
- `rotation_only=1`이면 위치가 갱신되지 않는 상태입니다.
- `setup_required=1`이면 맵 설정이 필요합니다.

SDK 콜백에서는 최신 샘플을 복사하기만 하고, 주 스레드에서 트래커별 최신값을
초당 최대 20회 출력합니다. 콜백 수신 주기를 20 Hz로 제한하는 것은 아닙니다.
중간 샘플을 모두 기록하는 프로그램은 아니며, 종료 시 실제 수신 콜백 수를 표시합니다.
사용자 로직은 `minimal/miniaml.cpp`의 주 스레드 루프에서 `sample.pose`를 처리하면 됩니다.

두 프로그램 모두 페어링·맵 생성 UI는 포함하지 않습니다.
이미 저장된 맵으로 추적을 시작하고, 필요하면 HTC 설정 프로그램으로 맵을 만드세요.

## 다른 폴더에서 실행

`vive2/` 전체를 옮기면 `sdk/`, `minimal/`, `gui-test/` 사이의 상대 심볼릭 링크가 유지됩니다.
새 위치에서도 각 프로그램의 `build.sh`로 다시 빌드하고 실행할 수 있습니다.

실행 파일 하나만 별도로 배포할 때에는 해당 실행 파일과 SDK 라이브러리 **실제 파일**을
같은 폴더에 복사하세요. SDK 심볼릭 링크만 복사하면 대상 `sdk/`도 있어야 합니다.

```text
별도 실행 폴더/
├── gui-test 또는 minimal
└── libultimate-tracker.so           # 실제 파일
```

실행 파일은 `$ORIGIN`을 사용해 자기 폴더의 라이브러리를 찾습니다.
현재 PC에 설치된 표준 C/C++ 런타임과 USB 장치 접근 권한은 필요합니다.
`gui-test`는 추가로 GLFW/OpenGL 런타임과 그래픽 세션이 필요합니다.
화면 글꼴은 `gui-test/third_party/stb_easy_font.h`를 사용하며, 원본 라이선스는 그 파일에 포함되어 있습니다.
