# VIVE Ultimate Tracker: minimal native C++ example

이 프로그램이 HTC SDK 서버를 직접 시작하고 `ITrackerSubscriber::Update()`
콜백으로 모든 연결된 트래커의 위치·자세를 받습니다.
별도 `tracker_demo`, 뷰어, Python 프로그램이나 원래 SDK 저장소는 필요 없습니다.

## 빌드와 실행

필요한 도구: Bash, C++17을 지원하는 `g++`. CMake나 make 없이 빌드할 수 있습니다.
프로젝트 루트의 `ultimate_tracker.h`와 `libultimate-tracker.so`는 이 PC의
x86-64용 HTC SDK v0.0.1 헤더·라이브러리입니다.

```bash
./build.sh
./minimal
```

`build.sh`는 어느 작업 디렉토리에서 호출해도 프로젝트 루트에 실행 파일 `minimal`을
만듭니다. 다른 컴파일러는 `CXX=clang++ ./build.sh`처럼 지정하세요.

VIVE Wireless Dongle을 연결하고 이미 페어링한 트래커의 전원을 켜세요.
이 프로그램을 실행하기 전 기존 `tracker_demo` 등 동글을 사용하는 서버를 종료하세요.
종료는 **Ctrl+C** 또는 연결된 트래커 버튼을 빠르게 세 번 누르면 됩니다.
프로그램 종료 시 `StopTrackerServer()`를 호출하며, 본체 전원은 별도로 끕니다.

일정 시간만 실행하려면:

```bash
./minimal --seconds 10
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
사용자 로직은 `miniaml.cpp`의 주 스레드 루프에서 `sample.pose`를 처리하면 됩니다.

이 예제는 포즈 수신에 집중하므로 페어링·맵 생성 UI는 포함하지 않습니다.
이미 저장된 맵으로 추적을 시작하고, 필요하면 HTC 설정 프로그램으로 맵을 만드세요.

## 다른 폴더에서 실행

빌드된 아래 두 파일을 같은 폴더에 놓으면 됩니다.

```text
minimal
libultimate-tracker.so
```

실행 파일은 `$ORIGIN`을 사용해 자기 폴더의 라이브러리를 찾습니다.
폴더를 옮긴 뒤에도 `./build.sh`로 다시 빌드할 수 있습니다.
현재 PC에 설치된 표준 C/C++ 런타임과 USB 장치 접근 권한은 필요합니다.
