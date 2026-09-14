# calib/ — 글러브 캘리브레이션 프로필

Manus 글러브의 캘리브레이션(`.mcal`)은 **한 사람의 손 기하·센서 오프셋**입니다.
여기 담긴 값이 글러브가 내보내는 관절각(`_q[20]`)의 의미를 정하고, `retarget.h`
계수는 그 관절각을 전제로 맞춰져 있습니다. 즉 **프로필을 바꾸면 리타게팅 계수도
같이 검토해야 합니다.** 남의 프로필로 켜 두고 계수를 만지면 헛수고가 됩니다.

## 번호 규칙

`vmaster` / `vhand` 의 `-pN` 이 이 폴더의 `pN/` 을 고릅니다.

| 번호 | 주인 | 비고 |
|---|---|---|
| `p0` | — | 업로드 안 함. `-p` 를 안 줬을 때의 기본값 |
| `p1` | 이동혁 (RCCL) | 기존 `donghyuk{Left,Right}Metaglove.mcal` |
| `p2` | 최태원 | 2026-09-08, SDKClient_Linux 로 생성 |
| `p3~` | (비어 있음) | 쓰고 싶은 사람이 폴더를 만들어 쓰는 자리 |

`donghyuk{Left,Right}Metaglove.mcal` 두 파일이 아직 여기 남아 있는데, **`p1/` 과
같은 내용의 사본**입니다. `vive/vmaster.cpp` 가 아직 옛 방식(이 파일명을 하드코딩,
`-n` 으로 건너뛰기)이라 지우면 그쪽이 깨집니다. `-pN` 을 `vive/` 에 반영하는
시점에 같이 지우면 됩니다. 새 코드(`devs/taewon_choi_rccl/vive-working/`)는
`p1/` 을 봅니다.

예전 `-n`(캘리브 건너뛰기)이 지금의 `p0`, 즉 **아무 플래그도 안 준 경우**입니다.
`-n` 은 없어졌으므로 그대로 쓰면 usage 를 띄우고 종료합니다.

이 폴더는 `vmaster` / `vhand` **실행 파일 옆**에서 찾습니다 (`/proc/self/exe` 기준,
cwd 와 무관). 다른 곳에 두려면 `KIDA_CALIB_DIR=/path/to/calib` 로 알려 주세요.

## p0 이 "초기화"가 아닙니다

`CoreSdk_SetGloveCalibration()` 은 값을 **글러브 하드웨어에 씁니다.** 그래서
`p0`(무플래그)은 깨끗한 상태가 아니라 **직전 실행에서 올린 프로필이 그대로 남아
있는 상태**입니다. `-p2` 로 한 번 띄웠다가 다음에 플래그 없이 띄우면 여전히 p2 가
들어 있습니다. 누구 손 기준인지 확실히 해야 할 때는 `-pN` 을 명시하세요.

## 프로필 추가

```
calib/p3/left.mcal      # Manus Core 에서 내보낸 왼손 프로필
calib/p3/right.mcal     # 오른손
```

파일명은 `left.mcal` / `right.mcal` 로 고정입니다. 폴더만 만들면 `-p3` 으로
잡히고, 위 표에 한 줄 적어 주세요. `manus.h` 상단 주석에도 같은 표가 있습니다.

### `.mcal` 은 어디서 만드나

Manus Core(캘리브레이션 마법사가 있는 정식 프로그램)는 Windows 전용입니다
(`~/Downloads/MANUS_Core_3.1.1_Version_Locked_Installer/ManusInstallerOffline.exe`).
하지만 **SDK 샘플 클라이언트로 리눅스에서 그대로 됩니다** — p2 는 그렇게 만들었습니다.

```bash
cd ~/Downloads/MANUS_Core_3.0.1_SDK/SDKClient_Linux
./build-integrated.sh          # 기본 Makefile은 안 됩니다. 아래 참고
./SDKClient_Linux.out
```

기본 `make` 가 막히는 두 곳(이미 `build-integrated.sh` 에 반영):

- `ClientLogging.hpp` 가 `<cstdint>` 를 include 하지 않아 GCC 13에서 컴파일 실패
  → `-include cstdint`
- 기본 `-lManusSDK` 는 protobuf 심볼을 외부에서 찾는데 시스템 버전과 안 맞음
  → protobuf 내장본 `-lManusSDK_Integrated` (vmaster 가 쓰는 그것)

**vmaster / vhand 는 꺼두세요.** 같은 동글을 잡습니다.

절차 (한 손씩, 기본은 왼손부터):

```
1                입력   -> Core Integrated
[C]                     캘리브레이션 메뉴. Glove: 가 0 이 아닌지 확인
[M] 여러 번             Step 0 으로 (번호는 0-based, [P]/[M] 은 수동)
[S]                     시작
[E] -> [P] -> [E] ...   스텝 실행하고 직접 번호 올리기 ([E] 는 자동으로 안 넘어감)
[F]                     완료 -> "Glove calibration finished"
[K]                     저장 -> ~/Documents/manus-calibrations/Calibration.mcal
[H]                     손 전환 후 반복
```

- `[E]` 는 블로킹입니다. 화면 `Time:` 초 동안 `Description` 동작을 계속 하세요
  (`Time` 이 음수면 연속 스텝 — SDK 가 충분하다고 볼 때까지).
- `IMU rotated too much` / `Fingers moved too much` 는 **그 스텝만 리셋**입니다.
  손등을 책상에 붙이고 손가락만 움직이면 대부분 해결됩니다.
- Manus 문구에서 `fingers` 는 **엄지를 뺀 네 손가락**입니다. 엄지가 필요한
  스텝은 `thumb` 이라고 따로 적혀 있습니다.
- 저장 파일명이 `Calibration.mcal` 로 고정이라 다음 `[K]` 가 덮어씁니다.
  한 손 저장할 때마다 이름을 바꾸세요.

### 확인

`mcalinfo.py` 로 내용이 실렸는지 봅니다 (`side` 로 좌/우도 확인됩니다).

```bash
./calib/mcalinfo.py calib/p2/*.mcal
```

손가락 5개가 다 나오고 `0 아닌 값` 이 300개 안팎이면 정상입니다.
`<-- 전부 0, 의심` 이 찍히면 그 손가락 스텝이 반영되지 않은 것입니다.

## 주의

`.mcal` 은 개인 손 치수가 담긴 파일입니다. 남의 프로필을 임의로 복사해 쓰지
마세요 — 리타게팅 결과가 조용히 어긋납니다.
