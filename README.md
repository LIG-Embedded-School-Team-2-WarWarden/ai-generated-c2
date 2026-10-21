# AI Generated C2 — 콘솔 MVP

교육·시연용 팬틸트 포인터를 위한 GUI 없는 통제소입니다.
OBS 원본 점군을 수신·조립하고 기존 LiDAR 저장소의 C++ Detector로 물체를 검출합니다.
`aim 201`은 최신 단일 물체를 계속 지향하며 `stop 201`로 종료합니다.
레이저 점등·출력 제어 기능은 포함하지 않습니다.

통신은 공용 저장소의 **ICD 1.0 Protobuf 생성 코드**를 사용합니다.
기존 로컬 raw 확장(field 100)과 별도 raw 송신기는 제거했습니다.
공용 proto는 [공용 저장소](https://github.com/LIG-Embedded-School-Team-2-WarWarden/protobuf)에 게시되어 있으며, 통제소는 submodule로 검증한 커밋을 고정합니다.

## 실행

Python 3.10+와 Visual Studio C++20 빌드 도구가 필요합니다. 저장소 루트에서:

```powershell
./scripts/setup-team.ps1
./scripts/build.ps1
./build/live/Release/c2_mvp_console.exe
```

다른 터미널 두 개에서:

```powershell
./.venv-team/Scripts/python.exe scripts/team_dummy.py OBS
./.venv-team/Scripts/python.exe scripts/team_dummy.py EFF
```

콘솔에서:

```text
status
targets
aim 201
status
stop 201
quit
```

기본 구성은 **localhost 전용 합성 데이터 시험**입니다. 실제 센서 취득시각·장착 축·모터 피드백이 검증됐다는 뜻이 아닙니다.
[실행 방법과 장비 연결 조건](docs/console-mvp.md), [ICD 변경 대응표](docs/team-icd-migration.md),
[공용 스키마·생성 방법](external/warwarden-protocol/TEAM_ICD.md)을 참고하세요.

## OBS CLI 조작

Python 더미와 native firmware `mfs_obs`/`mfs_eff` 모두 같은 명령을 사용합니다.

```text
obs point 10 5
obs status
obs point 0 0
obs status
targets
aim 201
stop 201
obs stop
```

`obs point`는 **EFF 지향 정지 확인 → 수집 중지 확인 → OBS 이동 → 실측 정지 확인 → 새 세션으로 수집**을
순서대로 진행합니다. `obs status`의 operation.state가 COMPLETED인지 확인합니다.
`obs stop`은 진행 중인 OBS 이동을 취소하고 정지·수집 중지를 요청합니다.
이동 후 EFF aim은 자동으로 다시 시작하지 않습니다.
현재 자동 aim은 OBS 실측 pan/tilt=0 고정 관측만 허용하므로, 다른 OBS 각도의 결과는 DISPLAY_ONLY입니다.

실물의 검증된 시간·노출 설정은 firmware의 `--acquisition-profile`과 C2 콘솔의
`clock-profile 101 profile.json`에 같은 파일을 사용합니다. 실제 값·증빙·유효기간이 필요하며,
설정 파일을 읽었다는 이유만으로 동기화나 자세를 임의로 인정하지 않습니다.

## 구성과 검증

- `team_mvp/engine.py`: CLI Facade와 서비스 조립·수신 분배·종료.
- `team_mvp/devices.py`, `requests.py`: 장치 세션/최신성과 명령 재시도/완료 상관관계.
- `team_mvp/observer.py`, `effector.py`: 명시적 OBS 상태 전이와 EFF 지향/STOP 세션.
- `team_mvp/observation.py`: 취득 시각·자세·관측 승인 정책.
- `team_mvp/clock.py`, `transport.py`, `detection.py`, `pointing.py`: 교체 가능한 시간·전송·검출·지향 계산 포트/어댑터.
- `team_mvp/cloud.py`: 원본 전체 슬롯 조립, 최신 프레임 교체, C++ 분석 작업 분리.
- `team_mvp/geometry.py`: 설치값 반영과 상대 벡터의 atan2 계산.
- `src/detector_worker.cpp`, `vendor/lidar/`: 기존 Detector 재사용. 내부 stdin 전달은 장치 통신 규격이 아닙니다.
- `external/warwarden-protocol/proto/mfs/icd/v1/mfs.proto`: 공용 장치 메시지 정의.
- `tests/test_team_mvp.py`: 실제 UDP·생성 Protobuf·C++ 검출기 및 콘솔 실행 파일 시험.

```powershell
./.venv-team/Scripts/python.exe tests/test_team_mvp.py --detector build/live/Release/c2_detector_worker.exe
./scripts/package.ps1
```

포장은 별도의 `dist/team-mvp-*` 폴더와 zip으로 생성됩니다. 가상환경을 복사하지 않으므로 추출 후 setup-team.ps1을 실행합니다.

이전 v3 서버·대시보드 코드는 [legacy 안내](docs/legacy-runtime.md)로 분리했습니다.
필요한 경우 `-DC2_BUILD_LIVE=ON`으로 빌드할 수 있지만 ICD 1.0 콘솔/더미와 호환되지 않습니다.
CI는 새 콘솔 시험과 Windows legacy 회귀 시험을 모두 수행하도록 구성했습니다.

## 설계와 개발 근거

[객체지향 구조·패턴과 실제 실행 경로](docs/software-design.md),
[요구사항·TDD 커밋·테스트 추적](docs/development-process.md)을 제공합니다.
새 도메인 시험은 `python tests/test_design.py`로 네트워크·스레드·실제 sleep 없이 실행합니다.
기존 프로젝트 전체가 TDD로 작성됐다고 소급해 설명하지 않습니다.
