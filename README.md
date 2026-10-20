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

## 구성과 검증

- `team_mvp/engine.py`: 등록·설치·수집·제어 세션, Heartbeat/상태 순번, 재전송, aim/stop.
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
