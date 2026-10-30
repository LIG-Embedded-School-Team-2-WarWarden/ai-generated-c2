# AI Generated C2 — C++ 콘솔 MVP

## 최신 GUI v2 연동

`c2-GUI@fe22d07`의 SharedMemory v2(484바이트), Named Pipe v2(28/40바이트)에
연결하는 C++ 어댑터를 추가했습니다. `c2_mvp_console.exe --gui`로 실행합니다.
RDR/FCS 위치, 방위각(135도=정면), 레이저 전원 상태와 RDR 고각 설정을 반영합니다.
[규격·실행·검증 방법](docs/gui-v2.md)을 참고하세요.

교육·시연용 팬틸트 포인터를 위한 콘솔 통제소이며, `--gui` 옵션으로 WPF GUI v2에 연결합니다.
**콘솔·통신·점군 조립·관측 검증·지향·정지 로직은 C++20으로 실행합니다. Python은 더미 장치와 블랙박스 테스트 전용입니다.**
OBS/EFF firmware·LiDAR SDK 등 다른 담당 파트는 이번 이전에서 수정하지 않았습니다.

## 실행

CMake 3.24+와 C++20 컴파일러가 필요합니다. Protobuf와 nlohmann/json이 없으면 CMake가 고정 버전을 다운로드합니다.
Windows에서는 Visual Studio C++ 빌드 도구를 사용합니다.

```powershell
./scripts/build.ps1 -BuildDir build/native
./build/native/Release/c2_mvp_console.exe
```

C++ 실행 파일은 Python을 설치하지 않아도 실행됩니다. Python 검색·더미 시험 자체를 제외하려면 CMake에 `-DC2_BUILD_DUMMY_TESTS=OFF`를 지정합니다. CTest의 더미 시험까지 실행하려면 먼저 `./scripts/setup-team.ps1`로 시험 의존성을 설치합니다.
다른 터미널 두 개에서 선택적으로 더미를 실행합니다.

```powershell
./.venv-team/Scripts/python.exe scripts/team_dummy.py OBS
./.venv-team/Scripts/python.exe scripts/team_dummy.py EFF
```

```text
status
targets
point 201 20 10
stop 201
aim 201
status
stop 201
obs point 10 5
obs status
obs point 0 0
obs status
quit
```

각 이동·정지는 `status`에서 완료를 확인하고 다음 명령을 입력합니다.
`aim`은 최신 단일 물체를 연속 지향하고, `stop`은 결과와 후속 실측 정지를 확인합니다.
OBS 비영점 관측은 DISPLAY_ONLY이며 자동 aim은 검증된 실측 0도 고정 관측만 허용합니다.
교육용 저출력 레이저는 `point`/`aim`으로 제어 세션을 연 뒤 `laser on 201 1000`,
`laser off 201`, `laser status 201`로 조작합니다. ON은 실측 정지 상태에서 1~5000ms만 허용하며,
지원 기능을 등록·보고한 EFF에만 전송합니다. `status`의 `lasers`에 장치 확인 상태를 표시합니다.
STOP·DRAIN·세션 교체/상실·시한 만료에는 EFF가 소등합니다. [레이저 운용 안내](docs/laser-control.md)를 참고하세요.

Linux에서도 같은 C++ 실행 파일을 사용합니다.

```sh
cmake -S . -B build/native -DC2_BUILD_LIVE=OFF
cmake --build build/native --parallel 2
./build/native/c2_mvp_console --config configs/team-demo.json
ctest --test-dir build/native --output-on-failure
```

공용 ICD 1.0 `.proto`를 submodule의 고정 커밋에서 읽어 C++ 코드를 생성합니다.
새로운 사설 wire 형식이나 Python 중계 프로세스는 없습니다.
검출은 `vendor/lidar/target_detector.h`를 C++에서 직접 호출합니다.

## 구조와 검증

- `native/console.cpp`: 실제 C++ CLI, 수신 루프와 종료 처리.
- `native/engine.cpp`: 서비스 조립·수신 분배·CLI Facade.
- `native/devices.cpp`, `requests.cpp`: 장치 세션·순번·명령 재전송·완료 처리.
- `native/effector.cpp`, `observer.cpp`, `states.cpp`: 지향·정지와 OBS 상태 전이.
- `native/observation.cpp`: 원본 점군 조립, 최신 작업 큐, 기존 검출기 Adapter, 관측 유효성.
- `native/transport.cpp`, `pointing.cpp`: UDP Adapter와 교체 가능한 계산 Strategy.
- `native/include/c2mvp/domain.hpp`: 형식화된 모델과 가상 인터페이스.
- `tests/native_domain.cpp`, `native_behaviors.cpp`: 실제 시간 대기·소켓 없는 C++ 단위 시험.
- `tests/test_native_mvp.py`, `tests/dummies/`: C++ 프로세스에 명령을 넣고 UDP로 검증하는 더미 시험.

```powershell
./.venv-team/Scripts/python.exe tests/test_native_mvp.py --console build/native/Release/c2_mvp_console.exe
./scripts/package.ps1 -BinDir build/native/Release
```

[콘솔 시연·ICD](docs/console-mvp.md), [객체지향 설계·패턴](docs/software-design.md),
[TDD와 요구사항 추적](docs/development-process.md)을 참고하세요.
기본 구성은 localhost 합성 시험입니다. 실제 보드 검증 결과를 의미하지 않습니다.
이전 v3 서버·대시보드는 [legacy 안내](docs/legacy-runtime.md)에 분리돼 있으며 `C2_BUILD_LIVE=ON`으로 빌드합니다.
