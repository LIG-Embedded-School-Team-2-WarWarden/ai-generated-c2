# Legacy v3 runtime (별도 참고)

새 콘솔 MVP와 호환되지 않는 과거 서버/대시보드 안내입니다. 기본 빌드에서는 제외됩니다.

실제 자산과 **UDP + Protobuf v3**로 통신하는 Windows 통제소입니다.
`c2_dashboard`는 실시간 화면, `c2_server`는 같은 런타임의 콘솔 서버입니다.
기존 시뮬레이션은 `c2_sim_dashboard`, `c2_console`로 분리했습니다.

## 바로 실행

Windows에서 실제 통제소 대시보드를 실행합니다.

```powershell
.\build\live\Release\c2_dashboard.exe --bind 0.0.0.0 --asset-port 5000
```

자산에서는 목적지를 **통제소 PC의 LAN IPv4 주소:UDP 5000**으로 지정합니다.
0.0.0.0은 서버의 바인드 주소이며 자산의 목적지 주소로 쓰지 않습니다.
같은 PC에서 실행하는 더미 자산의 예시:

```powershell
.\build\live\Release\dummy_observation.exe --asset-id 101 --listen-port 5101 --c2-address 127.0.0.1 --c2-port 5000
.\build\live\Release\dummy_effector.exe --asset-id 201 --listen-port 6001 --c2-address 127.0.0.1 --c2-port 5000
```

각 프로그램은 별도 터미널에서 실행합니다. 자산이 등록되면 대시보드에 자동 표시됩니다.
감시자산을 선택하고 `Scan observer`를 누르면 탐색 명령과 ACK를 확인할 수 있습니다.
효과기와 표적을 선택하고 `Assign + point`를 누르면 할당 후 각도 명령을 전송합니다.
`Stop selected`, `ESTOP all`을 지원합니다. 명령 전송과 장치 ACK 결과는 구분해 표시합니다.
대시보드와 콘솔 서버는 동일한 5000 포트를 함께 사용할 수 없습니다.

자산 담당자는 [접속 계약과 Zynq 연결 절차](asset-integration.md)를 먼저 확인하세요.
원시 라이다 입력을 직접 받는 서버가 아니라, 자산이 Protobuf 관측 결과를 보내는 서버입니다.

## 빌드와 검증

```powershell
git clone --recurse-submodules https://github.com/LIG-Embedded-School-Team-2-WarWarden/ai-generated-c2.git
cd ai-generated-c2
git submodule update --init --recursive
cmake -S . -B build/live -A x64 -DC2_BUILD_LIVE=ON
cmake --build build/live --config Release
ctest --test-dir build/live -C Release --output-on-failure
pwsh -File scripts/run_udp_smoke.ps1 -BinDir build/live/Release
```

Visual Studio C++20 도구와 Windows SDK가 필요합니다. 외부 테스트 라이브러리는 불필요합니다.
Linux에서는 초기 시뮬레이션 코어만 빌드되며 실제 UDP 서버는 현재 Winsock 기반입니다.
Windows에서는 `./scripts/build.ps1`로 Visual Studio 내장 CMake를 자동으로 찾아 빌드·시험할 수 있습니다.

검증 범위:

- 메모리 계약: 표적 식별·만료·중복 할당·용량·Heartbeat 처리
- 실제 대시보드 호스트 UDP: 등록·Pose·관측·양방향 Heartbeat·패킷 유실 재전송·ACK 완료·잘못된 패킷 거부
- 다중 프로세스: 감시 2대/효과기 3대, 상태·명령·이동 표적·세션 재시작·자동 할당·비상정지

실제 Zynq 및 센서/모터 하드웨어로 시험한 결과는 아닙니다.
대시보드의 버튼/시각 레이아웃은 자동 UI 시험 대상에 포함하지 않았습니다.

## 실제 통신 구조

- `src/live_dashboard.cpp`: 실제 자산 화면과 운용 요청
- `src/live_host.hpp`: UDP 소켓, 수신 큐, Heartbeat/재전송 worker의 수명 관리
- `runtime/`: 기존 통제소의 커밋된 통신·자산·표적·ACK 런타임 스냅샷
- `external/warwarden-protocol/`: 수정하지 않은 공용 계약/코덱 서브모듈
- `runtime/apps/`: 콘솔 서버와 실제 UDP 더미 클라이언트
- `include/c2/`: 초기 시뮬레이션 전용 인터페이스, live 런타임과 별도 타깃

출처와 고정 버전은 [runtime/ORIGIN.md](../runtime/ORIGIN.md)에 기록했습니다.
이 절은 과거 v3 구현의 기록이다. 새 콘솔은 공용 저장소의 ICD 1.0 스키마를 사용한다.

## 파트별 확장

통신은 `ServerRuntime::DatagramSender` Port와 `UdpTransport` Adapter로 분리됩니다.
화면은 `ServerRuntime` API를 호출하며 메시지 직렬화나 장비 SDK를 직접 다루지 않습니다.
라이다 담당자는 공용 `TargetCoordinate`를 생성하는 SDK 어댑터를 자산 쪽에 추가합니다.
팬틸트 담당자는 현재 wire 계약의 수신·ACK·상태 송신을 유지하고 계산/장치 구동을 구현합니다.

통제소의 수동 지향 및 후보 도달 가능성 계산을 교체할 때는
`runtime/include/c2/pointing.hpp`의 계약을 구현한 소스를 아래처럼 지정합니다.

```powershell
cmake -S . -B build/live -DC2_POINTING_SOURCE=C:/team/pointing_solver.cpp
```

현재 공용 계약에 따른 자동 추적은 자산이 `TargetTrackUpdate`를 받아 수행하는 방식입니다.
센서 로컬 좌표와 C2가 계산한 각도 명령에 track_id/유지 기한을 넣는 최신 설계는
아직 공용 저장소에 반영되지 않아 이번 실제 통신 버전에 포함하지 않았습니다.
Protobuf 후속 작업에서 그 책임 전환을 적용해야 합니다.
