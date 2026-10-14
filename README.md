# AI Generated C2

파트별 구현을 통합하기 위한 C++20 통제소입니다. Windows 대시보드와 콘솔을 제공합니다.
외부 라이브러리 없이 실행하며, 현재는 **시뮬레이션 전용**입니다.
기존 c2 및 warwarden-protocol 저장소를 수정하지 않습니다.

## 실행

Visual Studio Developer PowerShell 또는 CMake가 있는 환경:

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
.\build\Release\c2_console.exe
.\build\Release\c2_dashboard.exe
```

Windows 대시보드는 자산/표적 선택, 할당과 모의 지향, 개별/전체 정지,
최근 이벤트를 표시합니다. 기본으로 모의 관측 및 Heartbeat가 500ms마다 갱신됩니다.
상단 체크박스를 끄면 2초 뒤 표적, 3초 뒤 연결이 만료되며 할당도 해제됩니다.
실제 하드웨어 연결·레이저 출력·ACK 완료를 구현한 버전은 아닙니다.
화면의 queued는 로컬 전송 수락이며 장치 실행 성공을 뜻하지 않습니다.

콘솔 명령:

| 명령 | 동작 |
|---|---|
| `register ID X Y Z AZIMUTH` | 모의 자산 등록/갱신, 기존 할당 해제 |
| `heartbeat ID` | 모의 Heartbeat 갱신/재연결 |
| `sample [OBSERVER]` | 관측 샘플 입력, 기본 관측자 101 |
| `assets`, `targets`, `assignments` | 현재 상태 조회 |
| `assign ASSET TRACK` | 독점 할당, 최초 모의 지향이 큐에 들어간 경우 확정 |
| `point ASSET TRACK` | 모의 지향; 다른 표적에 할당된 자산은 거부 |
| `stop ASSET`, `stop-all` | 할당 해제 및 모의 정지; 연결 단절 자산 포함 |
| `events`, `status`, `help`, `quit` | 이력/요약/도움말/종료 |

콘솔은 명령 처리 시 `tick`을 호출하므로 입력을 기다리는 동안 만료 정리를 수행하지
않습니다. 다음 조회/명령 전에 만료가 반영됩니다. 대시보드는 250ms 타이머로 정리합니다.
콘솔 샘플을 할당하려면 `heartbeat 201`, `sample`, `assign 201 1` 순서로 빠르게 입력합니다.
할당은 명시적인 1회 지향을 포함하며, 관측 갱신에 따른 자동 재지향은 아직 없습니다.
정지 전송이 실패해도 할당은 해제되지만 장치가 정지했다고 간주하지 않습니다.
만료/단절에 따른 내부 할당 해제 역시 실제 장치 정지 확인을 뜻하지 않습니다.

기본 한도는 자산 64개, 활성 트랙 256개, 이벤트 128개입니다. `Limits`로 변경할 수
있습니다. 만료 트랙과 식별자 매핑은 함께 제거하고 전역 트랙 ID는 재사용하지 않습니다.

## 설계 패턴과 파트별 작업

Ports & Adapters: 핵심 로직은 외부 장비나 직렬화 라이브러리를 모르고
`ILidarProcessor`, `IPointingSolver`, `IAssetTransport` 계약에만 의존합니다.
Strategy: 라이다 처리 및 지향 계산 구현을 교체할 수 있습니다.
Dependency Injection: 생성자에 구현체를 주입하며 `src/main.cpp` 한 곳에서 조립합니다.
별도의 동적 플러그인 로더는 없고 소스 추가 후 재빌드하는 방식입니다.

| 담당 | 구현할 계약 | 입력/출력 |
|---|---|---|
| 라이다 | ILidarProcessor | LidarFrame → Observation 목록 |
| 팬틸트 계산 | IPointingSolver | Asset + Observation → optional Angles |
| 통신 | IAssetTransport | PointCommand / stop → 로컬 큐 수락 여부 |

1. `include/c2/control_center.hpp`의 인터페이스를 상속합니다.
2. 별도 `adapters/<part>/` 폴더에 구현을 추가합니다.
3. 필요한 소스/SDK 의존성을 CMake 대상에 연결합니다.
4. `main.cpp`에서 해당 더미를 새 구현체로 교체합니다.
5. 담당 알고리즘의 실패·단위·좌표변환 시험을 추가합니다.

내부 좌표 계약은 PROJECT_FRAME, 거리 m, 각도 deg,
시각 UTC Unix microseconds입니다. 센서 로컬 좌표 및 장착 자세의 변환은
라이다 어댑터에서 수행한 뒤 Observation으로 전달해야 합니다.
관측자 내부 detection_id는 안정적으로 유지되어야 합니다.
DemoLidarProcessor의 배열 순번 ID는 테스트용이며 실제 추적 ID가 아닙니다.
DemoPointingSolver는 항상 15/5도를 반환하는 테스트 구현입니다.
실제 계산기는 자산 설치 방위, 구동 한계 및 계산 불가능 조건을 처리해야 합니다.
인터페이스 구현은 직렬 호출 기준이며 예외 처리는 어댑터 책임입니다.

## Protobuf: 후속 작업

이번 변경은 `.proto`, 생성 코드, 공용 프로토콜 저장소를 변경하지 않습니다.
내부 C++ 구조체는 네트워크 메시지 규격이 아닙니다.
후속 통신 어댑터에서 기존 공용 Protobuf 메시지와 내부 모델을 변환합니다.
JSON이나 메모리 구조체를 자산 간 통신 규격으로 사용하지 않습니다.

후속 통합 항목: 실제 등록/session/lease, UDP 수신, 실제 Heartbeat, ACK·재전송,
상태 최신성, 제한된 송수신 큐, 자동 할당 정책,
기구 한계, 정렬 피드백, 비상정지와 장치 측 안전 처리입니다.
통신이 연결되기 전에 위 항목을 구현해야 합니다.

Zynq의 Linux/bare-metal/RTOS 여부를 확인한 뒤 C++ Protobuf 또는
nanopb 적용과 지원 필드 범위를 결정합니다. 공용 메시지 변경은 별도 작업입니다.
