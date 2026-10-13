# AI Generated C2

파트별 구현을 통합하기 위한 C++20 콘솔 통제소의 첫 기반입니다.
외부 라이브러리 없이 실행하며, 현재는 **시뮬레이션 전용**입니다.
기존 c2 및 warwarden-protocol 저장소를 수정하지 않습니다.

## 실행

Visual Studio Developer PowerShell 또는 CMake가 있는 환경:

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
.\build\Release\c2_console.exe
```

`assets`, `sample`, `targets`, `point 201 1`, `stop 201`, `quit`을 지원합니다.
sample 표적은 2초 동안 유효합니다. 실제 하드웨어 연결·레이저 출력·ACK 완료를
구현한 버전은 아닙니다. 화면은 현재 콘솔이며 GUI는 후속 단계입니다.

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

후속 통합 항목: 등록/session/lease, UDP 수신, Heartbeat, ACK·재전송,
상태 최신성, 제한된 큐/트랙 수 및 만료 정리, 자산 할당,
기구 한계, 정렬 피드백, 비상정지와 장치 측 안전 처리입니다.
현재 tracks/identity 목록은 데모 실행 동안 유지되며 용량 제한이 없습니다.
통신이 연결되기 전에 위 항목을 구현해야 합니다.

Zynq의 Linux/bare-metal/RTOS 여부를 확인한 뒤 C++ Protobuf 또는
nanopb 적용과 지원 필드 범위를 결정합니다. 공용 메시지 변경은 별도 작업입니다.
