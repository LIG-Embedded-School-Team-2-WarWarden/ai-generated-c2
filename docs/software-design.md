# 현재 C2 구조

진입점은 `C2.Console/Sources/main.cpp`입니다.
장비 통신은 UDP Big Endian 이진 ICD, WPF 연결은 같은 PC의 IPC v3입니다.
별도 GUI 저장소의 화면 소스는 이 저장소에서 수정하거나 포함하지 않습니다.

| 프로젝트·파일 | 책임 |
|---|---|
| C2.Console/Sources/main.cpp | 설정·CLI, 장치 상태 수집, GUI 요청, 프레임 게시, 할당 각도 계산 |
| C2.Console/Headers/rdr_control.hpp | 보고 확인 후 감시 시작, 구동·정지 |
| C2.Console/Headers/assignment_health.hpp | 할당·정렬 유효성 |
| C2.Console/Headers/fcs_angles.hpp | 모터 각도와 GUI 각도 변환 |
| C2.Console/Headers/installation_config.hpp | 설치 좌표 형식·검증 |
| C2.Core/Include/c2/rdr_client.hpp, rdr_session.hpp | RDR 세션·연결·재전송·표적 상태 |
| C2.Core/Sources/fcs_client.cpp, fcs_protocol.cpp | FCS 요청·응답과 코덱 |
| C2.Core/Sources/udp_transport.cpp | UDP 송수신 |
| C2.Ipc/Headers/protocol.hpp, Sources/protocol.cpp | IPC v3 구조·코덱 |
| C2.Ipc/Sources/transport.cpp | Windows 공유 메모리·mutex·Named Pipe |
| C2.Tests | 장비 측 fixture 및 ICD·UDP·IPC 회귀 시험 |
| simulators/devices.py, run.py | loopback 모의 장치와 실행·제어창 |

RDR DE03은 32바이트이며 Base(right,up,forward), float32 미터 좌표입니다. GUI 위치는 (right,forward,up), m입니다.
할당은 RDR 측정 벡터에 RDR 위치를 더하고 FCS 위치를 빼서 FCS 기준으로 계산합니다.
고각·모터 방위각을 0.1도 간격으로 반올림하고 FCS 한계를 검사합니다.
위치를 바꾸면 기존 정렬을 무효화하며 재할당해야 합니다.
자동 재지향·설치 회전축 보정·GUI 위치 편집의 영구 저장은 포함하지 않습니다.

기본 IPC 이름은 `ECS_NamedPipe`, `Local\\ControlStationMock.SharedMemory.v3`입니다.
기본 C2는 하나만 실행합니다. 자동 시험의 별도 접미사는 원본 WPF와 함께 사용하지 않습니다.
보드에 없는 고각 설정 명령은 NotSupported로 응답합니다.
상태 최신성과 응답 peer·ID·순번·길이를 검사합니다.
FCS 제어 응답 미수신은 결과 미확정이며 자동 재송신하지 않습니다.
[GUI IPC](operator-ipc.md), [장비 ICD](device-icd.md)에 상세 규격이 있습니다.
