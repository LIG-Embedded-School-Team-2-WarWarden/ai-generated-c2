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

RDR DE03은 32바이트이며 원본 센서 XYZ(float32 미터, X 좌우 반전·Y 위쪽·Z 앞쪽)와 같은 관측의 PAN·TILT를 담습니다.
실장비 수신 X는 좌우가 반전되어 있습니다. `sensor_to_base`에서 `right=-raw_x`로 정규화한 뒤 관측 TILT/PAN 회전을 적용합니다. 수신 원본은 변경하지 않으며 GUI 좌표나 설치 위치에 추가 반전을 적용하지 않습니다.
`rdr_coordinates.hpp`가 관측 TILT와 PAN을 적용해 공통 좌표(동쪽,북쪽,위쪽)로 변환합니다. 현재 장치 상태의 자세로 과거 관측을 다시 회전하지 않습니다.
GUI는 보정한 공통 좌표에 RDR 설치 위치를 더한 값을 표시합니다. 할당도 같은 보정 좌표에 RDR 위치를 더하고 FCS 위치를 빼서 계산합니다.
`correct_targets`가 한 번의 RDR 상태 스냅샷에서 관측 자세 보정과 설치 위치 이동을 적용합니다. GUI 표적 목록과 FCS 할당은 이 결과를 함께 사용하며, 전송 직전에 원본 XYZ로 덮어쓰거나 좌표를 다시 회전하지 않습니다.
FCS 할당 로그에는 보정된 GUI XYZ, FCS 기준 상대 XYZ와 계산한 PAN/TILT를 남깁니다.
고각·모터 방위각을 0.1도 간격으로 반올림해 전송합니다. ECS의 자산별 팬·틸트 구동 한계와 설정 필드는 제거했으며 구형 `fcs.limits`는 무시합니다. 장비의 승인·거부 응답은 그대로 반영합니다.
FCS 실장비 관찰에 따라 북쪽 PAN=180°, 오른쪽 회전이 양수입니다. 명령은 `180+atan2(east,north)`이며 GUI 지도용 값은 `PAN-45`를 0~360°로 정규화합니다. 기존의 왼쪽 양수 가정은 제거했습니다.
위치를 바꾸면 기존 정렬을 무효화하며 재할당해야 합니다.
PAN=1800은 북쪽, TILT=0은 수평이며 오른쪽/위쪽 회전이 양수입니다. 관측 자세가 미상이면 원본 데이터는 보존하되 공통 좌표는 무효로 처리합니다.
센서와 모터 축은 정렬·동일 원점으로 가정합니다. 자동 재지향·실측 장착 오차 보정·GUI 위치 편집의 영구 저장은 포함하지 않습니다.

기본 IPC 이름은 `ECS_NamedPipe`, `Local\\ControlStationMock.SharedMemory.v3`입니다.
기본 C2는 하나만 실행합니다. 자동 시험의 별도 접미사는 원본 WPF와 함께 사용하지 않습니다.
보드에 없는 고각 설정 명령은 NotSupported로 응답합니다.
상태 최신성과 응답 peer·ID·순번·길이를 검사합니다.
FCS 제어 응답 미수신은 결과 미확정이며 자동 재송신하지 않습니다.
[GUI IPC](operator-ipc.md), [장비 ICD](device-icd.md)에 상세 규격이 있습니다.
