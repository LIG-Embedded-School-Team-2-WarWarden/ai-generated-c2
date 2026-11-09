# C2 — Visual Studio

용어 정정 (2026-11-08): 사용자는 EF05를 촬영 시작 명령, 레이저를 촬영 중인 객체 표시 용도로 확인했습니다. 기존 문서의 EF05 ‘장치 데모’ 표현과 근거가 확인되지 않은 ‘사격’ 표현에 대한 정정은 [FCS 용어 설명](docs/laser-control.md#용어-정정-2026-11-08)과 [좌표 평가 명세의 정정 기록](docs/stationary-coordinate-evaluation-spec.md)을 참고하세요. 이 기록은 사용자 설명을 반영한 것으로, 실장비 동작의 독립 검증 결과는 아닙니다.

루트 **ai-generated-c2.sln**을 열고 **C2.Console / Debug / x64**에서 F5로 실행합니다.
Visual Studio에서 파일 추가와 프로젝트 속성을 직접 관리합니다.

| 항목 | 용도 |
|---|---|
| C2.Console | 콘솔과 운영 제어 |
| C2.Core | RDR·FCS 이진 ICD와 UDP 통신 |
| C2.Ipc | 별도 c2-GUI와 연결하는 서버 어댑터 |
| C2.Tests | 독립 장비 규격 시험·Python UDP 모의기 |
| configs | 실행 주소와 장비 연결 설정 |
| scripts | 빌드·시험·실물 실행 |
| docs | 장비 통신·외부 화면 연결 규격 |
| x64 / obj | 빌드 결과 / 중간 파일(자동 생성) |

장비 통신은 **RDR 제어 24바이트·DE03 32바이트**, **FCS EF/FE 8·12바이트** Big Endian ICD만 사용합니다.
DE03는 2026-11-05 사용자 확정 계약에 따라 원본 센서 XYZ·관측 PAN/TILT·TrackState를 수신합니다.
OBSERVED와 PREDICTED의 좌표를 보존하며, LOST는 좌표와 관측 자세를 무효화합니다.
JSON 헤더는 C2.Core/Include/nlohmann에 포함되어 있습니다. 빌드는 Visual Studio C++ 도구만 필요합니다.
GUI 프로그램은 별도 저장소에서 실행하며, C2에는 화면 코드가 없습니다.

## 실행

F5는 configs/rdr-live.json의 ICD 장비 주소를 사용합니다. configs/team-demo.json은 localhost 모의 시험용입니다. 실제 장비는 다음처럼 실행합니다.

```powershell
./scripts/build.ps1 -Configuration Release
./scripts/test.ps1 -Configuration Release
./scripts/run_rdr_live.ps1 -ConfigFile configs/rdr-live.json
```

```text
rdr report on
rdr start
rdr status
rdr targets
rdr exclude 7
rdr resume 7
rdr stop
fcs connect
fcs status
fcs point 20 180
fcs laser on
fcs laser off
quit
```

FCS point 인수는 고각·모터 방위각(도, 0.1도 간격) 순서입니다. ECS는 자산별 팬·틸트 구동 한계로 명령을 제한하지 않습니다. 기존 설정의 `fcs.limits`는 무시합니다.
실장비 회전 방향에 맞춰 FCS는 북쪽 PAN=180°, 오른쪽 회전이 양수입니다. `PAN=180°+atan2(FCS 상대 east,north)`를 사용하며 기존 GUI 지도용 각도는 `PAN-45°`를 0~360°로 정규화합니다.
응답의 confirmed는 해당 ICD 응답을 받았다는 뜻이며, 위치 도달은 다음 fcs status의 실측값으로 확인합니다.
수동 laser on은 최대 1초 뒤 OFF를 요청하고 정상 종료 시에도 OFF를 요청합니다.
제어 응답이 없으면 성공으로 처리하거나 같은 제어를 자동 재송신하지 않습니다.

외부 GUI는 같은 PC에서 실행합니다. 기존 GUI v3(프레임492/페이로드460/객체36, 요청40/응답40바이트)를 유지합니다. GUI 수정 없이 ECS만 갱신합니다.
GUI 각도는 float이며 RDR/FCS 장치 각도는 0.1도 단위 정수를 도 단위로 변환합니다.
센서 축은 오른쪽/위쪽/앞쪽이며, 관측 당시 자세를 적용해 GUI 동쪽/북쪽/위쪽 좌표로 변환합니다. PAN=1800은 북쪽(내부 방향0°, 기존 GUI 전달값135°), 오른쪽 회전이 양수이고 TILT는 수평0°·위쪽 양수입니다.
GUI 목표 목록 XYZ는 자세 보정 후 RDR 설치 위치를 더한 월드 좌표(미터)입니다. FCS는 같은 월드 좌표에서 FCS 설치 위치를 뺀 상대 벡터로 지향각을 계산합니다.
실제 라이다의 수신 X는 좌우 반전되어 있으므로 월드 변환 직전에 `센서 오른쪽 = -수신 X`로 한 번 보정합니다. 원본 `raw_x`는 그대로 보존하며 GUI와 FCS는 보정된 월드 좌표를 함께 사용합니다.
GUI 목표 목록과 지도는 같은 ID에도 최신 수신 XYZ·관측 PAN/TILT로 다시 계산한 월드 좌표를 표시합니다. 객체 ID별 칼만 필터를 적용하며, 실장비 설정은 정지 상태 2분 기록을 바탕으로 X/Y/Z 측정 잡음을 각각 2.5/1.8/0.5cm로 둡니다. 각 축의 과정 잡음을 따로 적용하고, 두 번의 실제 관측으로 이동을 확인하면 보정 속도를 높입니다. 최초 좌표로 고정하지 않으며 `rdr live on` 없이도 갱신됩니다. `rdr.gui_kalman`에서 조절하거나 끌 수 있습니다. 설정과 초기화 조건은 [GUI 좌표 필터](docs/operator-ipc.md#gui-좌표-필터)를 참고하세요. GUI와 FCS 객체 할당은 명령 처리 전에 계산한 동일한 최신 필터 좌표를 사용합니다. GUI 전송값을 다시 저장하거나 최초 좌표를 고정하지 않습니다. 필터 상태는 소실·제외·연결 세션 변경·RDR 설치 위치 변경 시 초기화합니다.
GUI 필터는 OBSERVED와 PREDICTED 모두에 적용합니다. 예측 좌표는 표시를 부드럽게 보정하되 독립 측정으로 간주해 필터의 확신도를 높이지 않습니다. 소실과 기존 수신 타임아웃은 그대로 적용합니다.
센서 확인용 콘솔 명령 `rdr live on`은 필터 적용 전의 최신 월드 좌표를 200ms 간격으로 출력합니다. `rdr targets`의 `gui_xyz`는 GUI 표시용 필터 결과입니다. `rdr live off`로 콘솔 조회를 종료해도 GUI 목록은 계속 갱신됩니다. 필드와 동작은 [콘솔 사용법](docs/console-mvp.md)을 참고하세요.
센서와 모터 축의 장착 회전은 단위행렬, 장착 이동량은 0인 기준 가정입니다. 실장비 장착 기하와 좌표 정확도 검증은 별도로 필요합니다.
원본 XYZ·관측 자세·Detected·TrackState는 ECS 내부와 콘솔 JSON에서 보존합니다. GUI에는 기존 유효 좌표만 전달하며 예측 라벨·관측 자세 상세는 추가하지 않습니다.
예측의 마지막 관측/점군 지지 후 최대 1.5초 제한은 송신측 책임이며, ECS의 마지막 DE03 수신 후 3초 무효화는 별도 통신 타임아웃입니다.
GUI의 라이다 구동·정지 버튼과 RDR/FCS XYZ 위치 편집을 지원합니다. 위치 편집은 ECS가 `--config` JSON의 `installation` 항목에 저장하고 재시작 시 복원합니다. 기본 실장비 실행은 `configs/rdr-live.json`에 저장하며 `--config` 없이 실행하면 실행 파일 옆 `installation.json`을 사용합니다. 저장 성공 후 지도와 FCS 할당 계산에 반영되며 변경 후 재할당이 필요합니다.
RDR은 실행 시 자동 연결하며 연결이 끊기면 재연결합니다. GUI 구동 요청은 보고 허용 확인 후 감시를 시작하며, 정지 시 측정값을 무효화합니다.
콘솔의 `rdr report on`, `rdr start`, `rdr stop`도 유지합니다.
ED/DE 보드에 고각 설정 메시지가 없어 GUI 고각 설정은 미지원으로 응답합니다. 자세한 규격은 [GUI IPC](docs/operator-ipc.md)를 참고하세요.
GUI 객체 할당은 RDR 감시·보고, FCS 연결·상태와 객체 유효성을 확인한 뒤 처리합니다. 음수 틸트도 장비에 전송하며 장비의 승인·거부 응답을 반영합니다. 장치 데모는 할당과 실측 정렬이 확인된 뒤 요청할 수 있습니다.
실물 센서·모터 검증과 로컬 모의기 통신 시험은 구분합니다.

## WPF 모의기

`./scripts/run_simulator.ps1 -Gui`로 RDR/FCS 모의기 제어창, C2, WPF를 함께 실행합니다.
정상·객체 소실·연결 끊김·FCS 결함을 선택할 수 있습니다. 통신은 localhost로 제한하며
기존 GUI의 v3 기본 IPC 이름을 사용합니다. 기존 C2와 WPF를 종료한 뒤 실행하세요.

이미 빌드된 C2와 WPF가 있으면 Python으로 직접 실행할 수 있습니다.
PowerShell 실행 정책으로 `.ps1`이 차단된 경우에도 아래 명령을 사용할 수 있습니다.

```powershell
python .\simulators\run.py --gui ..\c2-GUI\build\Release\AadcCommandCenter.exe
```

실행기가 C2도 함께 켜므로 ECS를 별도로 실행할 필요가 없습니다.
`--gui`를 생략하면 장치 모의기·C2·제어창만 실행합니다.
자세한 방법은 [모의기 안내](docs/simulator.md)를 참고하세요.
라이다만 더미로 두고 FCS 실장비를 쓰려면 실행기에 `--real-fcs --target 7 1 3 0.5`를 추가합니다. FCS 주소는 `configs/rdr-live.json`에서 읽으며, 타겟 위치 적용 후 GUI 객체 할당으로 실제 지향을 요청합니다.

## 문서

- [실장비 주소와 연결](docs/asset-integration.md), [RDR ED/DE 연결](docs/rdr-edde24.md)
- [콘솔 명령](docs/console-mvp.md), [FCS 수동 동작과 GUI 데모](docs/laser-control.md)
- [라이다 필터 전후 좌표 기록과 흔들림 비교](docs/lidar-coordinate-recording.md)
- [장비 ICD](docs/device-icd.md), [GUI IPC v3](docs/operator-ipc.md)
- [현재 코드 구조](docs/software-design.md), [빌드와 검증](docs/development-process.md)
- [모의기 실행](docs/simulator.md), [WPF 검증 기록](docs/wpf-verification.md)

구형 CMake/Protobuf 런타임·OBS/EFF CLI·GUI v2 코드는 제거했습니다. 이전 구현은 Git 이력에서 확인할 수 있습니다.
