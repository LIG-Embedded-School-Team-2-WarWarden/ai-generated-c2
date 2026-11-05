# C2 — Visual Studio

루트 **ai-generated-c2.sln**을 열고 **C2.Console / Debug / x64**에서 F5로 실행합니다.
Visual Studio에서 파일 추가와 프로젝트 속성을 직접 관리합니다.

| 항목 | 용도 |
|---|---|
| C2.Console | 콘솔과 운영 제어 |
| C2.Core | RDR·FCS 이진 ICD와 UDP 통신 |
| C2.Ipc | 별도 c2-GUI와 연결하는 서버 어댑터 |
| C2.Tests | 독립 장비 규격 시험·Python UDP 모의기 |
| configs | 실행 주소와 장비 한계 설정 |
| scripts | 빌드·시험·실물 실행 |
| docs | 장비 통신·외부 화면 연결 규격 |
| x64 / obj | 빌드 결과 / 중간 파일(자동 생성) |

장비 통신은 **RDR 제어 24바이트·DE03 32바이트**, **FCS EF/FE 8·12바이트** Big Endian ICD만 사용합니다.
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

FCS point 인수는 고각·모터 방위각(도, 0.1도 간격) 순서입니다. 설치 장비의 실제 한계를 configs의 limits에 지정합니다.
응답의 confirmed는 해당 ICD 응답을 받았다는 뜻이며, 위치 도달은 다음 fcs status의 실측값으로 확인합니다.
수동 laser on은 최대 1초 뒤 OFF를 요청하고 정상 종료 시에도 OFF를 요청합니다.
제어 응답이 없으면 성공으로 처리하거나 같은 제어를 자동 재송신하지 않습니다.

외부 GUI는 같은 PC에서 실행합니다. c2-GUI PR #5/#6의 IPC v3(프레임492/요청40/응답40바이트)를 사용합니다.
GUI 각도는 float이며 RDR/FCS 장치 각도는 0.1도 단위 정수를 도 단위로 변환합니다.
GUI의 라이다 구동·정지 버튼과 RDR/FCS XYZ 위치 편집을 지원합니다. 위치 편집은 현재 실행의 지도와 FCS 할당 계산에 반영되며 변경 후 재할당이 필요합니다.
RDR은 실행 시 자동 연결하며 연결이 끊기면 재연결합니다. GUI 구동 요청은 보고 허용 확인 후 감시를 시작하며, 정지 시 측정값을 무효화합니다.
콘솔의 `rdr report on`, `rdr start`, `rdr stop`도 유지합니다.
ED/DE 보드에 고각 설정 메시지가 없어 GUI 고각 설정은 미지원으로 응답합니다. 자세한 규격은 [GUI IPC](docs/operator-ipc.md)를 참고하세요.
GUI 표적 할당은 RDR 감시·보고, FCS 연결·상태, 표적과 구동 한계 검사를 통과하면 처리합니다. 장치 데모는 할당과 실측 정렬이 확인된 뒤 요청할 수 있습니다.
실물 센서·모터 검증과 로컬 모의기 통신 시험은 구분합니다.

## WPF 모의기

`./scripts/run_simulator.ps1 -Gui`로 RDR/FCS 모의기 제어창, C2, WPF를 함께 실행합니다.
정상·표적 소실·연결 끊김·FCS 결함을 선택할 수 있습니다. 통신은 localhost로 제한하며
원본 GUI의 기본 IPC 이름을 사용합니다. 기존 C2와 WPF를 종료한 뒤 실행하세요.
GUI 소스 수정이나 사용자 환경변수 설정은 필요하지 않습니다.

이미 빌드된 C2와 WPF가 있으면 Python으로 직접 실행할 수 있습니다.
PowerShell 실행 정책으로 `.ps1`이 차단된 경우에도 아래 명령을 사용할 수 있습니다.

```powershell
python .\simulators\run.py --gui ..\c2-GUI\build\Release\AadcCommandCenter.exe
```

실행기가 C2도 함께 켜므로 ECS를 별도로 실행할 필요가 없습니다.
`--gui`를 생략하면 장치 모의기·C2·제어창만 실행합니다.
자세한 방법은 [모의기 안내](docs/simulator.md)를 참고하세요.

## 문서

- [실장비 주소와 연결](docs/asset-integration.md), [RDR ED/DE 연결](docs/rdr-edde24.md)
- [콘솔 명령](docs/console-mvp.md), [FCS 수동 동작과 GUI 데모](docs/laser-control.md)
- [장비 ICD](docs/device-icd.md), [GUI IPC v3](docs/operator-ipc.md)
- [현재 코드 구조](docs/software-design.md), [빌드와 검증](docs/development-process.md)
- [모의기 실행](docs/simulator.md), [WPF 검증 기록](docs/wpf-verification.md)

구형 CMake/Protobuf 런타임·OBS/EFF CLI·GUI v2 코드는 제거했습니다. 이전 구현은 Git 이력에서 확인할 수 있습니다.
