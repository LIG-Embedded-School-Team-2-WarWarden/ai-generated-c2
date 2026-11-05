# RDR/FCS 모의기와 WPF 확인

모의기는 실제 RDR/FCS 대신 로컬 UDP로 응답하며, 실제 C2.Console이 그 값을 GUI v3로 게시합니다.
장비 출력은 메모리 안의 상태 변화입니다. 모든 장비 IP는 127.0.0.1이며 포트는 실행 시 자동 배정합니다.

## 실행

프로젝트 루트에서 실행합니다. Python 표준 라이브러리(tkinter 포함), Visual Studio C++와 .NET 10 SDK가 필요합니다.

```powershell
# 모의기 제어창 + C2 + WPF 빌드 및 실행
.\scripts\run_simulator.ps1 -Gui

# 이미 빌드했으면 바로 실행
.\scripts\run_simulator.ps1 -Gui -NoBuild

# 다른 위치의 GUI 소스를 사용할 때
.\scripts\run_simulator.ps1 -Gui -GuiDir 'C:\path\to\c2-GUI'
```

GUI 소스 기본 위치는 프로젝트 옆의 `../c2-GUI`입니다. 확인한 원본은 GUI PR #5/#6의
`94bca8c5c477e4cf1bb15b93c288a5c450603910`이며 해당 폴더의 SOURCE_REVISION.txt에 기록했습니다.
제어창을 닫거나 WPF를 종료하면 이 실행에서 만든 C2와 모의기만 정리합니다.
이미 실행 중인 다른 통제소는 종료하지 않습니다.

기본 GUI 연결 이름과 겹치지 않도록 `_sim_<id>` 접미사를 사용합니다.
WPF는 기존 AADC9_SHARED_MEMORY_NAME/AADC9_SHARED_MEMORY_MUTEX_NAME과 함께
선택적인 AADC9_PIPE_NAME 환경변수를 사용합니다. 원본 GUI에는 다음 한 줄의 변경만 적용했습니다.

```csharp
private static readonly string PipeName = Environment.GetEnvironmentVariable("AADC9_PIPE_NAME") ?? "ECS_NamedPipe";
```

환경변수가 없으면 기존 GUI 동작을 유지합니다. 각도·레이아웃·버튼 코드는 원본입니다.

## 시나리오

| 버튼 | 동작 |
|---|---|
| 정상 · 표적 10개 | 관측 가능 상태, GUI 구동 버튼을 누르면 표적 1001~1010 보고 |
| 표적 소실 | 미검출 보고로 목록과 할당 최신성 변화 확인 |
| RDR 연결 끊김 | RDR 응답 중단, 연결 만료 확인 |
| FCS 연결 끊김 | FCS 응답 중단, 연결·사용 가능 상태 확인 |
| FCS 결함 | 상태의 fault bit, 사용 불가·명령 거부 확인 |
| 전체 종료 | 현재 모의기 실행과 그 자식 프로세스 정리 |

정상 버튼으로 복구합니다. RDR 연결이 끊겼다가 복구되면 GUI에서 구동을 다시 요청해야 합니다.
모의기 전용 생성 설정에서만 assignment_enabled=true를 사용합니다. 사용자의 실장비 설정은 변경하지 않습니다.
표적 할당은 모의 FCS의 실측 각도가 목표에 도달한 뒤 완료됩니다. 데모 명령은 모의 상태 비트만 바꿉니다.

GUI 좌표 편집은 현재 C2 실행의 지도와 FCS 할당 계산에 적용합니다. 변경 후에는 기존 할당을 다시 요청해야 합니다.
장치 회전축 보정·영구 저장과는 구분합니다.
고각 설정은 실제 RDR ED/DE 계약에 메시지가 없어 현재 C2가 미지원으로 응답합니다.
미지원 오류 안내도 확인 항목에 포함하며 가상의 장치 명령을 추가하지 않습니다.

## 무인 검증과 로그

```powershell
python .\C2.Tests\Integration\test_simulator.py

# 화면 없이 장기 실행; logs/simulator/control.json으로 시나리오 변경 가능
python .\simulators\run.py --headless
```

logs/simulator에 config.json, control.json, simulator-state.json, c2-state.json,
c2.stdout.log, c2.stderr.log가 생성됩니다. control.json 형식은 다음과 같습니다.

```json
{"scenario":"normal","target_count":10}
```

scenario는 normal/empty/rdr-offline/fcs-offline/fcs-fault입니다. stop.txt를 만들면 종료됩니다.
GUI 메모리 이름은 simulator-state.json의 ipc_suffix로 확인할 수 있습니다.

통신 자동 시험은 실제 Windows IPC를 이용해 10개 표적, 소수 각도, 모의 할당·동작·취소,
소실·결함·연결 단절·복구·구동·정지를 검사합니다. 실제 WPF 창의 시각 검증 결과는 별도로 기록합니다.
