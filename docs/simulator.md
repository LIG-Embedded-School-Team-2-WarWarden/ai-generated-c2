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
GUI 소스 수정 없이 기존 `ECS_NamedPipe`와 기본 공유 메모리 이름으로 연결합니다.
실행 전에 실제 운용 C2와 다른 모의기용 C2를 종료해야 합니다. 같은 PC에서 동시에 실행하지 않습니다.
제어창을 닫거나 WPF를 종료하면 이 실행에서 만든 C2와 모의기만 정리합니다.
이미 실행 중인 다른 통제소는 종료하지 않습니다.

모의기 실행기는 C2(ECS)도 자동으로 실행합니다. ECS를 따로 켜지 마세요.
`--gui`를 생략하면 장치 모의기·C2·제어창만 실행하며 WPF는 띄우지 않습니다.

### PowerShell에서 ps1 실행이 차단될 때

Visual Studio에서 Release/x64로 C2를 빌드하고, 별도 GUI에서 빌드한 WPF 실행 파일을 준비합니다.
이미 빌드돼 있으면 프로젝트 루트에서 다음 명령으로 직접 실행합니다.

```powershell
python .\simulators\run.py --gui ..\c2-GUI\build\Release\AadcCommandCenter.exe
```

Windows 실행 정책이나 사용자 환경변수를 바꿀 필요가 없습니다.
`CreateNamedPipe ... Win32 231` 오류는 기존 서버가 기본 파이프를 점유한 경우입니다.
기존 C2나 다른 모의 통제소를 종료하고 다시 실행합니다.
GUI와 공유 메모리 이름을 바꾸거나 다른 통제소를 임의로 종료하지 않습니다.

실행기는 WPF 자식 프로세스에 기본 공유 메모리 이름을 전달하고 `AADC9_PIPE_NAME` 환경변수를 제거합니다.
Windows의 사용자·시스템 환경변수는 변경하지 않습니다. GUI PR #7은 필요하지 않습니다.

## 시나리오

### 라이다 더미 + FCS 실장비

```powershell
python .\simulators\run.py --real-fcs --target 7 1 3 0.5 --gui ..\c2-GUI\build\Release\AadcCommandCenter.exe

# 장비 접속 없이 적용 주소 확인
python .\simulators\run.py --real-fcs --target 7 1 3 0.5 --print-plan
```

기존 ECS와 GUI를 종료한 뒤 실행합니다. 실행기가 ECS와 GUI를 함께 띄웁니다. `--real-fcs`는 **라이다만 더미**로 만들며 FCS 더미 소켓을 생성하지 않습니다. FCS는 기본 `configs/rdr-live.json`의 local/peer를 그대로 사용합니다. 현재 기본값은 ECS 192.168.0.102:2000 ↔ FCS 192.168.0.103:2000입니다. 다른 설정은 `--fcs-config C:\path\config.json`으로 선택합니다. 원본 설정 파일은 수정하지 않고 `logs/rdr-dummy-real-fcs/config.json`에 RDR만 바꾼 실행 설정을 만듭니다.

1. GUI에서 **라이다 구동**을 누릅니다.
2. RDR/FCS의 XYZ 위치를 공통 월드 기준으로 입력합니다. RDR은 기본 (0,0,0)을 사용합니다.
3. 더미 창에서 타겟 7의 월드 XYZ를 지정하고 **객체 위치 적용**을 누릅니다.
4. GUI에서 해당 타겟을 FCS에 **할당**하면 실제 FCS로 EF03 지향 명령을 보냅니다.
5. GUI의 FCS 실측 각도와 할당 완료/정렬 상태로 도달 여부를 확인합니다. 위치를 다시 바꾸면 재할당합니다.

더미는 시작하거나 좌표를 편집하는 것만으로 FCS 지향 명령을 보내지 않습니다. GUI 객체 할당으로 이동을 요청합니다. 실장비 모드 제어창에는 가짜 FCS 각도나 FCS 결함/연결 끊김 시나리오를 표시하지 않습니다.

### 라이다와 FCS 모두 모의 장비로 위치 지정

아래 일반 모드는 FCS도 모의 장비로 실행합니다.

```powershell
# ID=7, 월드 X=1m(오른쪽), Y=3m(앞쪽), Z=0.5m(위)
python .\simulators\run.py --target 7 1 3 0.5 --gui ..\c2-GUI\build\Release\AadcCommandCenter.exe

# 여러 객체도 지정 가능 (최대 10개)
python .\simulators\run.py --target 7 1 3 0.5 --target 8 -1 3 0.5
```

기존 ECS를 종료한 뒤 실행합니다. 실행기가 더미 RDR/FCS와 ECS를 함께 켜므로 F5로 ECS를 별도 실행하지 않습니다. WPF에서 라이다 구동을 누르면 지정 객체이 나타납니다.

WPF에서 객체를 FCS에 할당한 뒤 모의기 창의 **표적 월드 위치**에서 같은 ID를 선택합니다. X/Y/Z를 바꾸고 **표적 위치 적용**을 누르면 같은 ID의 새 좌표를 계속 보고합니다. 예를 들어 ID 7을 `(1, 3, 0.5)`에서 `(-1, 3, 0.75)`로 옮기면 좌우 회전과 고각 변화를 함께 확인할 수 있습니다.

제어창 오른쪽의 **FCS 모의 모터**에서 ECS가 보낸 EF03 명령각, 모의 모터의 현재각, 마지막 명령을 받은 뒤 움직인 각도를 확인합니다. 카메라 모형은 모의 모터의 현재각을 따라 움직입니다. 새로운 ID를 추가하려면 `--target`으로 다시 실행합니다.

라이다 X 좌우 반전과 관측 PAN/TILT를 포함한 원본 센서 패킷을 생성합니다. 입력 XYZ는 월드 미터 좌표입니다. 지정 객체 모드에서는 ECS가 보고하는 RDR 월드 위치를 더미에 자동 반영하므로 GUI에서 RDR 위치를 바꿔도 입력 타겟 월드 좌표가 유지됩니다. 동기화는 ECS 상태 주기(최대 약 2초) 후 소실/재검출로 반영되며 기존 할당은 재요청합니다. control.json의 `rdr_world`는 자동 기록됩니다. FCS 위치도 GUI에서 변경할 수 있습니다.

화면 없이 제어하려면 `--headless`를 붙이고 control.json을 다음처럼 수정합니다.

```json
{"scenario":"normal","target_count":1,"rdr_world":[0,0,0],"targets":[{"id":7,"xyz":[-1,3,0.75]}]}
```

이 예시는 ID 7을 왼쪽 1m, 앞쪽 3m, 높이 0.75m로 옮깁니다. `--target`을 생략하면 기존 10개 객체 모의 장면을 사용합니다.

| 버튼 | 동작 |
|---|---|
| 정상 · 객체 10개 | 관측 가능 상태, GUI 구동 버튼을 누르면 객체 1001~1010 보고 |
| 객체 소실 | 미검출 보고로 목록과 할당 최신성 변화 확인 |
| RDR 연결 끊김 | RDR 응답 중단, 연결 만료 확인 |
| FCS 연결 끊김 | FCS 응답 중단, 연결·사용 가능 상태 확인 |
| FCS 결함 | 상태의 fault bit, 사용 불가·명령 거부 확인 |
| 전체 종료 | 현재 모의기 실행과 그 자식 프로세스 정리 |

정상 버튼으로 복구합니다. RDR 연결이 끊겼다가 복구되면 GUI에서 구동을 다시 요청해야 합니다.
감시 중 모의 라이다의 팬·틸트가 함께 움직입니다. 각 DE03에는 그 자세에서의 원본 센서 XYZ와 같은 관측의 PAN·TILT가 실리며, ECS 보정 후 10개 정지 객체의 공통 좌표는 유지됩니다.
모의기 전용 설정은 localhost 주소와 자동 배정 포트로 생성하며 사용자의 실장비 설정 파일은 변경하지 않습니다.
객체 할당은 모의 FCS의 실측 각도가 목표에 도달한 뒤 완료됩니다. 데모 명령은 모의 상태 비트만 바꿉니다.
용어 정정 (2026-11-08): 여기서 ‘데모’는 기존 EF05 명칭입니다. 사용자는 실제 장치의 EF05를 촬영 시작으로 설명했으며, 모의기의 상태 비트 변경은 사진 저장이나 실장비 촬영 검증을 뜻하지 않습니다. [용어 정정](laser-control.md#용어-정정-2026-11-08)을 참고하세요.

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
기본 실행의 simulator-state.json에서 ipc_suffix는 빈 문자열입니다.
자동 시험은 `--isolated-ipc`로 `_sim_<id>` 접미사를 붙여 다른 실행과 분리합니다.
이 옵션은 원본 WPF와 함께 사용할 수 없습니다.

통신 자동 시험은 실제 Windows IPC를 이용해 팬·틸트 변화 전후 10개 정지 객체의 좌표 유지, 소수 각도, 모의 할당·동작·취소,
소실·결함·연결 단절·복구·구동·정지를 검사합니다. 실제 WPF 창의 시각 검증 결과는 별도로 기록합니다.
