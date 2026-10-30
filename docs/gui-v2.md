# GUI v2와 네이티브 C++ 통제소

기준 GUI: `LIG-Embedded-School-Team-2-WarWarden/c2-GUI`의
`fe22d07ded6258075c5260f45eb1b7747a131e70` (2026-10-30 PR #4).
기반 엔진: `test/native-device-integration@68d371b`.
통제소 엔진·CLI·장치 통신·GUI IPC·구조체·수동 코덱은 C++이다.
Python은 선택적인 loopback fake 장치와 통합 시험에만 사용한다.

## 규격

| 항목 | 규격 |
|---|---|
| SharedMemory | `Local\ControlStationMock.SharedMemory.v2` |
| Mutex | `Local\ControlStationMock.SharedMemory.Mutex.v2` |
| Magic / version | `0x314D4354` / 2 |
| Frame / Header / Payload | 484 / 32 / 452 bytes |
| RDR / FCS / Target | 각각 36 bytes, Pack=4 |
| Payload 내 target 배열 | offset 92, 최대 10개 |
| Pipe | `ECS_NamedPipe`, magic `0x50495045`, version 2 |
| Pipe request / response | 28 / 40 bytes |

RDR에는 positionX/Y와 fieldOfViewDeg, FCS에는 positionX/Y가 추가된다.
`gui/ipc_protocol.cpp`는 정수/float의 각 필드를 little endian으로 직접 직렬화/역직렬화한다.
float는 IEEE-754 binary32, FNV-1a는 payload 전체에 적용한다. 구조체 전체를 wire에
복사하지 않는다. 버전·길이·체크섬·개수·상태·예약값·좌표/FOV 유한성을 검증한다.
v1과는 호환되지 않는다.

## 각도와 좌표

엔진 Pan은 0=정면, 양수=왼쪽이다. GUI는 0~270을 받고 135=정면이므로
경계에서 `GUI = 135 - Pan`으로 변환한다. 기존 장치 명령의 좌표계는 유지한다.

| 장치 Pan | GUI 값 | 화면 |
|---|---|---|
| -135 | 270 | 오른쪽 뒤 |
| -90 | 225 | 오른쪽 |
| 0 | 135 | 정면 |
| +90 | 45 | 왼쪽 |
| +135 | 0 | 왼쪽 뒤 |

world `(forward,left,up)`의 장비 위치와 물체 위치를 GUI 평면 `(X,Y)=(-left,forward)`로
변환한다. 물체에는 센서 rotation·offset·설치 위치를 적용한다. `lidarZ`는 GUI 주석대로
원래 센서 Z를 보존한다. GUI ID 1은 설정의 OBS와 유일한 EFF에 대응한다.
ICD에 측정 FOV가 없으므로 0을 보내며 임의의 탐지 영역은 만들지 않는다.
장치 Pan ±135 초과는 사용 가능으로 표시하지 않는다. GUI 고각 입력은 0~90이지만,
실제 장치 capability 한계도 검증하므로 fake의 tilt 한계 ±45에서 46~90 요청은 거부된다.

GUI v2의 `isOperating`은 원본 모의 ECS의 `g_available` 매핑과 같이 장비가 운용 가능한
상태라는 뜻으로 사용한다. 모터의 MOVING 여부를 보내면 정지·도달한 자산에서 GUI가
전원 버튼을 막으므로, 설치 완료·유효 상태·각도 범위를 충족하면 1을 보낸다.
실제 모터의 이동/정지 판단은 C++ 엔진의 DeviceStatus.motion_state로 별도 유지한다.

## 명령

- 상태 조회(0): 현재 할당 및 레이저 전원 명령 결과.
- 할당(1): 최신 단일 관측을 기존 엔진으로 전달한다. POINT 확인, 현재 정지 상태,
  현재 실측 각도 도달을 확인한 뒤 Assigned를 표시한다.
- 취소(2): 기존 STOP 경로를 사용하고 확인된 정지 이후 Idle로 전환한다.
- Fire(3): 교육용 포인터의 **1초 전원 ON** 요청이다. 장치가 지원할 때만 기존
  LaserCommand를 사용한다. 확인된 LaserResult 이후 FireState=1을 표시한다.
  현재 `laserOn`은 유효한 LaserStatus로 표시한다. 전원 ON만으로 HIT를 만들지 않는다.
  중복 클릭으로 타이머를 재시작하지 않는다.
- SetRdrElevation(4): TargetId를 정수 고각으로 해석한다. 현재 OBS Pan을 유지하고
  기존 OBS 상태 전환으로 요청하며 설치·정지·장치 한계 검증을 유지한다.

GUI 요청은 엔진과 같은 직렬화된 스레드에서 처리한다. 별도 pipe 스레드는 요청/응답
전달만 담당한다. GUI 게시 주기는 50ms다. 부분 읽기, idle client, 재연결, 종료 중 I/O 취소를
처리한다. 같은 pipe 서버를 두 개 실행하지 않는다.

## 실행과 시험

```powershell
git submodule update --init --recursive
./scripts/build.ps1 -BuildDir build/gui-v2
./build/gui-v2/Release/c2_mvp_console.exe --gui
```

별도 GUI는 위의 고정 커밋을 사용한다. 기존 v1 GUI/모의 ECS 서버는 종료한다.
통제소 실행에는 Python이 필요하지 않다. 선택적인 fake 시연은 다음과 같다.

```powershell
./scripts/setup-team.ps1
./scripts/run_gui_v2.ps1 -GuiDir ../c2-GUI
# 기존 합성 점군이 필요한 고정 시야 시연
./scripts/run_gui_v2.ps1 -GuiDir ../c2-GUI -FixedObserver
```

기본 fake OBS는 Pan ±135를 초당 30도로 왕복하고 고각은 0이다. 회전 모드에서는
정지 자세용 합성 점군을 보내지 않으므로 물체 목록이 비어 있다. `-FixedObserver`는
기존 합성 점군을 보낸다. fake EFF는 실제 전기 출력을 제어하지 않는 모의 전원 상태다.
GUI 종료 시 C++ 엔진을 정상 정지하고 해당 시연의 fake도 종료한다.

```powershell
dotnet build tests/gui_v2_client/GuiProbe.csproj -c Release -p:GuiSource=C:/path/to/c2-GUI -o build/gui-probe
python tests/test_gui_v2.py --console build/gui-v2/Release/c2_mvp_console.exe --probe build/gui-probe/GuiProbe.dll -v
# GUI WPF도 빌드한 후 실제 표시 모델의 버튼 조건 확인
dotnet build/gui-probe/GuiProbe.dll --view-check build/gui-reference/AadcCommandCenter.dll
```

GuiProbe는 고정 GUI 원본 SharedMemoryFrame.cs/PipeCommandClient.cs를 직접 컴파일한다.
네이티브 프로세스·실제 UDP·shared memory·pipe를 거쳐 6개 각도, 좌표, 할당, 모의 전원
ON/OFF, 중복 요청, 취소, 고각을 확인한다. C++ gui_v2 시험은 golden byte offset,
484개 절단 길이, 버전/체크섬 오류, 신규 float 및 명령 경계를 검사한다.
레이저 결과 누락 시 완료를 표시하지 않는 것, pipe 재연결, 장치 단절도 확인한다.
2026-10-30 검증 결과: CTest 4/4 통과, 기존 Python 블랙박스 시험 16개 통과 및
POSIX 전용 1개 건너뜀, GUI v2 통합 시험 통과, 최신 WPF 및 실제 AssignmentView 검사 통과.
실제 하드웨어 피드백·실제 광출력·센서 FOV는 본 검증 범위가 아니다.
