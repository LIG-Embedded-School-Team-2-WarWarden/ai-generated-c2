# 카메라 설정 저장 및 독립 미리보기

## 작업 위치

- ECS: `ai-generated-c2`
- GUI: `camera-settings-gui` (GitHub c2-GUI main `45f210e6506bdff0c88c1340b5451aaf4934b576`에서 새로 복제)
- 기존 GUI 사본과 기존 사용자 변경은 유지했다. 커밋/푸시는 하지 않았다.

## 설정

기존 ECS `--config` JSON의 `installation.camera`에 다음을 추가한다.

```json
{"rdr_zero":[0,0,0],"fcs_zero":[0,0,0],"ranges":[1.5,10,3]}
```

`ranges` 순서는 RDR 최소 인식 거리, RDR 최대 인식 거리, FCS 최대 촬영 거리다.
모두 m 단위이며 영점 보정은 GUI XYZ 좌표계다. 기존 `devices.*.position`은
설치 위치로 유지한다. 기존 좌표계 변환도 유지한다.
기존 설정에 camera 필드가 없으면 기본값을 사용한다.
`--config`가 없으면 실행파일 옆 `installation.json`에 저장한다.

영점 보정과 범위는 이번 작업에서 저장/조회/표시만 한다. 실제 장비 명령,
좌표 계산, 대상 할당, 출력 제어에 적용하지 않는다. 설치 위치 변경은 기존 동작을 유지한다.

## GUI

메인 화면의 위치 입력을 영점 보정 입력으로 변경하고 실제 설치 위치를 별도로 표시한다.
장비 설정 팝업의 설치 위치 두 행과 거리 행은 **행별로 저장**한다.
ECS 성공 응답 후에만 저장 완료를 표시하고 저장값으로 미리보기를 갱신한다.
실패하면 입력은 재시도를 위해 남겨두고 저장값과 미리보기는 유지한다.
미리보기는 XY 평면이며 RDR/FCS 각각의 실제 설치 위치를 중심으로 그린다.
그림 바로 아래 노란색/빨간색 범례가 있다. Z와 영점 보정은 평면 범위에 적용하지 않는다.

## IPC 호환성

기존 v3 프레임과 요청/응답 길이를 유지한다. 추가 명령:

- 8: 설정 조회. RDR/FCS ID=0, TargetId=그룹(0=RDR 영점, 1=FCS 영점, 2=거리).
  성공 응답의 오프셋 20/24/28은 세 float의 IEEE754 비트다. 이 명령에서는
  기존 FcsId/AssignedTargetId/AssignmentState 의미로 읽지 않는다.
- 9: 같은 그룹의 세 값을 요청 PositionX/Y/Z로 저장. 기존 결과코드 사용.
- 5: 기존 설치 위치 저장 명령 유지.

숫자가 아닌 JSON 값, NaN/Infinity, float 범위 초과, 음수 거리,
RDR 최소보다 작은 최대를 거부한다. 음수 영점/설치 좌표는 허용한다.
임시파일 쓰기와 교체 성공 후에만 메모리 설정을 변경한다.

## 검증

```powershell
python ai-generated-c2/C2.Tests/Integration/test_camera_settings.py
python ai-generated-c2/C2.Tests/Integration/test_installation_persistence.py
dotnet run --project camera-settings-check/CameraSettingsCheck.csproj -c Release
```

통합 테스트는 loopback에서 실행하며 실제 장비를 사용하지 않는다.
GUI 검증은 WPF 컨트롤의 입력 거부·저장 실패·성공 갱신·범례와 렌더링을 확인한다.
화면 결과는 `camera-settings-check/preview.png`에 생성한다.

### 실행 결과 (2026-11-12)

- ECS Release 빌드 성공.
- GUI Release 빌드 및 WPF 검증 성공.
- 카메라 설정 통합 테스트 성공.
- 기존 설치 위치 통합 테스트 최종 재실행 성공. 이전 실행 1회에서
  `runtime_stop graceful=1` 출력 후 프로세스 종료 검사가 실패했다.
  간헐적 종료 실패의 원인은 이번 작업에서 확정하지 않았다.
- 실제 장비 연결 검증은 수행하지 않았다.

## 2026-11-13 변경

실제 GUI 작업 위치는 `C:/Users/user/Documents/projects/c2-GUI`다.
RDR 수평/수직 시야각을 `installation.camera.rdr_horizontal_fov_deg`와
`rdr_vertical_fov_deg`로 ECS에 저장한다. 기본값은 120/45도다.
IPC 그룹 3은 [수평, 수직, 0]이며 수평 (0,360], 수직 (0,180] 범위를 검증한다.
노란 음영은 FCS 최대 shoot 거리 안쪽에 표시하며 색상 설명 범례는 제거했다.
GUI 명칭은 shoot로 통일했다. 수직값은 표시에 사용하며 지도는 XY 평면도다.
