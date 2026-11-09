# 현재 C2 콘솔 사용법

실행 파일은 `x64/Release/C2.Console.exe`입니다.
Visual Studio의 `ai-generated-c2.sln`에서 C2.Console을 시작 프로젝트로 선택할 수도 있습니다.
실장비 설정은 `configs/rdr-live.json`입니다.

| 명령 | 동작 |
|---|---|
| rdr connect | RDR 연결 요청 |
| rdr status / rdr targets | RDR 상태 / 객체 목록 |
| rdr live on / rdr live off | 최신 수신값을 월드 XYZ로 변환한 읽기 전용 진단 출력 시작 / 종료 |
| rdr report on / rdr report off | 보고 허용 / 해제 |
| rdr start / rdr stop | 감시 시작 / 정지 |
| rdr exclude ID / rdr resume ID | 객체 ID 제외 / 복귀 |
| fcs connect / fcs status | FCS 연결 / 상태 조회 |
| fcs point ELEVATION AZIMUTH | 고각·모터 방위각 순서로 지향 |
| fcs laser on / fcs laser off | EF04 ON / OFF |
| quit | 정상 종료 |

`rdr live on`은 현재 수신 상태를 200ms 간격으로 읽어 `type: "rdr_live"` JSON을 출력합니다. 각 객체의 `world_xyz`는 최신 원본 XYZ·관측 PAN/TILT와 RDR 설치 위치로 매번 계산한 동쪽·북쪽·위쪽 좌표(미터)입니다. 같은 ID의 위치가 달라져도 최초 값에 고정하지 않습니다. `raw_xyz`는 원본 수신값이며 `timestamp_ms`는 출력 시각입니다. 200ms는 조회 주기이며 센서 수신 주기를 의미하지 않습니다. 좌표·자세가 무효이거나 감시/보고가 중지되면 `world_valid: false`, `world_xyz: null`로 출력합니다. 이 명령은 센서 구동을 시작하거나 제어 명령을 보내지 않습니다. GUI 목록은 이 명령 없이도 최신 월드 XYZ로 갱신됩니다.

`rdr targets`의 `gui_xyz`는 GUI에 전달하는 칼만 필터 적용 후 월드 XYZ입니다(필터를 끄면 최신 변환값). 해당 명령의 `world_xyz`는 필터 적용 전 최신 월드 좌표이며, FCS 객체 할당은 `gui_xyz`와 같은 필터 결과를 사용합니다. 필터 적용 전의 최신 월드 좌표를 연속 비교할 때는 `rdr live on`을 사용합니다. [GUI 좌표 필터 설정](operator-ipc.md#gui-좌표-필터)을 참고하세요.

`fcs point 20.5 180.2`처럼 도 단위, 0.1도 간격으로 입력합니다.
ECS의 자산별 팬·틸트 구동 한계 검사는 제거했습니다. 기존 `fcs.limits`는 무시하며 음수 틸트도 전송합니다. 0.1도 단위·signed int16 전송 형식은 검증합니다.
confirmed는 ICD 응답 수신이며 목표 각도 도달을 뜻하지 않습니다.
후속 `fcs status`의 실측값으로 확인합니다.

`obs`, `eff`, `collect start`, `assets`는 현재 명령이 아닙니다.
잘못된 명령·인수 개수에는 `invalid command or argument count`가 반환됩니다.
수동 레이저 동작은 [FCS 동작](laser-control.md)을 참고하세요.

## 모의기

빌드된 C2와 WPF가 있는 상태에서 실행합니다.

```powershell
python .\simulators\run.py --gui ..\c2-GUI\build\Release\AadcCommandCenter.exe
```

실행기가 장치 모의기·C2·제어창과 지정한 WPF를 함께 실행합니다.
기존 실장비 C2를 먼저 종료해야 합니다. [모의기 안내](simulator.md)를 참고하세요.

## 실행 및 할당 실패 로그

기본 실행도 작업 디렉터리의 `logs/events.log`에 이벤트를 누적 기록합니다. `--status-file`을 지정하면 기존처럼 해당 파일 옆 `events.log`를 사용하며, `--event-log FILE`로 경로를 명시하면 가장 우선합니다. 로그 디렉터리는 자동 생성하며 파일을 열 수 없으면 시작 오류로 알립니다.

시작 기록에는 실행 파일 절대 경로, 빌드 시각, PID, 필터 설정과 로그 경로를 남깁니다. 상태 변경과 GUI 명령 결과에는 숫자 결과뿐 아니라 `reason_name`과 GUI 호환 변환 전 `internal_result`를 남깁니다. `detail` JSON의 현재/기준 필터 좌표, 차이, 축별 허용 범위, 연속 범위 초과 수신 횟수, 장치 상태·각도·상태 나이, RDR 세션 정보로 좌표 변화·소실·통신·장치·정렬 문제를 구분할 수 있습니다. 정상 종료는 `runtime_stop graceful=1`로 기록합니다.
