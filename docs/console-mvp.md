# 현재 C2 콘솔 사용법

실행 파일은 `x64/Release/C2.Console.exe`입니다.
Visual Studio의 `ai-generated-c2.sln`에서 C2.Console을 시작 프로젝트로 선택할 수도 있습니다.
실장비 설정은 `configs/rdr-live.json`입니다.

| 명령 | 동작 |
|---|---|
| rdr connect | RDR 연결 요청 |
| rdr status / rdr targets | RDR 상태 / 표적 목록 |
| rdr report on / rdr report off | 보고 허용 / 해제 |
| rdr start / rdr stop | 감시 시작 / 정지 |
| rdr exclude ID / rdr resume ID | 표적 ID 제외 / 복귀 |
| fcs connect / fcs status | FCS 연결 / 상태 조회 |
| fcs point ELEVATION AZIMUTH | 고각·모터 방위각 순서로 지향 |
| fcs laser on / fcs laser off | EF04 ON / OFF |
| quit | 정상 종료 |

`fcs point 20.5 180.2`처럼 도 단위, 0.1도 간격으로 입력합니다.
구동 한계는 설정의 `fcs.limits`로 검사합니다.
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
