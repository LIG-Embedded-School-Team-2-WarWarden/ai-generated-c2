# WPF 모의기 검증 기록 — 2026-11-04~05

GUI 기준: c2-GUI PR #5/#6, `94bca8c5c477e4cf1bb15b93c288a5c450603910`.
소스 위치: `../c2-GUI`. 모의기 실행 방법은 [simulator.md](simulator.md)에 있습니다.

기존 화면 검증 당시에는 GUI 파이프 이름을 임시 변경했습니다. 해당 변경은 철회하고 원본으로 복구했습니다.
현재 실행기는 원본 GUI의 기본 IPC를 사용합니다. 아래 스크린샷은 이전 검증 기록이며 원복 후 재촬영한 결과가 아닙니다.

## 완료한 검증

| 항목 | 결과와 근거 |
|---|---|
| 최신 WPF Release 빌드 | 통과, 경고0·오류0 |
| C2 + RDR/FCS 모의기 실행 | 통과, 양쪽 연결·FCS 실측각 상태 수신 |
| 기존 통제소와 동시 실행 | 현재 원본 WPF 실행은 기본 IPC를 사용하므로 지원하지 않음. 다른 통제소를 종료한 뒤 실행 |
| GUI v3 프레임 | 실제 Windows 공유 메모리, 492바이트·버전·체크섬·float 오프셋 통과 |
| GUI v3 파이프 | 실제 Windows Named Pipe, 40바이트 요청/응답과 requestId/command 매칭 통과 |
| 각도 | RDR/FCS 0.1도 변환, float 소수 표시용 값 검증 통과 |
| 좌표 편집 | RDR/FCS XYZ, 프레임 반영과 표적 좌표 이동 검증 통과 |
| 구동/정지 | 보고 확인→감시 시작, 정지 직후 측정값 무효화 검증 통과 |
| 표적 | 최대10개, 소실/복구, 제외/복귀 검증 통과 |
| 할당/모의 동작/취소 | 모의 FCS 목표 도달 후 완료, 상태 비트와 응답 검증 통과 |
| 결함/연결 끊김 | FCS 결함·응답 단절, RDR 단절·재연결, 감시 자동 재개 없음 검증 통과 |
| WPF 창 생성 | `AADC-9 | 대공방어 통제 시스템` 창 확인 |

자동 시험: `scripts/test.ps1 -Configuration Release`.
이 결과는 데이터·통신 검증이며 실제 WPF 화면의 시각 검증을 대신하지 않습니다.

## 실제 WPF 시각 검증

Windows 잠금 해제 후 실제 WPF 창에서 아래 항목을 조작하고 스크린샷을 기록했습니다.
검증한 화면 크기는 약 1486×843과 최대화 1920×1032이며, 배율100%/90% 및 패널 분할선 조절을 확인했습니다.
검증 범위에서 화면 겹침·렌더링 오류·앱 종료는 발생하지 않았습니다.

| 확인 대상 | 확인할 내용 | 상태 |
|---|---|---|
| 초기 화면 | 연결 상태, 정지 상태, 빈 표적·할당 안내, 파이프 연결 안내 | 통과 |
| 감시 자산 패널 | 구동/정지, 방위각 주사, 고각 소수, 상태 색상 | 통과 |
| 레이더 | RDR/FCS 위치·방향과 표적10개, 범위 표시 | 통과 |
| 표적 목록 | ID·각도·거리·XYZ·FCS 선택 표시·할당 버튼 | 통과 |
| 할당 정보 | 조준 접수·할당 완료·모의 동작 완료·취소 | 통과 |
| FCS 패널 | 실측 소수 각도, 연결·구동·결함·모의 레이저 ON/OFF | 통과 |
| 위치 편집 | RDR X=1.25, FCS Z=1.5 입력·적용, 로그·지도·표적 좌표 반영 | 통과 |
| 고각 편집 | 현재값12.3 표시, 슬라이더 조작 후 미지원 로그와 실측값 복귀 | 통과(장치 제어 미지원) |
| 이벤트 로그 | 명령·상태 변화 기록, event-log.txt 메모장 창 생성 | 통과 |
| 연결 끊김·복구 | 표적 소실, FCS 결함·단절, RDR 단절 후 재연결·정지 유지 | 통과 |
| 화면 크기 | 100%/90%, 최대화/일반 창, 분할선 드래그 | 통과(아래 표시 범위 참고) |

1486px 폭에서 100% 배율의 할당 표는 오른쪽 열에 가로 스크롤이 생깁니다.
90% 배율 또는 1920px 최대화 100%에서는 모든 열을 동시에 확인했습니다.
모든 모니터 배율과 최소 창 크기의 조합까지 검증했다는 의미는 아닙니다.

## 스크린샷

스크린샷은 실제 창 캡처이며 `logs/wpf-verification`에 보관했습니다.

| 장면 | 파일 |
|---|---|
| 최종 정상 구동 / 표적10개 / 기본 좌표 | [21-final-running.png](images/wpf/21-final-running.png) |
| 초기 정지 화면 | [01-initial.png](images/wpf/01-initial.png) |
| 할당 완료 / 모든 열 표시 | [04-assignment-90percent.png](images/wpf/04-assignment-90percent.png) |
| 모의 동작 ON / 완료 | [05-simulated-output.png](images/wpf/05-simulated-output.png), [06-completed.png](images/wpf/06-completed.png) |
| RDR/FCS 위치 편집 | [08-position-editors.png](images/wpf/08-position-editors.png) |
| 표적 소실 | [10-target-loss.png](images/wpf/10-target-loss.png) |
| FCS 결함 / 명령 거부 안내 | [11-fcs-fault.png](images/wpf/11-fcs-fault.png), [12-unavailable-dialog.png](images/wpf/12-unavailable-dialog.png) |
| FCS / RDR 단절 | [13-fcs-disconnected.png](images/wpf/13-fcs-disconnected.png), [14-rdr-disconnected.png](images/wpf/14-rdr-disconnected.png) |
| 재연결 후 정지 유지 | [15-reconnected-idle.png](images/wpf/15-reconnected-idle.png) |
| 수동 정지 / 분할선 조절 | [18-stopped.png](images/wpf/18-stopped.png), [19-splitter.png](images/wpf/19-splitter.png) |

검증 후 모의기를 다시 실행해 좌표 편집값을 초기화했고, WPF를 정상 구동 화면으로 유지했습니다.

알려진 기능 범위: 현재 보드에는 고각 설정 메시지가 없어 GUI 고각 설정은 NotSupported 응답입니다.
이는 화면 오류가 아니라 장치 프로토콜의 미지원 기능입니다.

## 2026-11-05 위치 변경 후 FCS 방향 추가 검증

추가 확인 결과, 기존 구현은 GUI 위치를 지도·표적 표시에는 반영했지만 FCS 할당 계산은 RDR 원점의
측정값만 사용했습니다. 위치가 다른 두 장비를 같은 원점으로 계산하던 부분을 수정했습니다.

변경한 설치 위치를 지도와 FCS 상대 벡터 계산에 함께 사용하며, 각도를 0.1도 단위로 보존합니다.
위치 변경 시 기존 할당의 정렬 판정을 무효화하고 재할당을 기다립니다. 동일 좌표의 재적용은 할당을 유지합니다.
영점·설치 방향 기준과 기존 장치 limits는 유지하며 회전축/장착 방향 보정을 새로 도입하지 않았습니다.

| 모의 조건 | 확인 결과 |
|---|---|
| RDR만 X 방향 이동 | FCS 명령 각도 변경 |
| FCS만 X 방향 이동 | FCS 명령 각도 변경 |
| RDR/FCS를 서로 다른 XYZ로 이동 | 변경된 상대 좌표에 따른 각도 확인 |
| RDR/FCS를 같은 XYZ만큼 평행 이동 | 상대 각도 유지 |
| 할당 완료 후 위치 변경 | 기존 할당 무효화, 재할당 전 동작 거부 |
| 동일 위치 재적용 | 기존 할당 유지 |
| FCS 바로 위의 표적 / 장치 고각 한계 밖 | 명령 거부 유지 |

기존 실행 파일은 새 회귀 시험의 RDR 위치 변경 사례에서 잘못된 각도를 반환해 실패했고,
수정 후 전체 `scripts/test.ps1 -Configuration Release`가 통과했습니다.
Windows 상태 파일 교체가 일시적인 읽기 잠금과 충돌할 때 모의기가 종료되는 문제도 재시도로 보완했습니다.

WPF에서도 RDR=(0,0,0), FCS=(0.6,0,0)로 변경하고 모의 표적1001을 할당해
모의 FCS 실측 방위각215.5°/고각5.8°, GUI 방위각99.5°/고각5.8°를 확인했습니다.
목록의 표적 방위각108.4°는 RDR 기준이므로 FCS 기준의99.5°와 구분합니다.
[위치 반영 검증 화면](images/wpf/22-position-aware-fcs.png).
