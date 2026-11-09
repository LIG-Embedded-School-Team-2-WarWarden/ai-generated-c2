# 라이다 GUI 좌표 흔들림 기록

동일 수신 샘플의 필터 전 월드 XYZ와 GUI에 게시하는 필터 후 XYZ를 CSV에 함께 저장한다.
기록 옵션이 없는 실행에는 파일을 만들지 않는다. 기록 중에도 필터 설정과 장치 제어 동작은 동일하다.

기존 ECS를 종료한 상태에서 기록 기능이 포함된 실행 파일을 사용한다. 동시에 두 ECS를 실장비에 연결하지 않는다.

```powershell
.\x64\Release\C2.Console.exe --config configs/rdr-live.json --rdr-record logs/coordinates.csv --rdr-record-seconds 30
```

실행 후 GUI에서 라이다를 구동하면 첫 유효 객체 수신부터 30초 동안 기록한다. 대상 객체는 정지 상태를 유지한다. 기록이 끝나도 ECS는 계속 실행된다. CSV 또는 동명 `.csv.json`이 존재하면 덮어쓰지 않고 실행을 거부한다.

- `before_x/y/z`: 해당 관측의 PAN/TILT로 변환한 최신 월드 좌표(m). ID별로 고정된 명령용 좌표가 아니다.
- `after_x/y/z`: 같은 GUI 갱신에서 게시하는 필터 출력(float32 정밀도, 표시용 소수점 반올림 전).
- `received_s`: ECS의 단조 증가 수신 시간. 센서 자체의 관측 시각은 프로토콜에 없다.
- `track_state`: 1 OBSERVED, 2 PREDICTED, 0 LOST. 유효하지 않은 행의 비교 좌표는 비워둔다.
- 원본 센서 XYZ, 관측 PAN/TILT, 설치 원점, 세션 generation, 수집 시각도 함께 보존한다.

GUI 게시 루프에서 소비한 최신 샘플을 기록하며 같은 수신 시각의 반복 표시는 제외한다. 모든 UDP 패킷을 캡처하는 기능은 아니므로 GUI보다 빠른 수신에서는 중간 패킷이 생략될 수 있다. 상태 변화에 따른 무효 행은 별도로 남긴다.

동명 `.csv.json`에 필터 설정, 라이다 peer, 실행 파일과 `waiting_for_valid_target`, `recording`, `completed`, `interrupted`, `failed` 상태를 저장한다. 완료 전 종료하거나 쓰기에 실패하면 정상 측정 완료로 해석하지 않는다.

```powershell
python scripts/analyze_rdr_recording.py logs/coordinates.csv --warmup-seconds 2
```

분석기는 소실·수신 간격 3초 초과·세션/원점 변경으로 구간을 나누고 각 구간의 첫 2초를 제외한다. 동일 객체의 유효 전후 쌍만으로 XYZ의 모집단 표준편차와 최대–최소 폭, 각 스트림 평균 중심의 3D RMS 및 감소율을 계산한다. 전체, OBSERVED, PREDICTED 결과를 분리하며 실제 잡음 감소 평가는 OBSERVED 결과를 우선 사용한다. 평균 위치 변화도 별도 기록한다. 실제 정답 위치가 없으므로 이 결과는 좌표의 흔들림이며 절대 위치 정확도가 아니다.

## 기존 기록으로 설정 비교

```powershell
python scripts/tune_gui_position_filter.py logs/coordinates.csv --tests x64/Release/C2.Tests.exe
python scripts/tune_gui_position_filter.py logs/coordinates.csv --tests x64/Release/C2.Tests.exe --per-axis
```

이 도구는 장치에 연결하지 않고 실제 C++ GUI 필터로 같은 월드 좌표를 재생한다. 기존 설정으로 기록된 출력과 재생 출력이 1µm 이내로 일치하는지 먼저 검사한다. 전반부 OBSERVED만으로 측정 잡음을 추정하고 과정 잡음 후보를 선택하며, 후반부는 별도 결과로 보고한다. 기본 선택 조건은 1Hz 합성 입력에서 5cm 변화의 90%를 8회 이내, 10/20cm 변화는 3회 이내에 반영하는 후보 중 전반부 흔들림이 가장 작은 값이다. PREDICTED도 실제 순서대로 필터에 넣되 측정 잡음 추정에는 사용하지 않는다.

`--per-axis`는 축별 측정 잡음과 과정 잡음을 독립적으로 비교한다. 전·후반 각각 객체별 OBSERVED 최소 15개가 필요하다(스칼라 모드는 5개). 소실·원점/세션 변경·3초 초과 수신 간격에서 구간을 나누며 재인식 후 첫 2초를 통계에서 제외한다. 무효 행과 PREDICTED도 재생 순서에 포함하므로 실제 필터 초기화와 상태 전이를 보존한다. 합성 이동 시험은 매 축에 대해 1/2cm 10회, 5cm 8회, 10/20cm 3회 이내의 90% 반영과 2cm/s 이동의 최대 지연 거리 6cm 이하를 요구한다. 정지 기록 전반부의 3D 흔들림으로 후보를 고른 뒤, 후반부에서 3D RMS가 개선되지 않거나 개별 축이 5% 넘게 나빠지면 기존 설정을 유지한다.

출력은 설정 후보와 응답 시간을 담은 `tuning-report.json`, 추천값 `selected-filter.json`, 같은 원본에 대한 기존/새 출력 `comparison.csv`다. 설정 파일을 자동으로 바꾸지 않는다. 재생 결과를 별도 실측 성능이나 절대 정확도로 해석하지 않는다.
