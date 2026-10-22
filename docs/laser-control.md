# 교육용 저출력 레이저 제어

공용 ICD 1.0의 `LaserCommand`/`LaserResult`/`LaserStatus`를 사용합니다.
등록 요청의 `CAPABILITY_LASER_CONTROL`과 기능 보고의 `laser_control_supported=true`가 모두 필요합니다.
기존 EFF와 Python 더미는 지원 기능을 알리지 않으면 레이저 명령을 받지 않습니다.

```text
point 201 0 0
status
laser on 201 1000
laser status 201
status
laser off 201
stop 201
```

`point`의 `POINT_COMPLETED`와 실측 정지를 확인한 뒤 ON을 입력합니다. ON 시간은 정수 1~5000ms이며
장치가 출력을 켠 시점부터 단조 시계로 계산합니다. 명령 수락 기한은 발행 후 1초입니다.
`aim` 제어 세션도 사용할 수 있지만 ON 시점에는 장치의 실측 정지가 확인되어야 합니다.
방향 제어가 자동으로 레이저를 켜지는 않습니다.

레이저 순번은 지향 순번과 독립적이며 같은 제어 세션/epoch에 묶입니다. OFF와 STOP은 진행 중 ON
재전송을 취소합니다. 결과는 요청 ID·접속 세션·제어 세션·epoch·순번을 모두 대조합니다.
시간 초과 또는 확인 불가능한 ON 완료 응답에는 OFF/STOP을 보냅니다.

`status.lasers`의 `state`는 관측 시각이 있는 최신 장치 상태만 표시합니다.
명령 송신·ACCEPTED·타이머 계산으로 ON/OFF를 추정하지 않습니다. 오래되거나 fault가 있으면 UNKNOWN입니다.
firmware 실물 backend의 피드백은 Linux GPIO 출력-enable line 읽기이며, 광센서로 광출력을 측정한 값이 아닙니다.

EFF 시뮬레이터는 `mfs_eff --simulate --c2-port 5000`을 사용합니다. 실물은 기존 UART/축 보정/시각 설정과
`--laser-gpiochip /dev/gpiochipN --laser-line N --laser-estop-line M`을 명시해야 지원 기능을 알립니다.
출력 active-high, 비상 입력 active-high이며 두 line은 같은 chip의 서로 다른 offset입니다.
비상 입력 또는 입력 읽기 오류는 소등·래치 후 ON/이동을 차단하며 재시작 전 해제하지 않습니다.
OBS에는 레이저 출력을 연결하지 않습니다. 실물 광출력·배선·비상입력·부팅/프로세스 강제 종료 시 GPIO 상태는 장비 검증이 필요합니다.

firmware의 `tests/test_laser.py --eff ... --obs ... --console ... --config configs/team-demo.json`은
실제 C++ 프로세스끼리 CLI→UDP→EFF→결과/상태, 만료·중복·충돌·STOP/DRAIN·세션 상실/교체를 검증합니다.
