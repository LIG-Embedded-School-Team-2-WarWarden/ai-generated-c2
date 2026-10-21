# 요구사항에서 테스트와 PR까지의 개발 절차

## 적용 범위

기존 프로젝트 전체가 TDD로 개발되었다고 소급해 주장하지 않습니다. 이번 구조 개선에서는 테스트를 먼저 실패시키고
구현으로 통과시키는 증분을 별도 커밋에 남겼습니다. 기존 동작을 옮길 때 추가한 특성화 테스트는 TDD와 구분합니다.

## 실제 Red → Green → Refactor 기록

| 단계 | 커밋 | 내용과 확인 |
| --- | --- | --- |
| Red | `f1dc048` | 시계/전송 포트, 명령 수명주기, OBS 전이 계약을 먼저 작성. 미구현 모듈 import에서 실패 확인 |
| Green | `19cbdd4` | 포트·재시도 관리자·상태 모델 구현. 새 11개와 기존 UDP 17개 통과 |
| Red | `c2123be` | 전송 뒤 호출자가 메시지를 바꾸면 정상 완료 응답을 놓치는 경우를 재현. `0 != 1`로 의도한 동작 실패 확인 |
| Green | `a934e0a` | 전송 전에 Protobuf 명령을 복사해 상관관계와 wire snapshot이 같게 유지. 해당 테스트 통과 |
| Refactor | 후속 구조 개선 커밋 | OBS/EFF·장치 세션·관측 정책 분리. 시간 제어형 workflow 시험과 기존 연동 시험 유지 |

의미 있는 Red 예시는 `RequestTests.test_caller_mutation_cannot_change_inflight_correlation`입니다.
전송 요청 ID는 같지만 호출자의 command_sequence가 바뀌는 상황에서 원래 순번의 완료 응답을 수락해야 한다는 요구입니다.

```sh
git show c2123be -- tests/test_design.py
git diff c2123be a934e0a -- team_mvp/requests.py
python tests/test_design.py RequestTests.test_caller_mutation_cannot_change_inflight_correlation
```

현재 HEAD에서는 마지막 명령이 통과합니다. 과거 커밋에 있는 실패와 현재 통과를 구분해 설명합니다.

## 요구사항과 검증의 연결

| 요구 ID | 수용 기준 | 대표 검증 |
| --- | --- | --- |
| REQ-CMD-01 | 재전송은 처음 전송한 바이트와 같아야 함 | RequestTests.test_retry_preserves_exact_bytes_and_stops_after_budget |
| REQ-CMD-02 | ACCEPTED만으로 완료하지 않음 | RequestTests.test_accepted_suppresses_retry_but_does_not_complete |
| REQ-CMD-03 | 세션·epoch·순번이 다른 결과는 완료에 반영하지 않음 | RequestTests.test_wrong_sequence_or_connection_cannot_complete |
| REQ-CMD-04 | 취소한 명령은 재전송/지연 완료 콜백이 없어야 함 | RequestTests.test_cancelled_point_never_retries_or_calls_late_callback |
| REQ-OBS-01 | EFF 정지·수집 정지·후속 실측 정지 전 수집을 재개하지 않음 | ObserverWorkflowTests.test_collection_waits_for_result_and_fresh_stopped_feedback |
| REQ-OBS-02 | 이전 이동의 지연 완료가 STOP을 취소하지 않음 | ObserverWorkflowTests.test_stop_ignores_late_completion_of_replaced_move |
| REQ-OBS-03 | 이동 미확인 시 높은 순번의 STOP 후 수집 재개 금지 | ObserverWorkflowTests.test_lost_completion_issues_stop_without_resuming_collection |
| REQ-POINT-01 | 계산 구현은 교체하되 장치 한계 검사는 유지 | PointingStrategyTests.test_replacement_solver_cannot_bypass_reported_limits |
| REQ-TIME-01 | 벽시계 보정이 재전송 deadline을 진행시키지 않음 | RequestTests.test_wall_clock_adjustment_does_not_advance_retry_deadline |
| REQ-FRESH-01 | 중복 상태/Heartbeat가 최신성을 연장하지 않음 | DeviceFreshnessTests |

## 이후 변경 절차

1. 문제의 입력과 기대 결과를 수용 기준으로 작성하고 기존 요구 ID 또는 새 ID에 연결합니다.
2. 가장 작은 경계에서 실패 테스트를 작성합니다. 실패 이유가 요구한 동작인지 확인합니다.
3. 최소 구현으로 통과시킵니다. 실제 장치 없이 판단 가능한 시간/상태 규칙은 가짜 시계로 시험합니다.
4. 책임 중복·경계 위반을 리팩터링합니다. 패턴은 실제 교체점과 변경 이유가 있을 때 적용합니다.
5. 관련 UDP/native 통합 시험과 CI를 실행합니다. 실패가 남으면 통과로 보고하지 않습니다.
6. PR에 문제·변경 동작·설계 이유·검증 결과·장비 검증 범위를 적습니다.

이 순서는 모든 단순 문서 수정마다 복잡한 테스트를 추가하라는 뜻이 아닙니다.
프로토콜·시간·상태 전이처럼 회귀 위험이 있는 동작 변경에 집중합니다.

## 실행

```sh
python -m pip install -r requirements-dev.txt
python tests/test_design.py
python -m black --check --workers 1 team_mvp tests/test_design.py
ctest --test-dir build -C Release --output-on-failure
```

CMake에 domain_design 시험을 등록했고 Windows/Linux CI에서 형식 검사와 기존 회귀 시험을 실행합니다.
Native device integration workflow는 고정 firmware 커밋과 실제 C++ 검출기를 사용합니다.
설치·축 보정·실센서 시각 증빙·실물 정지 확인은 별도 장비 수용 시험으로 남습니다.
