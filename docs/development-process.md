# C++ 통제소의 요구사항·TDD·검증

담당 범위는 통제소입니다. Python 제어부를 C++로 이전했고, Python은 더미 장치·블랙박스 시험에만 사용합니다.
이전 프로젝트 전체가 처음부터 TDD였다고 소급해 주장하지 않습니다.

## 실제 Red → Green 기록

| Red | Green | 실패와 수정 |
| --- | --- | --- |
| `6b987cd` | `8389949` | 먼저 C++ 요청 복사·재전송·전이 계약을 작성하고 네이티브 빌드/구현을 추가한 이관 단계. 초기에는 시험 target이 없어 실행 불가였으며, 동작 결함 재현 사례와 구분합니다. |
| `a50f520` | `9881f5d` | 사유 없는 FAILED 응답이 pending을 제거하는 결함. `failed result without reason must stay pending` 실패를 확인한 뒤 reason_code 검사를 복원했습니다. |
| `9cedd01` | `5004767` | Windows 경로의 역슬래시가 제거돼 clock-profile 파일을 열지 못하는 결함. 실제 콘솔 시험의 `cannot open evidence file` 실패 후 인자 파서를 수정했습니다. |

```sh
git show a50f520 -- tests/native_domain.cpp
git diff a50f520 9881f5d -- native/domain.cpp
git show 9cedd01 -- tests/test_native_mvp.py
git show 5004767 -- native/engine.cpp
```

후속 책임 분리에서 RequestTracker는 `native/requests.cpp`로 이동했습니다.
이전 Python 단계의 Red/Green 기록은 Git 이력에 남지만 현재 실행 구조의 근거로 혼용하지 않습니다.
기존 동작을 지키기 위한 회귀·특성화 테스트와, 새 실패를 먼저 작성한 TDD 증분을 구분합니다.

## 요구사항과 테스트

| 요구사항 | C++ 단위 시험 / 콘솔 더미 시험 |
| --- | --- |
| 재전송은 동일 바이트·최대 3회 | native_domain / retry_keeps_exact_packet_and_applies_once |
| ACCEPTED는 완료가 아님 | ACCEPTED suppresses retry / accepted_does_not_mean_completed |
| 세션·순번 불일치·취소 후 응답 무시 | wrong sequence and session / cancelled POINT / replaced connection |
| 상태·Heartbeat 중복은 최신성 갱신 금지 | duplicate and older status / heartbeat uses uint64 sequence |
| 계산 교체가 장치 한계를 우회하지 못함 | injected Strategy / replacement Strategy cannot bypass device limits |
| STOP 결과와 후속 실측 정지 확인 | STOP needs result / stop_result_required_for_confirmation |
| OBS 이동 전 정지·수집 중지 확인 | OBS waits for EFF stop / observer_move_stops_active_effector |
| 이동 완료 응답만으로 수집 재개 금지 | OBS motion completion alone cannot resume collection |
| 지연 응답으로 취소 작업 부활 금지 | late replaced OBS motion / terminal OBS operation |
| 검출·관측 소실 시 지향 중지 | empty_detection / incomplete_new_frame / conflicting_chunk |
| 시각 증빙·유효기간·세션 확인 | expired acquisition profile / clock evidence / clock_profile_and_foreign_session_evidence |

## 반복 가능한 실행

```sh
cmake -S . -B build/native -DC2_BUILD_LIVE=OFF
cmake --build build/native --config Release --parallel 2
ctest --test-dir build/native -C Release --output-on-failure
```

Python 더미 시험 의존성은 `requirements-team.txt`에 있습니다. C++ 단위 시험과 실제 C2 실행에는 Python이 필요 없습니다.
단위 시험은 Release에서도 동작하는 예외 기반 검사를 사용하며 assert 비활성화에 영향을 받지 않습니다.
Windows/Linux CI는 공용 proto 생성, C++ 빌드, 단위 시험, 실제 콘솔과 더미 간 UDP 시험을 수행합니다.
장치 내부 구현은 다른 담당 범위이므로 이번 C2 CI는 해당 저장소의 코드를 수정하거나 테스트를 유지보수하지 않습니다.

개발 절차는 요구사항 → 실패 테스트 → 최소 수정 → 책임별 리팩터링 → 단위·더미 회귀 → PR 검토로 진행합니다.
