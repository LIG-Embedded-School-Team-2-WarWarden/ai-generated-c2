# 팀 ICD 전환 기록

기준: [메인 ICD v0.21 / page version 33](https://ligproject2team.atlassian.net/wiki/spaces/MFS/pages/360475/03+ICD),
[처리 규칙 version 7](https://ligproject2team.atlassian.net/wiki/spaces/MFS/pages/29917185/ICD).

공용 저장소에서 기존 v3 스키마만 확인되어, 사용자 승인에 따라 공용 submodule 작업본에
`proto/mfs/icd/v1/mfs.proto`와 공식 생성 Python 바인딩을 추가했습니다.
필드/enum 번호는 이 공용 파일 한 곳에서 관리하며 공용 저장소에 게시합니다.
통제소의 submodule이 정확한 커밋을 고정합니다. 보드 펌웨어 적용·실물 검증 완료를 뜻하지는 않습니다.

| 이전 MVP | 현재 콘솔 |
| --- | --- |
| private RawEnvelope field 100·capability bit 4 | 공용 POINT_CLOUD_CHUNK·CAPABILITY_REPORT |
| 수동 raw bytes 직렬화 | 공식 Protobuf 생성 코드 |
| 숫자 장치 발급 접속 세션 | C2 발급 UUID, REGISTER_REQUEST/RESULT |
| 고정 collection | MEASUREMENT_COMMAND/RESULT로 C2 수집 세션 발급 |
| stationary/임의 UTC flag | OBS_POSE·취득 구간·SDK 시각/매핑/오차 검증 |
| EffectorTurretCommand | CONTROL_OPEN 확인 후 POINTING_COMMAND |
| AttackCommand STOP | 같은 controlSession/epoch, 증가한 commandSequence의 STOP |
| 10Hz 업데이트 | 최대 5Hz, 이전 명령 최종 결과 확인 |
| timestamp 기반 상태 갱신 | uint64 Heartbeat/sampleSequence 독립 중복·역순 제거 |
| GUI와 공유한 구형 runtime | 독립 team_mvp 콘솔, C++ LiDAR Detector 재사용 |

로컬 proto/raw_cloud.proto, raw_cloud_wire C++ 코덱, 이전 raw 송신 스크립트,
C2 폴더에 있던 보드용 raw SDK 어댑터 및 해당 구형 테스트를 제거했습니다.
기존 v3 런타임은 legacy 회귀 대상으로 남기되 기본 빌드 OFF이며 새 콘솔과 연결하지 않습니다.

공용 schema는 MVP에 필요한 **장치 UDP 부분집합**입니다. GUI IPC, OBS 이동 추적,
전체 OBS_MODE_STATUS, 비상 래치 해제, 저장/재생까지 ICD 전체를 구현한 것은 아닙니다.
새 콘솔은 표시용 포인터의 방향 제어만 제공하며 레이저 출력/점등 명령을 추가하지 않습니다.
수동 point 명령도 새 제어 세션 경로로 유지하며, raw→검출→aim→stop 시연을 우선 검증합니다.

게시 순서는 공용 protobuf 변경 커밋/push → C2 submodule 참조 갱신 → C2 변경 커밋/push입니다.
공용 proto를 먼저 push한 뒤 통제소 submodule 포인터와 구현을 함께 게시합니다.
이후 `git clone --recurse-submodules`로 동일한 스키마와 통제소 코드를 가져올 수 있습니다.
보드 담당자는 새 공용 proto 커밋을 고정하고 생성 코드를 교체해야 상호 운용할 수 있습니다.

실행/제약/시험 방법은 [콘솔 안내](console-mvp.md), schema 매핑은
[공용 TEAM_ICD.md](../external/warwarden-protocol/TEAM_ICD.md)를 참고하세요.

## 게시 전 검증 (2026-10-20)

- 공용 proto 고정 커밋: `935fc4ffd23c853d6688edeb4a01d9abc30641bf`.
- 공용 저장소 GitHub CI: schema lint·생성 코드 일치 검사·계약 3건·C++ 시험 통과.
- Windows Release 빌드와 CTest 5개 통과.
- 새 ICD 통합 시험 15개: 실제 UDP·C++ 검출기·독립 콘솔/OBS/EFF 프로세스 포함.
- 시연 경로: 등록→설치→수집→원본 점군→검출→aim 연속 지향→stop 확인→수집 중지/재시작→quit.
- 오류 경로: 중복·역순, 불완전/상충 점군, 대상 소실, 취득시각 매핑 부재, ACCEPTED만 수신,
  STOP 결과 유실, OBS 재시작, 설치 revision 변경 시 추가 지향 차단.
- 기존 다중 자산 UDP smoke 시험 통과.

실제 OBS 보드·센서·모터 연결 시험은 이 더미 검증에 포함되지 않습니다.
