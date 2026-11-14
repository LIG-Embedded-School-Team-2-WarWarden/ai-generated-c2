# 시퀀스 세분화 전 / 후 복원

이 작업은 기존 작업 폴더의 미커밋 변경이나 인덱스를 초기화하지 않았다.

| 저장소 | 작업 전 태그 | 의미 |
|---|---|---|
| ai-generated-c2 | 시퀀스-세분화-전-2026-11-14 | 최신 main cd1e474: ECS 수동 단계 구현 전 |
| c2-GUI | 시퀀스-세분화-전-2026-11-14 | 030c3b0: GUI는 이미 단계 화면 적용, ECS 연동 작업 전 |
| c2 | 시퀀스-세분화-전-2026-11-14 | 이전 소스·설계·문서 및 작업 폴더별 백업 목록 |

작업 후 태그는 각 저장소의 `시퀀스-세분화-후-2026-11-14`이다. GUI 소스는 이번 작업에서 수정하지 않아 GUI의 두 태그는 같은 커밋이다. ECS 신규 기능은 `codex/manual-sequence-20261114` 브랜치에 있다.

## 기존 파일을 건드리지 않고 작업 전 ECS 실행 준비

```powershell
git fetch origin --tags
git worktree add --detach ../ecs-before-sequence '시퀀스-세분화-전-2026-11-14'
pwsh -NoProfile -File ../ecs-before-sequence/scripts/build.ps1 -Configuration Release
```

작업 후 상태를 별도 폴더에 열려면 태그를 `시퀀스-세분화-후-2026-11-14`로 바꾸고 다른 새 폴더 이름을 사용한다. 실제 장비 실행 전 사용 중인 프로세스를 종료하고 장비 연결 설정을 확인한다. 이 문서의 명령은 소스 체크아웃과 빌드만 수행한다.

## 이전 미커밋 작업 복원

`backup-checkpoints.json`의 repository/branch/commit을 확인하고 해당 저장소에서 다음과 같이 새 폴더를 만든다.

```powershell
git fetch origin
git worktree add --detach ../recovered-work origin/codex/checkpoint-before-sequence-20261114-ai-generated-c2
```

개별 변경은 각각의 백업 브랜치에 보존하며 서로 다른 미완성 구현을 임의로 합치지 않았다. 로컬의 브라우저 프로필·캐시·빌드 실행파일/중간물·외부 의존성은 GitHub 소스 백업에서 제외했다. 루트에 이미 스테이징돼 있으나 현재 작업 트리에는 없는 과거 소스/자료도 별도 인덱스로 읽어 보존했다. 원래 인덱스는 수정하지 않았다.

## Confluence 원복

| 페이지 ID | 작업 전 페이지 버전 | 이번 갱신 버전 |
|---|---:|---:|
| 37158926 GUI–ECS | 5 | 7 |
| 37355617 RDR–ECS | 15 | 17 |
| 31358993 FCS–ECS | 10 | 12 |

Confluence 페이지 기록에서 작업 전 버전으로 복원할 수 있다. `icd-snapshots/before-페이지ID.html`과 `after-페이지ID.html`에 본문 사본도 보관했다. 이후 다른 사람이 추가 수정했다면 페이지 전체 복원 전에 변경분을 비교한다.

## 검증 범위

자동 시험은 localhost 모의 RDR/FCS와 Windows Named Pipe/SharedMemory를 사용한다. 실제 장비에 명령을 전송하거나 펌웨어를 배포하지 않았다. 기존 장비 ICD 명령과 바이트 크기는 유지한다. 결과 미확정은 HIT로 바꾸지 않으며, 실제 결과의 수동 기록 방법과 제한은 manual-sequence.md를 참고한다.
