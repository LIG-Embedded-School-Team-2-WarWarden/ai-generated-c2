# 외부 운영 화면 IPC v3

기준: [c2-GUI PR #5](https://github.com/LIG-Embedded-School-Team-2-WarWarden/c2-GUI/pull/5),
[PR #6](https://github.com/LIG-Embedded-School-Team-2-WarWarden/c2-GUI/pull/6),
GUI 커밋 `94bca8c5c477e4cf1bb15b93c288a5c450603910`.
GUI와 C2는 같은 Windows 사용자 세션에서 실행합니다.

| 항목 | 규격 |
|---|---|
| SharedMemory | `Local\ControlStationMock.SharedMemory.v3` |
| Mutex | `Local\ControlStationMock.SharedMemory.Mutex.v3` |
| Frame magic / version | `0x314D4354` / 3 |
| Frame / header / payload | 492 / 32 / 460 bytes |
| RDR / FCS / target | 40 / 40 / 36 bytes, Pack=4 |
| Pipe | `ECS_NamedPipe`, magic `0x50495045`, version 3 |
| Request / response | 40 / 40 bytes |

모든 필드는 little endian이며 float는 IEEE-754 binary32입니다. FNV-1a payload 체크섬을 사용합니다.
RDR/FCS ID는 1, 표적 최대 10개입니다. 결과0~6, 할당상태0~3의 GUI 계약을 유지합니다.
v2 프레임·요청은 호환되지 않습니다. `--ipc-suffix`는 모의 시험용 named object 구분자입니다.

## 프레임과 각도

RDR/FCS/표적의 방위각·고각은 도 단위 float이며 소수점 한 자리까지 표시합니다.
장치 RDR/FCS는 signed int16, 0.1도 단위입니다. ECS 수신 시 /10.0으로 변환합니다.
미상값 -32768은 변환 전에 판별합니다. XYZ는 장치 mm에서 m로 변환합니다.

RDR Base(right,up,forward)는 GUI(X,Y,Z)=(BaseX,BaseZ,BaseY)로 옮기고 RDR 표시 위치를 더합니다.
각도와 거리는 RDR 기준 측정값입니다. RDR 방위각은 135-pan, FCS는 모터180도를 GUI135도로 변환합니다.
FCS 표시 방위각은 360도 순환을 지원하며 실제 모터 구동 한계는 configs의 limits를 유지합니다.
RDR isOperating은 연결·보고 허용·ACTIVE가 확인된 경우에만 1입니다.
정지 요청 직후 측정값은 무효이며 미검출·제외·정지 표적은 GUI 목록에서 제거합니다.

## 파이프 요청

공통 offset: magic@0, version@4, command@8, requestId@12, rdrId@16, fcsId@20, targetId@24는 uint32입니다.
positionX@28, positionY@32, positionZ@36은 float(m)입니다. 응답은 requestId와 command를 보존합니다.

| 번호 | 명령 | 값과 처리 |
|---|---|---|
| 0 | GetStatus | 현재 상태 조회 |
| 1~3 | Assign / Cancel / Fire | 기존 계약 유지. 감시 정지 시 할당 불가 |
| 4 | SetRdrElevation | targetId=0~900, 0.1도 단위. 현재 ED/DE 보드는 명령이 없어 NotSupported |
| 5 | SetAssetPosition | RDR: rdrId=1/fcsId=0, FCS: rdrId=0/fcsId=1. 유한한 XYZ 필요 |
| 6 | SetRdrOperating | rdrId=fcsId=1, targetId=0 정지/1 구동 |

위치 설정은 현재 실행에서 보관하고 후속 프레임과 FCS 할당 계산에 함께 반영합니다.
GUI 좌표는 내부 설치 좌표 (forward,left,up)=(GUI Y,-GUI X,GUI Z)로 변환합니다.
표적의 공통 좌표는 RDR 위치 + RDR 기준 측정값이며, 여기서 FCS 위치를 빼서 FCS 기준 벡터를 구합니다.
파일 영구 저장·회전축/장착 방향 보정은 포함하지 않습니다.

위치를 바꾸면 기존 할당의 정렬 판정을 무효화하고 재할당을 기다립니다. 자동으로 다시 지향하지 않습니다.
동일 좌표 재적용은 기존 할당을 유지합니다. 계산 결과는 0.1도 단위로 전송하고 장치 limits를 그대로 검증합니다.
GUI 표적 목록의 각도는 RDR 기준, FCS 패널의 각도는 FCS 기준이므로 장비 위치가 다르면 두 숫자는 달라질 수 있습니다.

구동 요청은 ED03 보고 허용 후 DE02에서 reporting=1을 확인하고 ED06 시작을 보냅니다.
Success 응답은 요청 접수이며 실제 구동 완료는 후속 프레임의 isOperating으로 확인합니다.
정지 요청은 ED06 정지를 보내고 진행 중인 GUI 시작 단계를 취소합니다. CLI 정지도 GUI 시작 단계를 취소합니다.
통신 재연결만으로 감시를 자동 재개하지 않습니다. CLI `rdr` 명령과 확장 명령10~17은 유지합니다.

기본 assignment_enabled=false에서는 할당은 NotSupported입니다.
GUI 고각 설정은 정의되지 않은 보드 명령을 임의로 생성하지 않습니다.

## 검증

`scripts/test.ps1`은 독립 루프백 RDR/FCS와 실제 Windows Named Pipe·공유 메모리로
v3 크기·버전·체크섬·float 오프셋, 위치 편집, 보고→감시 시작, 정지 무효화, 소수 각도를 검사합니다.
실제 센서·모터 또는 WPF 창의 화면 검증은 이 시험에 포함되지 않습니다.
