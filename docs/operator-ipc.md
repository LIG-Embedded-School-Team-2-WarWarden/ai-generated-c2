# 외부 운영 화면 IPC v3 유지

GUI 소스와 계약은 변경하지 않습니다. ECS만 새 DE03를 수신해 기존 GUI 형식으로 변환합니다.

| 항목 | 규격 |
|---|---|
| SharedMemory | `Local\ControlStationMock.SharedMemory.v3` |
| Mutex | `Local\ControlStationMock.SharedMemory.Mutex.v3` |
| Frame magic / version | `0x314D4354` / 3 |
| Frame / header / payload | 492 / 32 / 460 bytes |
| RDR / FCS / target | 40 / 40 / 36 bytes, Pack=4 |
| Pipe | `ECS_NamedPipe`, magic `0x50495045`, version 3 |
| Request / response | 40 / 40 bytes |

모든 필드는 little endian이며 float32 및 기존 FNV-1a 체크섬을 유지합니다.

## 좌표와 표시

원본 센서 XYZ(오른쪽·위·앞), 관측 PAN/TILT, Detected, TrackState는 ECS 내부 및 `rdr targets` JSON에 보존합니다. GUI v3에는 이 필드들을 추가하지 않습니다.
관측 TILT, 북쪽 기준 PAN을 순서대로 적용하고 설치 위치를 더해 기존 GUI XYZ(동·북·위)를 만듭니다. 장착 회전 단위행렬·장착 이동량 0을 가정하며 실측 검증은 별도입니다.
GUI 목표 목록 XYZ는 RDR 기준 상대 좌표가 아닌 **월드 좌표(미터)**입니다. `월드 표적 = 관측 자세로 회전한 센서 표적 + RDR 월드 위치`를 적용합니다. 같은 ID의 원본 XYZ·관측 자세가 갱신되면 목록과 지도도 최신 월드 좌표로 갱신됩니다. GUI 게시 시 최초 좌표 저장값을 사용하지 않으며 별도 콘솔 명령이 필요하지 않습니다. 기존 명령 계산의 좌표 저장 방식은 별도로 유지합니다.
PAN=1800은 북쪽입니다. RDR·FCS 장치 방향은 기존 GUI 방향선 계약의 135도 표시 오프셋을 유지합니다. 표적 목록의 방위각에는 이 오프셋을 적용하지 않으며 북쪽 0도·동쪽 +90도·서쪽 -90도 기준으로 전달합니다. RDR 방향은 기존 0~270도 범위로 제한하므로 범위 밖 방향은 기존 GUI에서 정확히 표시할 수 없습니다.
현재 RDR 자세와 표적 방향은 서로 다르며 표적 계산에는 해당 표적의 관측 자세를 사용합니다.
PREDICTED는 동일 ID와 좌표를 기존 유효 표적으로 전달합니다. 기존 GUI에는 별도 예측 라벨이 없습니다.
LOST와 자세 미상은 valid=0으로 전달하므로 기존 GUI 목록에서 숨겨집니다. ECS 내부 ID와 원본 정보의 보존과 GUI 표시 여부는 별개입니다.
ECS의 마지막 수신 이후 3초 통신 타임아웃과 송신측 마지막 관측 이후 최대 1.5초 예측 제한은 별개입니다. DE03에는 관측 시각이 없어 ECS가 실제 예측 경과 시간을 입증할 수 없습니다.
기존 GUI는 미상 각도 센티널을 처리하지 않으므로 현재 자세 미상일 때 기존 기본값(방위135/고각0)을 유지합니다.

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
동일 좌표 재적용은 기존 할당을 유지합니다. 계산 결과는 0.1도 단위로 전송하며 ECS의 자산별 팬·틸트 구동 한계 검사는 없습니다. 장비가 거부하면 장비 거부 결과를 전달합니다.
GUI 표적 목록의 각도는 RDR 기준, FCS 패널의 각도는 FCS 기준이므로 장비 위치가 다르면 두 숫자는 달라질 수 있습니다.

구동 요청은 ED03 보고 허용 후 DE02에서 reporting=1을 확인하고 ED06 시작을 보냅니다.
Success 응답은 요청 접수이며 실제 구동 완료는 후속 프레임의 isOperating으로 확인합니다.
정지 요청은 ED06 정지를 보내고 진행 중인 GUI 시작 단계를 취소합니다. CLI 정지도 GUI 시작 단계를 취소합니다.
통신 재연결만으로 감시를 자동 재개하지 않습니다. CLI `rdr` 명령과 확장 명령10~17은 유지합니다.

표적 할당은 RDR 감시·보고, FCS 연결·상태와 표적 유효성을 검사한 뒤 처리합니다.
GUI 고각 설정은 정의되지 않은 보드 명령을 임의로 생성하지 않습니다.

## 검증

`scripts/test.ps1`은 GUI v3 크기·버전·체크섬·기존 필드 배치, 새 DE03 수신과 관측 자세 변환, 예측 좌표 유지, 위치 편집과 연결 복구를 검사합니다.
실제 센서 장착 기하·좌표 정확도 검증은 별도입니다.
