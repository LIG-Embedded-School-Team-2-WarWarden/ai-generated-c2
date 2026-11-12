# RDR 제어 24바이트·DE03 32바이트 연결

현재 RDR 코드는 `C2.Core/Include/c2/rdr_client.hpp`와 `rdr_session.hpp`입니다.
`C2.Console.exe`에서 사용하며 Protobuf나 이전 c2_mvp_console은 필요하지 않습니다.

## 설정과 실행

`configs/rdr-live.json`의 rdr.local은 ECS 192.168.0.102:1000,
rdr.peer는 보드 192.168.0.101:1000입니다.
같은 설정의 FCS는 ECS 192.168.0.102:2000 ↔ 보드 192.168.0.103:2000입니다.

```powershell
.\x64\Release\C2.Console.exe --config configs/rdr-live.json --print-network
.\x64\Release\C2.Console.exe --config configs/rdr-live.json
```

`--print-network`는 소켓을 열지 않고 적용 주소만 출력합니다.
`scripts/run_rdr_live.ps1 -PrintPlan`은 실행 계획을 출력합니다.
FCS 없이 RDR만 확인하려면 `--rdr-only`를 추가합니다.

## 중계

`configs/rdr-relay.json`의 RDR local은 192.168.64.1:1000, peer는 192.168.64.129:1000입니다.
실제 PC·VM에 맞게 선택·수정합니다. peer는 ECS에서 보이는 송신 주소입니다.
스크립트는 PC 주소를 바꾸거나 기존 ECS를 종료하지 않습니다.

## 명령과 단위

```text
rdr status
rdr report on
rdr start
rdr targets
rdr exclude 7
rdr resume 7
rdr stop
```

연결은 자동입니다. 최초 연결·재연결 뒤 보고 허용과 감시 시작은 명시적으로 요청합니다.
GUI 구동은 ED03 보고 허용 후 DE02 확인을 거쳐 ED06 시작을 보냅니다.
정지 시 측정값을 무효화하며 재연결만으로 감시를 재개하지 않습니다.
제어는 24바이트, DE03은 32바이트 Big Endian입니다. 원본 센서 XYZ·관측 자세·TrackState를 받으며 실장비 X의 좌우 반전은 ECS에서 `right=-raw_x`로 보정합니다. Y는 위쪽, Z는 앞쪽입니다.

| Offset | 필드 | 형식 |
|---|---|---|
| 0 / 2 / 4 / 6 | ID=0xDE03 / TxSeq / RxSeq / TargetID | uint16 |
| 8 / 12 / 16 | 원본 X / Y / Z | IEEE-754 float32, m |
| 20 / 22 | 관측 TILT / PAN | int16, 0.1° |
| 24 / 26 | Detected / TrackState | uint16 |
| 28–31 | 예약 | 모두0 |

TrackState=0 LOST는 Detected0·XYZ0·관측 자세 미상,1 OBSERVED는 Detected1,2 PREDICTED는 Detected0·유효한 예측 XYZ입니다.
ECS는 PREDICTED의 같은 ID·좌표·관측 자세를 보존합니다. 잘못된 상태 조합, LOST의 잔여 좌표/자세, 비유한 XYZ, 예약값과 길이 오류를 거부합니다.
송신측은 마지막 관측 또는 점군 지지 후 최대 1.5초까지만 예측을 유효하게 보내며 예측 자체로 기한을 연장하지 않습니다.
패킷에 관측 시각은 없습니다. ECS의 마지막 DE03 수신 후 3초 무효화는 별도 통신 타임아웃이며 송신측 예측 기한을 검증하는 시계가 아닙니다.

관측 PAN=1800은 북쪽, 오른쪽 회전이 양수입니다. TILT는 수평0°·위쪽 양수입니다.
ECS가 해당 객체의 관측 자세로 XYZ를 한 번 회전해 동쪽/북쪽/위쪽 좌표와 방향각을 계산합니다. 현재 모터 상태를 객체의 관측 자세로 대체하지 않습니다.
장착 회전 단위행렬·장착 이동량 0을 가정하며 실장비 장착 기하·좌표 정확도 검증은 별도입니다.

[RDR-ECS ICD v13](https://ligproject2team.atlassian.net/wiki/spaces/MFS/pages/37355617/RDR-ECS+ICD)에 위 TrackState·원본 좌표·관측 각도 계약을 반영했습니다.
RDR 송신 펌웨어도 이 계약을 적용해야 합니다. 같은 32바이트라도 구형 보정 XYZ와 원본 XYZ의 의미를 자동 구분할 수 없으므로 송신측 적용 버전과 실제 패킷을 대조해야 합니다.
팬·틸트는 signed int16, 0.1도 단위이고 -32768은 미상입니다.
현재 보드에 고각 설정 명령은 없습니다. [장비 ICD](device-icd.md)를 참고하세요.
