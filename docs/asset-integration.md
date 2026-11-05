# RDR/FCS 연결

현재 C2는 RDR 제어 24바이트·DE03 32바이트와 FCS EF/FE 8·12바이트 UDP 이진 ICD를 사용합니다.
Protobuf 등록, OBS/EFF 명령, UDP 5000 경로는 현재 실행에 사용하지 않습니다.

## 실장비 주소

기본 설정은 `configs/rdr-live.json`입니다.

| 통신 | ECS local: 수신·바인딩 | peer: 상대 장치 |
|---|---|---|
| RDR | 192.168.0.102:1000 | 192.168.0.101:1000 |
| FCS | 192.168.0.102:2000 | 192.168.0.103:2000 |

local IP는 ECS PC가 가진 주소여야 합니다. peer는 상대 보드 주소입니다.
스크립트는 PC의 IP나 방화벽 설정을 변경하지 않습니다.
중계 경로는 `configs/rdr-relay.json`을 선택하고 실제 local/peer를 확인합니다.
수신 peer도 검사하므로 ECS에서 실제로 보이는 송신 IP·포트를 사용합니다.

## 실행

프로젝트 루트에서 빌드된 실행 파일을 실행합니다.

```powershell
.\x64\Release\C2.Console.exe --config configs/rdr-live.json --print-network
.\x64\Release\C2.Console.exe --config configs/rdr-live.json
```

`--print-network`는 소켓을 열기 전에 적용 주소를 출력하고 종료합니다.
RDR/FCS는 자동 연결하며 RDR 감시는 명시적으로 시작합니다.

```text
rdr status
rdr report on
rdr start
rdr targets
fcs status
rdr stop
quit
```

FCS EF01 response_timeout은 연결 응답 미수신입니다. local/peer, PC 주소,
보드 전원·네트워크·UDP 포트·ICD를 확인합니다. RDR과 FCS의 연결 성공은 별개입니다.
WPF는 같은 PC의 IPC에 연결합니다. 기본 IPC를 쓰는 C2는 하나만 실행합니다.
[장비 ICD](device-icd.md), [RDR 연결](rdr-edde24.md), [GUI IPC](operator-ipc.md)를 참고하세요.
