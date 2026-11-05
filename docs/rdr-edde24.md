# RDR ED/DE 24바이트 연결

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
24바이트 Big Endian이며 좌표는 Base(right,up,forward), mm입니다.
팬·틸트는 signed int16, 0.1도 단위이고 -32768은 미상입니다.
현재 보드에 고각 설정 명령은 없습니다. [장비 ICD](device-icd.md)를 참고하세요.
