# FCS 수동 동작과 GUI 데모

현재 FCS는 EF/FE 이진 ICD입니다. 이전 Protobuf LaserCommand/LaserResult 경로는 사용하지 않습니다.

```text
fcs status
fcs point 20.5 180.2
fcs status
fcs laser on
fcs laser off
quit
```

point는 고각·모터 방위각 순서, 도 단위, 0.1도 간격입니다.
설정의 FCS limits를 벗어나면 거부합니다.
laser on/off는 EF04를 보냅니다. 수동 ON 후 C2는 1초가 지나면 OFF를 요청하고,
정상 종료 시 연결된 FCS에 OFF를 요청합니다. 이는 장치 자체 출력 시간 보장을 뜻하지 않습니다.
응답 미수신은 결과 미확정이며 같은 제어를 자동 재송신하지 않습니다.
후속 FE02의 flags와 실측 각도를 확인합니다.

GUI 할당은 EF03 지향 요청 후 실측 정렬을 확인합니다.
GUI Fire는 EF05 장치 내장 데모, Cancel은 EF04 OFF를 요청합니다.
`assignment_enabled=false`가 기본값입니다.
실장비에서 활성화하기 전에 설치 좌표와 구동 범위를 검증해야 합니다.
위치 변경·표적 소실·연결 상태 등으로 정렬이 무효화되면 재할당해야 합니다.

모의기는 flags·각도를 메모리에서 바꾸며 실제 출력을 만들지 않습니다.
자동 시험은 ICD 통신·상태 변화 검증이며 실물 출력 검증은 아닙니다.
[장비 ICD](device-icd.md), [GUI IPC](operator-ipc.md)를 참고하세요.
