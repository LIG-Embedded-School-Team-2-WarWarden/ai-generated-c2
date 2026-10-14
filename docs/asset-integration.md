# 자산 접속 계약

## 공통 계약

고정 Protobuf: `external/warwarden-protocol/proto/mfs/icd/v3/mfs.proto`
커밋: `36be2707f6b88769118a0fb6519a8c5193e0ff3b`.
보드의 생성 코드도 이 스키마를 기준으로 만듭니다.

UDP datagram 하나에 직렬화한 `mfs.icd.v3.Envelope` 하나를 넣습니다.
JSON, 별도의 길이 prefix, gRPC framing은 붙이지 않습니다.
Envelope 안에 해당 payload를 설정해야 하며 개별 메시지만 직렬화해서 보내면 안 됩니다.
IPv4 UDP의 최대 datagram은 65507 bytes입니다. 원시 point cloud 전송 규격이 아닙니다.

| 항목 | 값/의미 |
|---|---|
| 목적지 | C2 PC LAN IPv4, UDP 5000 기본 |
| protocol_version | 3 |
| source_id | 감시 1, C2 2, 효과기 3 |
| destination_id | 자산 → C2는 2 |
| asset_id | 자산별 고유한 0 아닌 uint64 |
| session_id | 부팅/프로세스 시작마다 새 0 아닌 uint64 |
| sequence | 메시지별 증가하는 uint32, 0 제외 |
| timestamp_us | UTC Unix epoch microseconds |
| 위치/속도 | PROJECT_FRAME, m / m/s |
| 각도 | deg, 설치 방위는 +X → +Y, [0, 360) |

시계는 C2와 동기화해야 합니다. 부팅 이후 tick이나 milliseconds를 그대로 넣지 않습니다.
현재 공용 스키마는 PROJECT_FRAME만 지원합니다. 센서 로컬 좌표는 감시자산에서
설치 Pose를 적용해 공통 좌표로 변환한 뒤 보냅니다.

## 권장 송신 순서

1. 명령을 받을 UDP 소켓을 열고 포트를 정합니다.
2. 같은 송신 소켓으로 `AssetRegistration`을 전송합니다.
3. `Heartbeat`, `AssetPose`, 역할별 Status를 전송합니다.
4. 등록은 1초 주기로 갱신하고 lease_duration_ms=5000을 사용합니다.
5. Heartbeat는 1초, Status는 100ms 정도로 갱신합니다.
6. 감시자산은 `TargetCoordinate`를 유효시간 2초 이내에서 갱신합니다.

등록의 command_port에는 보드가 실제로 명령을 수신하는 포트를 넣습니다.
명령 목적지 IP는 등록 패킷의 실제 송신 IP로 결정됩니다.
등록·Heartbeat·Pose·Status·표적·ACK는 등록한 것과 **같은 송신 IP/port**에서 보내야 합니다.
소켓을 바꾸면 새 session_id로 다시 등록합니다.
Heartbeat만 보내고 등록 lease를 갱신하지 않으면 연결이 만료됩니다.

감시 capability: observation_scan bit 0 (1).
효과기 capability: effector_point bit 1 (2), effector_attack bit 2 (4).
개발 Pose 변경 지원: bit 3 (8), 선택적입니다.
자동 할당의 기본 정책은 point와 attack capability가 모두 있는 효과기를 요구합니다.
지원하지 않는 기능을 capability로 광고하지 않습니다.
PanTiltLimits는 실제 기구 한계를 명시합니다.

## 감시자산

- `ObservationStatus`: 현재 상태, 각도, 라이다/터렛 동작 상태를 보고합니다.
- `AssetPose`: PROJECT_FRAME 설치 위치와 방위를 보고합니다.
- `TargetCoordinate`: 안정적인 detection_id, measurement_time_us, 위치, confidence,
  속도 및 velocity_valid를 보냅니다.
- detection_id가 같더라도 다른 asset/session이면 C2가 다른 전역 track_id를 부여합니다.
- 속도를 모르면 velocity_valid=false입니다. 정지 표적은 velocity_valid=true와 0 속도로 표현합니다.
- `ObservationTurretCommand`를 수신해 HOME/STOP/ABSOLUTE_ANGLE/SCAN을 실행하고 ACK를 보냅니다.

## 팬틸트 자산

- `EffectorStatus`, `AssetPose`, `Heartbeat`를 정기적으로 보냅니다.
- `EffectorTurretCommand`: 수동 절대 각도 지향. 현재 공용 규격에는 track_id가 없습니다.
- `TargetTrackUpdate`: 할당된 표적의 전역 좌표·속도 스트림. 기존 규격에서는 자산이
  자신의 설치 자세를 반영해 추적합니다. 이 상태 스트림에는 CommandAck를 보내지 않습니다.
- `AttackCommand`: ARM/START/STOP/EMERGENCY_STOP. 기한과 상태를 확인한 뒤 수행합니다.
- START의 duration_ms는 출력 시간이며 자동 추적 기간과 구분됩니다.
- 표적 기한 만료, 통신 단절, 구동 한계, STOP/ESTOP 처리는 장치에서도 수행합니다.

## 명령과 ACK

header.asset_id/session_id가 현재 장치와 일치하고 valid_until_us가 남아 있는지 검사합니다.
`CommandAck.command_id`에는 받은 command_id를 그대로 넣습니다.
ACK의 Header source/destination은 자산 → C2이며, 자산의 현재 session_id를 사용합니다.
ACK는 명령을 보낸 UDP source port가 아니라 **설정한 C2 공용 수신 포트 5000**으로 보냅니다.
C2의 명령 송신 소켓은 별도의 ephemeral port를 사용합니다.

RECEIVED/ACCEPTED/IN_PROGRESS는 완료가 아닙니다.
실제 완료 시 COMPLETED, 거부/실패 시 REJECTED/FAILED를 보고합니다.
같은 command_id의 재전송은 중복 실행하지 말고 저장한 ACK를 반환합니다.
C2 기본 delivery ACK timeout은 200ms, 시도는 최초 포함 3회, 완료 timeout은 2초입니다.

## Zynq

Linux/C++ 환경은 공용 cpp 코덱을 장비의 POSIX/lwIP UDP 어댑터와 연결할 수 있습니다.
이 통제소의 Winsock 구현을 보드에 그대로 빌드하는 방식은 아닙니다.
Bare-metal/RTOS라면 동일 `.proto`로 nanopb C 생성 코드를 준비하고 Envelope를 송수신합니다.
이번 작업에서 보드 BSP, lwIP 초기화, nanopb 생성 코드 및 하드웨어 드라이버는 제공하지 않습니다.
보드 환경이 정해지면 해당 어댑터를 별도 파트에서 구현해야 합니다.

## 접속 확인

1. C2와 보드가 서로 도달 가능한 IPv4 네트워크인지 확인합니다.
2. Windows 방화벽에서 C2 실행 파일 또는 UDP 5000 인바운드를 허용합니다.
3. 대시보드에 자산과 CONNECTED, pose=OK가 표시되는지 확인합니다.
4. Status가 최신인지, 표적이 화면에 나타나는지 확인합니다.
5. 관측 SCAN/STOP 또는 수동 지향을 요청하고 ACK COMPLETED를 확인합니다.
6. 연결이 안 되면 콘솔 `events`, `errors`, `pending`, `outcomes`, `metrics`를 조회합니다.

실제 보드 호환성은 위 메시지를 구현한 펌웨어와 실제 네트워크에서 확인해야 합니다.
임의의 기존 UDP 패킷이나 다른 버전의 Protobuf 메시지를 자동 변환하지 않습니다.
