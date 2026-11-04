# Team ICD 1.0 C++ 콘솔 MVP

## 시연 시나리오

C2 실행에는 Python이 필요 없습니다. 더미 시험에는 `./scripts/setup-team.ps1`을 먼저 실행합니다.
`./scripts/build.ps1 -BuildDir build/native`로 빌드한 뒤 각각 별도 터미널을 사용합니다.

```powershell
./build/native/Release/c2_mvp_console.exe
./.venv-team/Scripts/python.exe scripts/team_dummy.py OBS
./.venv-team/Scripts/python.exe scripts/team_dummy.py EFF
```

세 프로그램이 같은 localhost UDP 5000을 사용합니다. 다른 C2 서버와 동시에 포트를 점유하지 않습니다.
더미 OBS/EFF는 합성 시험 장치이며 실제 LiDAR SDK나 모터 드라이버가 아닙니다.

1. 장치 등록 → C2 접속 UUID 발급 → 기능 보고 → 설치 적용 결과와 후속 상태 revision 확인.
2. 고정 OBS 모드 확인 → C2 수집 UUID 발급 → MEASUREMENT_COMMAND START 실제 결과 확인.
3. OBS가 전체 8×6 원본 슬롯을 POINT_CLOUD_CHUNK와 OBS_POSE로 전송합니다. 테스트는 역순·중복 조각도 보냅니다.
4. C2가 원본 배열을 복원해 LiDAR Detector에 전달합니다. `targets`에서 검출 중심과 C2 object UUID를 확인합니다.
5. `aim 201` → CONTROL_OPEN 완료 확인 → 최신 관측마다 최대 5Hz로 POINTING_COMMAND POINT.
6. `status`에서 RUNNING과 장치가 보고한 각도를 확인합니다.
7. `stop 201` → 이전 POINT 재전송 취소 → 같은 제어 세션에서 더 큰 순번의 STOP → 결과와 후속 정지 상태 확인 후 STOPPED.
8. `quit`과 입력 EOF는 지향 정지를 요청·확인하고 수집 중지를 요청한 뒤 종료합니다. Linux의 Ctrl+C도 같은 정리 경로를 사용합니다. 응답 미확인은 마지막 상태 출력에 남습니다.

## 명령

레이저 명령 `laser on 201 DURATION_MS`, `laser off 201`, `laser status 201`와 지원 조건은
[레이저 운용 안내](laser-control.md)를 참고하세요. `status`에는 `lasers` 상태도 포함됩니다.

| 명령 | 동작 |
| --- | --- |
| `status` | 접속/부팅 UUID, 설치·측정 상태, 현재각 유효성, 제어 상태 |
| `targets` | 최신 분석 프레임·물체 목록. 오래되거나 조립 중인 좌표는 현재값으로 내보내지 않음 |
| `point 201 20 10` | 지정 각도 1회 이동. 관측 없이 수동 제어 세션으로 실행; 다음 조작 전 stop 확인 |
| `aim 201 [object-UUID]` | 단일 물체 연속 지향 시작. 선택 시 UUID 일치 필수 |
| `stop 201` | 지향 중지. 수집은 계속. STOP_UNCONFIRMED이면 다시 요청 가능 |
| `collect stop` / `collect start` | 지향 중지 요청 후 측정 중지 / 새 수집 세션 시작 |
| `obs point PAN TILT` | EFF 정지·수집 중지 확인 → OBS 유한 이동·실측 정지 확인 → 수집 재개 |
| `obs stop` / `obs status` | OBS 이동·수집 정지 요청 / 단계·현재각·관측 상태 확인 |
| `clock-profile 101 profile.json` | firmware와 같은 검증 프로파일을 현재 boot/session에 연결 |
| `events` | 최근 요청·결과·거부 진단 |
| `clock-load 101 evidence.json` | 현재 boot/session에 대한 검증된 센서 시각 매핑 증빙 적용 |
| `quit` | 지향·측정 정리 후 종료 |

GUI, 다중 객체 선택·재획득, 이동 중 OBS 취득·비영점 OBS 자동 지향, GUI IPC, 녹화/재생은 이번 범위에서 제외했습니다.
현재 MVP는 물체가 하나일 때만 aim을 시작하며, 프레임 사이 0.5m 이내 단일 검출을 같은 물체로 연결합니다.
이 단순 연결은 원본 LiDAR BoxTracker 전체를 가져온 것이 아니며, 교차·가림·동일 위치의 물체 교체를 판별하지 못합니다.
소실·모호함·시각 불신·장치/현재각 만료·명령 미확인 후에는 자동 재개하지 않습니다.
수동 `point`도 같은 CONTROL_OPEN/POINTING_COMMAND 경로를 사용합니다. 이전 `offset`, `axes` 명령 대신 설치값을 설정 파일에서 확인합니다.

## 통신과 계산

공용 `mfs.icd.v1.Envelope`만 사용하고 직렬화 전체 크기는 1,200 bytes 이하입니다.
UUID 접속·수집·제어 세션은 서로 구분합니다. 기존 v3 패킷과 private field 100을 변환해 받지 않습니다.
상태의 sampleSequence, Heartbeat의 sequence는 각각 같은 접속 세션 내 uint64 증가값만 적용합니다.
동일/이전 순번은 수신시각·최신성을 갱신하지 않습니다. 이전 v3의 uint32 순환 규칙은 적용하지 않습니다.

각 POINT에는 관측 장치/접속/프레임, 수집·물체 ID, 양쪽 설치 revision, 1초 만료시각을 넣습니다.
200ms 간격 최대 3회 재전송은 동일 messageId·sentAt·요청 본문을 유지합니다.
ACCEPTED는 최종 완료가 아니며 장치가 보고한 완료와 응답 미확인을 구분합니다.
STOP 응답이 없으면 실제 정지 각도가 보고되어도 STOPPING/STOP_UNCONFIRMED로 남습니다.

원본 점군은 무효 픽셀을 포함한 모든 슬롯을 pixelIndex로 복원합니다.
중복 조각은 무시하고 충돌·겹침·누락은 거부합니다. 조립 1개·대기 1개, 조립 1초,
500,000 슬롯·8,192 조각·8MiB 상한을 적용합니다. 더 최신 프레임을 받으면 이전 분석 결과를 사용하지 않습니다.
센서의 SDK XYZ를 원본대로 보내되, x/y/z 유효성·sdkDepthRaw·SDK 시각·프로파일·취득 자세를 함께 전달해야 합니다.

설치 회전과 위치를 적용한 후 상대 벡터 `(forward,left,up)`에서:

```text
pan  = atan2(left, forward)
tilt = atan2(up, hypot(forward, left))
```

설정의 `position`은 공통 좌표계에서 장비 기준점의 설치 위치이고,
`sensor_offset`은 장비 기준점에서 실제 센서·포인터 기준점까지의 고정 장착 오프셋입니다.
둘 다 미터 단위 `[X,Y,Z]`이며, 오프셋은 각 장비의 로컬 축으로 입력합니다.
설치 방향은 `rotation`으로 적용합니다. OBS(`101`)와 FCS/EFF(`201`)의 오프셋을 각각 반영합니다.

```text
물체 공통좌표 = OBS.position + OBS.rotation × (측정좌표 + OBS.sensor_offset)
FCS 센서 위치 = FCS.position + FCS.rotation × FCS.sensor_offset
(forward,left,up) = FCS.rotation의 역회전 × (물체 공통좌표 − FCS 센서 위치)
```

예를 들어 FCS 장비 기준점이 `[0,0.1,0]`이고 센서가 장비 로컬 축에서 위로 5cm 떨어져 있으면,
`201.position`은 그대로 두고 `201.sensor_offset`을 `[0,0,0.05]`로 설정합니다.
실행 시 `--config`로 선택한 파일을 수정하고 C2를 재시작하면 자동 `aim` 계산에 반영됩니다.
수동 `point`는 입력 각도를 그대로 사용합니다. GUI의 장비 위치 표시는 `position`을 사용합니다.
이 오프셋은 설치 좌표계에 고정된 변위이며, 팬·틸트 회전에 따라 이동하는 장착점의 기구학은 포함하지 않습니다.

두 장치 축이 같고 오프셋이 0이면 위치를 빼는 계산으로 줄어듭니다. SDK 축을 임의로 x=전방이라고 가정하지 않습니다.
`configs/team-demo.json`의 축과 위치는 합성 데이터 예시입니다. 실제 장비 보정값이 아닙니다.
OBS 자동 지향은 측정된 pan=tilt=0의 고정 관측만 지원합니다. OBS 자체는 CLI로 이동할 수 있습니다. 물체 중심은 그대로 재사용한 Detector가 반환한 검출 점들의 평균입니다.
출처: `vendor/lidar/ORIGIN.md`. 검출 헤더는 원본과 동일합니다.

## 실제 OBS/EFF 연결 시 필요한 것

실물용 설정은 `simulation: false`로 별도 작성하고 `--config path --bind C2_PC_IP`로 실행합니다.
허용 deviceId/role, 설치 위치·오프셋·오른손 회전행렬, calibrationId, OBS sensorDataProfileId,
필수 허용 시각 오차를 실제 확인한 값으로 제공합니다. EFF는 실제 각도 피드백, STOP/DRAIN 지원,
허용각과 실제 완료 판정 timeout을 CAPABILITY_REPORT로 보고해야 합니다.

기본 합성 시각 프로파일은 loopback에서만 허용됩니다. 실물 프레임은 검증된 시각 매핑 없이 DISPLAY_ONLY로 처리합니다.
firmware의 취득 프로파일은 `clock-profile 101 profile.json`으로 읽을 수 있습니다. 유효기간과 센서 설정을 확인하고 현재 boot/session에 연결합니다.
기존 `clock-load` 증빙 파일에는 다음 값이 필요합니다. 예시 값으로 검증을 대신하지 마세요.

```text
id, boot_id, connection_session_id, evidence,
offset_seconds, uncertainty_ms, sync_type, sync_state, frame_protocol_version
```

`status`의 현재 접속·부팅 UUID와 일치해야 합니다. `sdkTime + offset_seconds`와 capturedAt을 대조하며,
취득 구간·실측 고정 자세·설치/모드 revision·profile도 확인합니다.
이 함수는 외부에서 검증한 매핑을 입력하는 경로이며 시계 동기화나 노출 구간을 스스로 측정하지 않습니다.
실제 NTP/SDK 매핑·취득 구간 동안의 안정 정지는 장치 담당자가 확인해야 합니다.

보드의 원본 송신 어댑터와 모터 드라이버는 각 담당 저장소에 이 공용 schema로 연동해야 합니다.
이번 시험은 실제 센서·모터·레이저의 하드웨어 검증이 아닙니다.

## 실행 검증

```powershell
./.venv-team/Scripts/python.exe tests/test_native_mvp.py --console build/native/Release/c2_mvp_console.exe
```

테스트는 생성 Protobuf + 실제 localhost UDP + 실제 C++ Detector로 실행됩니다.
순서 뒤바뀜/중복 조각, 연속 지향과 stop, 패킷 재전송 멱등성, 상태/Heartbeat 순번,
대상 소실, 불완전/상충 프레임, 시각 정보 누락, ACCEPTED만 오는 경우와 STOP 응답 유실을 포함합니다.
C++ 콘솔을 독립 프로세스로 실행하고 Python OBS/EFF 더미와 UDP로 연결해 stdin으로 명령을 넣고, 수집 중지/재시작과 종료까지 검사합니다.
설치 revision 변경 시 추가 지향을 차단하는 경우도 검사합니다.
