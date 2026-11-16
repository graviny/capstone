# 외부 명령 기반 멀티로봇 통신·실행 시스템

## 1. 구현 결과와 범위

`feature/pi_car1`의 기존 `pi_car/`를 확장했다. PC가 외부 MOVE를 받아 지정 차량에
목표를 전달하고, 차량 실행부가 저장 지도와 최신 Pose로 A*를 실행하여 주행 명령을 계산한다.
도착하면 정지·ARRIVED 보고 후 대기한다. 목표 대기열·목표 자동 생성·작업 배정은 없다.
PC의 Fleet 서버는 경로를 계산하거나 경로를 전송하지 않는다.

다음 두 실행 경로를 제공한다.

- 실제 구성: D455 → 다중 태그 위치 추정 → PC Fleet TCP → Pi RobotClient/RobotAgent.
  CLI의 실제 모터는 기본 비활성을 넘어 **항상 비활성**으로 제공한다.
- Mock 구성: 외부 위치 관측을 모사하는 테스트 월드 → 동일한 TCP → 동일 RobotAgent/Runtime.
  모터 출력으로 가상 위치만 갱신하며, 명령이 없으면 아무 목표도 발급하지 않는다.

WES, 새로운 Mapping/SLAM, 강화학습, 다중 로봇 경로 최적화·로봇 간 충돌 회피는 구현하지 않았다.
로봇별 독립 실행은 상호 충돌 방지를 의미하지 않는다.

## 2. 통신과 모듈 구조

```text
향후 WES / 수동 CLI
    │ MOVE(robot_id, command_id, goal) / STOP
    ▼
CommandInterface → FleetController.dispatch_goal() / stop_robot()
    │                 │ 차량별 상태 + 영구 명령 기록 + 결과 대기 API
    ▼                 ▲
FleetTCPServer ── 하나의 양방향 TCP/차량 ── RobotClient
    ▲                                      │
POSE(robot_id, frame_id, map_version)         ▼
    │                                   RobotAgent (Pi)
D455 → MultiRobotLocalizer               지도 검증 → A* → VehicleRuntime
                                           → WaypointFollower → WatchdogMotor
                                           → DisabledMotor (CLI 기본)
```

상위 인터페이스는 `pc/commands.py`, 상태/명령 처리는 `pc/fleet.py`, 실제 TCP는
`pc/server.py`와 `transport/tcp.py`에 나눴다. RealSense 없이 API/TCP를 사용할 수 있다.

### 차량별 상태

`FleetController.robots[robot_id]`에 연결 상태, 최신 유효 Pose, 현재/마지막 command_id,
목표, 상태, 마지막 통신 시각을 보관한다. 명령 직렬화 lock과 Pi의 실행기도 차량별로 독립적이다.
READY 이후 IDLE에서 대기하고, 외부 명령이 있어야 새 목표가 생긴다.

### 프레이밍과 메시지

UTF-8 JSON Lines, `schema_version:1`, 한 메시지 최대 16384 bytes다.
`readline` 버퍼로 분할/합쳐진 수신을 처리하고 개행 없는 종료, 과대 메시지, 중복 JSON 키,
비정상 숫자 및 식별자를 거부한다. 송신 lock으로 메시지가 섞이지 않게 한다.

모든 차량 메시지에 `robot_id`, `frame_id`, `map_version`을 넣는다.
GOAL/STOP 및 명령 상태에는 `command_id`도 넣는다. ID는 1~128자의 영문·숫자·`_.:-`만 허용한다.

| 방향 | 메시지 | 의미 |
| --- | --- | --- |
| Pi → PC | HELLO | robot_id, 지도 frame/version, clearance 등록 |
| Pi → PC | READY | 새 명령을 받을 수 있음. 목표 자동 발급 없음 |
| PC → Pi | POSE | 해당 차량의 위치·방향·촬영 시각·valid |
| PC → Pi | GOAL | 외부 이동 명령 |
| PC → Pi | STOP | 정지 명령. STOP에도 별도 고유 command_id 사용 |
| Pi → PC | GOAL_ACCEPTED | **Pi에서** 경로 생성 성공 |
| Pi → PC | MOVING | Runtime이 이동 출력을 계산하기 시작 |
| Pi → PC | ARRIVED | 최종 목표 도착, 모터 정지 |
| Pi → PC | BLOCKED | 초음파 장애물로 종료·정지 |
| Pi → PC | ERROR | 위치·지도·계획·제어·통신 오류 |
| Pi → PC | CANCELLED | 기존 명령을 새 명령/STOP으로 중단 (추가 상태) |
| Pi → PC | STOPPED | STOP 처리 완료 (추가 상태) |
| 양방향 | PING / PONG | heartbeat (추가 메시지) |

API의 `PENDING`은 송신·실행 대기 기록이다. 차량의 수신·경로 생성 성공을 뜻하지 않는다.
수신 성공은 이후 `GOAL_ACCEPTED`로 확인한다.

GOAL 예시:

```json
{"type":"GOAL","schema_version":1,"robot_id":"B","command_id":"CMD001","frame_id":"floor-uuid","map_version":"map-uuid","goal":{"x":2.5,"y":1.5}}
```

## 3. 명령 수명과 안전 정책

1. PC는 robot_id, 연결 상태, 목표 숫자·Free 여부, 선택적 frame/version을 검증한다.
2. 해당 차량의 연결에만 GOAL을 보낸다. PC가 A*를 실행하지 않는다.
3. Pi는 로봇/지도/명령 ID를 재검증하고 현재 주행을 정지·기존 명령을 CANCELLED로 처리한다.
4. 최신 Pose와 차량 내부 nav grid로 **새 GOAL에 대해서만** A*를 실행한다.
5. 성공 시 `VehicleRuntime.set_path()`로 넘기고 GOAL_ACCEPTED를 보고한다.
6. 매 제어 tick에는 최신 Pose로 경유점을 추종한다. 정상 주행 중 A*를 반복하지 않는다.
7. 도착하면 ARRIVED를 기록하고 경로를 폐기·정지하며 READY를 보낸다.

A*가 반환하는 셀 중심 뒤에 정확한 외부 목표 좌표를 최종 경유점으로 추가한다.
이 실행기의 허용 거리는 `min(0.05m, 지도 해상도×0.45)`로 설정한다.
이는 초기 계산 파라미터이며 실차 성능이나 정지 정확도를 보장하지 않는다.

### 교체·취소·중복

- 새 ID의 GOAL은 이전 주행을 중단한다. 기존 결과는 CANCELLED, 새 명령은 별도로 추적한다.
- PC에서 잘못된 교체 목표를 거부할 때에도 해당 차량에 안전 STOP을 발급한다.
  내부 STOP의 ID는 `safety.<uuid>`이며 목표 생성이나 목표 대기열은 아니다.
- 같은 ID·같은 payload는 기록된 결과를 반환한다. Pi에서도 같은 ID를 다시 실행하지 않는다.
- 같은 ID를 다른 차량/목표에 재사용하면 충돌로 거부한다. 명령 ID는 전체 Fleet에서 고유해야 한다.
- 늦게 끝난 A* 결과에는 generation 검사를 적용해 STOP/새 명령 이후 경로가 살아나지 않게 한다.
- ARRIVED 등 최종 결과는 PC에서 불변으로 보관한다. 중복/뒤늦은 상태 보고가 결과를 바꾸지 않는다.

### 실패 시 재출발 방지

통신 단절, 태그 검출 실패, Pose timeout, frame/map 불일치, 경로 실패, 초음파 이상/장애물은
해당 명령을 ERROR/BLOCKED로 종료하고 경로를 폐기한다. 원인이 해소되거나 재연결돼도
그 명령을 다시 실행하지 않는다. **새 ID의 외부 명령이 있어야 재출발할 수 있다.**
STOP은 이전 명령을 CANCELLED로 만들고 STOP 명령 자체에는 STOPPED를 반환한다.

Pose에는 PC 촬영 시각(UNIX seconds)을 유지한다. Pi는 촬영 age와 로컬 수신 monotonic age를
함께 확인한다. 기본 Pose timeout 0.3초, TCP 무응답 timeout 2초, heartbeat 0.5초,
제어 주기 0.02초, front range timeout 0.2초다. PC/Pi의 시각 동기화가 필요하다.

### 영구 ID 기록과 재연결

`CommandJournal`은 SQLite에 명령 fingerprint와 마지막 결과를 저장한다. CLI는 PC와 Pi별
파일 경로를 사용한다. 실제 실행에서는 파일을 유지해야 재시작 후 중복 처리가 유지된다.
서버/실행기가 재시작되면 미완료 기록을 ERROR로 만들고 복구 주행을 하지 않는다.
Pi 재연결 시에는 최종 결과만 재보고한다. PC는 기록된 최종 결과를 덮어쓰지 않는다.
연결 단절 직전에 실제 도착했는지 불명확하면 PC는 ERROR를 유지할 수 있으며 상위 시스템이
위치/작업을 확인하고 새 명령으로 판단해야 한다.

HELLO의 로봇·지도·여유 값이 맞지 않거나 이미 연결된 robot_id이면 새 연결을 거부한다.
다른 로봇의 세션은 유지한다. 이 TCP 예제는 인증/TLS를 제공하지 않으므로 신뢰하는 실험망에서 사용한다.

### Watchdog

`WatchdogMotor`가 제어 loop와 별도 스레드에서 기본 0.25초의 명령 갱신 만료를 감시한다.
만료되면 정지·래치하고 새 명령에서만 arm한다. Python 프로세스 강제 종료, GIL/OS 전체 정지,
I2C 고장까지 이 스레드가 보장하지는 않는다. 실제 주행 전에는 모터 전원/컨트롤러 측의
독립적인 fail-off와 긴급 정지를 추가해야 한다.

## 4. 상위 시스템 연결 API

Fleet 서버가 사용하는 **같은 FleetController 인스턴스**를 WES 어댑터에 주입한다.
메서드는 비동기식이며 서버의 asyncio loop에서 호출한다.

```python
# fleet는 FleetTCPServer에 넘긴 FleetController 객체
receipt = await fleet.dispatch_goal(
    robot_id="B", x=2.5, y=1.5, command_id="CMD001"
)
# receipt는 PENDING일 수 있다. 다음 호출로 완료/실패 결과를 받는다.
result = await fleet.wait_result("CMD001", timeout_s=60)
print(result["type"])  # ARRIVED / ERROR / BLOCKED / CANCELLED

await fleet.stop_robot(robot_id="B", command_id="STOP001")
snapshot = fleet.snapshot()
last_result = fleet.command_result("CMD001")
```

검증 거부는 `ValueError`, 완료 대기 시간 초과는 `asyncio.TimeoutError`다.
`wait_result`의 호출자 timeout 자체가 로봇을 정지시키지는 않는다. 취소하려면 STOP을 호출한다.
요청 예시 그대로 `CommandInterface(fleet).handle({...})`를 호출하는 방식도 제공한다.
WES 서버·주문 처리·배정 알고리즘은 이 구현에 없다.

## 5. 실행 방법

모든 명령은 저장소의 `pi_car/`에서, 설치한 가상환경을 활성화하고 실행한다.

### 장비 없는 Mock 2대

```bash
python -m pip install -e '.[vision]'
pi-car mock-fleet
```

4×3m 테스트 격자와 초기 위치 A=(1,1), B=(1,2)를 사용한다. 초기 위치만 정하며 목표는 없다.
다른 터미널에서 외부 명령을 보낸다.

```bash
pi-car send-command --json '{"type":"STATUS"}'
pi-car send-command --json '{"type":"MOVE","command_id":"CMD001","robot_id":"A","goal":{"x":1.5,"y":1.0}}'
pi-car send-command --json '{"type":"RESULT","command_id":"CMD001"}'
pi-car send-command --json '{"type":"STATUS"}'
# 추가 명령이 들어와야 다음 이동을 시작한다.
pi-car send-command --json '{"type":"MOVE","command_id":"CMD002","robot_id":"B","goal":{"x":1.5,"y":2.0}}'
pi-car send-command --json '{"type":"STOP","command_id":"STOP001","robot_id":"B"}'
```

이 Mock의 A*·Runtime·TCP·명령 처리기는 실제 실행부와 같다. 단순 차동 모델은 Adeept 물리 모델이
아니며 로봇끼리의 충돌도 처리하지 않는다. Mock journal은 메모리 기반이고 프로세스 종료 시 사라진다.

### 실제 PC: D455 + 다중 차량 서버

먼저 카메라를 고정하고 지도와 calibration을 준비한다. 양쪽 Pi에 같은 원본 지도 폴더의
`map.npy`, `map_metadata.json`을 복사한다. 지도 전송 자동화는 이번 범위에 포함하지 않았다.
`config/robots.json`의 태그 ID·실측 변 길이·차량 중심/방향 오프셋을 실제 값으로 바꾼다.
예시 8cm·중심 아래 10cm는 실측값이 아니다.

```bash
python -m pip install -e '.[pc]'
pi-car fleet-server --map data/map --calibration data/calibration.json \
  --robots config/robots.json --host 0.0.0.0 --port 8765 \
  --command-port 8766 --clearance 0.20 --journal data/fleet.sqlite3
```

외부 명령용 테스트 포트 8766은 기본 localhost에만 열고, 로봇 TCP 8765는 지정 host에 연다.
WES를 동일 PC 프로세스에서 연결하면 별도 명령용 TCP를 거치지 않고 위 API를 사용할 수 있다.
GUI 검사 도구와 Fleet 서버가 동시에 D455를 열지 않도록 한다.

### 각 Raspberry Pi: 현재는 모터 비활성 Dry-run

PC 주소와 각 차량 robot_id를 맞춘다. 지도 frame/version과 clearance가 PC와 같아야 한다.

```bash
python -m pip install -e .
# Pi A
pi-car robot-client --robot-id A --host 192.168.0.10 --map data/map \
  --clearance 0.20 --journal data/robot_A.sqlite3 --mock-range 1.0
# Pi B: 다른 차량에서 실행
pi-car robot-client --robot-id B --host 192.168.0.10 --map data/map \
  --clearance 0.20 --journal data/robot_B.sqlite3 --mock-range 1.0
```

`--mock-range`는 Dry-run에서만 쓰는 가상 초음파 값이다. 생략하면 실제 센서가 연결돼 있지
않으므로 안전 입력 부족으로 정지한다. 이 CLI에는 실제 모터 활성화 옵션이 없다.
실제 PC 서버의 Dry-run은 물리적으로 차량이 움직이지 않으면 Pose도 움직이지 않아 도착하지 않는다.
실제 모터 출력 없이 도착까지 보려면 앞의 Mock 환경을 사용한다.

## 6. 다중 태그 처리

`MultiRobotLocalizer`는 한 영상에서 태그를 한 번 검출하고 차량별 태그 크기·중심·방향 오프셋으로
기존 `AprilTagLocalizer.estimate_corners()`를 호출한다. 각 차량에 Pose 또는 None을 반환한다.
한 태그가 사라지면 그 차량에만 `valid:false`를 전송한다. 다른 차량의 위치 전송은 계속된다.
카메라 예외 또는 전체 좌표계 불일치 시에는 모든 관측을 무효화하며 차량은 Pose 만료로도 정지한다.
Mapping과 Localization은 동일 calibration의 frame_id를 사용한다.

이번 다중 로봇 실시간 경로는 **실측 태그 크기 기반 PnP**를 사용한다.
앞선 휴대폰 태그의 크기 미상 Depth 진단 경로는 별도 `tagcheck` 도구로 유지한다.

## 7. 검증 결과

실제 차량·D455를 사용하지 않은 전체 회귀/통합 검사에서 **38개 테스트 통과**.
기존 14개, Fleet/전송/안전 검사 21개, 다중 태그 검사 3개를 포함한다.

```bash
MPLBACKEND=Agg python -m unittest discover -s tests -v
```

검증한 흐름:

- 실제 localhost TCP 연결 2개, 개별 Pose와 동시/독립 명령.
- A에만 MOVE → A 도착·정지·대기, B는 명령 없이 IDLE.
- 완료 뒤 새 명령이 있어야 재이동하고 자동 목표가 생기지 않음.
- 이동 중 목표 교체, STOP, 진행 중 A* 이후 늦은 결과 무효화.
- 한 차량의 연결 단절/태그 소실과 다른 차량의 정상 도착.
- 재연결/장애물 해소 후 자동 재출발하지 않음.
- 잘못된 ID·좌표·frame/version, Unknown/Occupied 목표와 경로 실패.
- 중복 명령·중복 ARRIVED·뒤늦은 MOVING, 프로세스 재시작 후 명령 중복 방지.
- 중복 robot_id/잘못된 등록 거부, 분할/합쳐진/잘린/과대/무응답 프레임.
- Pose 수신 만료와 독립 thread watchdog.
- 한 번의 다중 태그 검출, 서로 다른 크기·중심·방향 오프셋, 한 태그 소실.

별도 `mock-fleet`와 `send-command` **서로 다른 프로세스**로도 A MOVE → ARRIVED → IDLE,
B IDLE·command_id 없음까지 확인했다. 결과는 Git 제외 파일 `data/fleet_cli_smoke.json`에 저장했다.
이는 실차 성공률·주행 정확도·실제 네트워크 지연·실측 다중 태그 정확도 평가가 아니다.

## 8. 추가/수정 파일

새 파일:

- `config/robots.json`: 다중 차량 태그 예시 설정.
- `src/pi_car/pc/fleet.py`: 외부 API, 차량별 상태, 결과 대기.
- `src/pi_car/pc/commands.py`: WES 형식 요청 어댑터.
- `src/pi_car/pc/server.py`: PC의 다중 차량 TCP 세션.
- `src/pi_car/pc/live.py`: D455 다중 Pose 전송 loop.
- `src/pi_car/vehicle/agent.py`: Pi 명령 실행·A*·상태 전이.
- `src/pi_car/vehicle/client.py`: Pi의 TCP·재연결·제어 loop.
- `src/pi_car/vehicle/hardware/watchdog.py`: 독립 thread 정지 감시.
- `src/pi_car/transport/protocol.py`: 새 메시지 스키마·검증.
- `src/pi_car/transport/tcp.py`: 스트림 프레이밍·양방향 송수신.
- `src/pi_car/transport/journal.py`: SQLite 명령 ID/결과 기록.
- `src/pi_car/transport/commands.py`: localhost 외부 명령 테스트 포트.
- `src/pi_car/mock.py`: 목표를 생성하지 않는 Mock Fleet.
- `src/pi_car/fleet_cli.py`: Fleet/차량/명령/Mock CLI.
- `tests/test_fleet.py`, `tests/test_multi_localization.py`: 추가 검사.
- `docs/multi_robot_commands.md`: 이 문서.

기존 파일 수정:

- `pc/localization.py`: 기존 자세 계산 재사용 및 한 번의 다중 검출.
- `vehicle/runtime.py`: 경로 폐기·정지 `halt()` 추가.
- `common/messages.py`: Pose의 boolean/숫자 형식 검사 강화.
- `transport/codec.py`: 기존 pose codec 유지, 새 메시지 codec 노출.
- `transport/__init__.py`: 현재 기능 설명 갱신.
- `cli.py`: 기존 명령 유지, 새 명령 라우팅.
- `tests/test_stack.py`: epoch 시각의 초↔ms 반올림을 고려한 1μs 테스트 허용 오차.
- `docs/usage.md`, `architecture.md`, `implementation_and_results.md`: 최신 기능 안내.

`planning/astar.py`, `control/waypoint.py`, `common/grid.py`는 중복 구현 없이 그대로 재사용한다.
저장소 루트 README는 변경하지 않았다.

## 9. 실차 구동 전 필요한 작업

1. 고정 카메라·실측 태그 크기·차량 중심 보정으로 여러 태그의 좌표/방향 오차를 평가한다.
2. PC/Pi 시각 동기화, 실험망 timeout·처리 속도·전송 지연을 측정한다.
3. 같은 원본 지도·버전·차체 여유 값을 각 Pi에 배포한다.
4. 센서 sampling 작업을 만들어 새 `RangeReading`을 전달한다. 원본 gpiozero 캐시의 실제 갱신
   및 서보 이동 이후 방향별 fresh echo 여부를 확인해야 한다.
5. 실측 모터 방향·출력 제한·정지거리·회전 범위를 확인한 `AdeeptMotor`를 명시적으로 주입한다.
   CLI의 DisabledMotor를 우회하는 실제 구동 연결은 이번 작업에서 활성화하지 않았다.
6. 스레드 watchdog 외 하드웨어 fail-off·긴급 정지를 마련하고 낮은 출력부터 시험한다.
7. 센서/통신 오류, 도착 오차와 성공률을 실차로 평가한다. 장애물 회피·로봇 간 충돌 회피는 별도 후속 작업이다.
