# 역할과 확장 지점

현재 외부 명령·다중 로봇 TCP·Pi 실행기의 상세 구조와 실행법은
[multi_robot_commands.md](multi_robot_commands.md)에 정리했다.

## 데이터 흐름

```text
PC: D455 → camera → calibration → mapping → map.npy + metadata
                   └→ localization → Pose (world frame)
                                        │ 향후 TCP
Pi: map + Pose + goal → A* → WaypointFollower → MotorCommand
                  ultrasonic → safety / LocalPlanner ─┘
                         VehicleRuntime → single MotorDriver
```

## 좌표 계약

모든 내부 위치는 m, 각도는 rad이다. XY는 바닥, +Z는 위, theta는 +X에서 CCW이다.
카메라 광학 좌표는 X 오른쪽, Y 아래쪽, Z 전방이다.
camera.py는 Depth를 RGB로 정렬하고 두 영상을 동일한 RGB 렌즈 모델로 보정한다.
따라서 저장된 RGB/Depth의 픽셀과 pinhole 내부 파라미터가 일치한다.
깊이는 카메라 optical Z이며 카메라에서 점까지의 유클리드 거리가 아니다.

바닥 실측 기준점의 rigid fitting으로 `T_world_camera`를 구하고 Mapping과 Localization이 공유한다.
매번 보정할 때 새로운 `frame_id`를 발급한다. 지도와 Pose의 frame ID가 다르면 사용을 거부한다.
지도 갱신은 `version`을 바꾸며, goal에는 frame과 map version을 같이 저장한다.
차량에서는 지도 교체 시 정지하고 기존 경로를 폐기한 뒤 새 지도로 다시 계획해야 한다.

PnP는 태그→카메라 자세를 구하고 태그 좌표의 차량 중심 오프셋을 적용한다.
태그 +X가 차량 전방이 아니면 heading offset을 설정한다. 평면 태그의 자세 모호성이
남아 있으므로 실제 캡처에서 오차·프레임 간 점프를 평가하고 주행 전에 필터를 추가한다.

## 지도 계약

`map.npy`는 int8, shape `(height, width)`, 값 -1/0/1만 허용한다.
원점은 셀 (0,0)의 바깥쪽 왼쪽 아래 경계이고, 경로는 셀 중심 좌표를 사용한다.
raw 지도에는 팽창을 중복 적용하지 않는다. Pi도 동일한 차체 반경/여유로 nav grid를 만든다.
최대 장애물 높이는 관측 환경에 맞춰 조절해야 하며 차량의 충돌 높이보다 낮게 두면 안 된다.

## 통신 계약

초기에는 지도 폴더를 파일로 전달하고 PC와 Pi가 같은 metadata를 로딩한다.
실시간 연결은 TCP + UTF-8 JSON Lines로 구현했다. 새 프로토콜은 대문자 POSE/GOAL/STOP과
frame/map 검증·명령 ID·상태 보고를 사용한다. `codec.py`의 아래 소문자 pose 형식은 기존
도구 호환용이며 Fleet wire 형식과 구분한다. 분할 수신·최대 길이·재연결·heartbeat와
명령 중복 방지는 구현했고 인증/TLS는 제공하지 않는다.

```json
{"type":"pose","schema_version":1,"robot_id":"B","x":1.2,"y":0.8,"theta":1.5708,"timestamp":1790000000.0,"frame_id":"calibration-uuid","valid":true}
```

`timestamp`는 PC 촬영 시각의 UNIX seconds다. PC와 Pi 시계를 동기화하고,
Pi는 캡처 시각과 로컬 수신 monotonic 시각을 모두 추적해야 한다.
PC의 검출 실패는 즉시 invalid로 전송하고 Pi는 연결 단절과 수신 만료에도 정지한다.
메시지에는 goal/map 버전·명령 ID·중복 처리·상태 응답을 추가한다.

## 차량 제어 통합 순서

1. 기존 Adeept Move/Ultra/RPIservo를 별도 hardware bootstrap에서 초기화한다.
2. 모터 없이 `DisabledMotor`로 pose/range/goal 수신 및 상태 전이를 확인한다.
3. Pi가 지도를 로딩하고 nav grid에서 A*를 실행해 `runtime.set_path()`로 전달한다.
4. 센서 작업은 새 초음파 값과 실제 서보 방향을 주기적으로 갱신한다.
5. 제어 루프는 `tick()`만 모터를 소유하도록 한다. vendor 자동주행 스레드를 함께 실행하지 않는다.
6. 연결·위치·센서 오류, 예외, 프로세스 종료에 정지한다. `finally`에서 stop/destroy 한다.
7. 제어 루프가 멈춰도 모터가 꺼지는 별도 watchdog을 구현한 뒤 실제 구동을 연결한다.

`runtime.py`는 정상 경로 추종과 장애물/오류 정지까지만 연결한다.
`LocalPlanner`는 장애물에서 정지만 반환하며 아직 runtime에 회피 상태 전이를 연결하지 않았다.
`AVOIDANCE`, `REJOIN`, `REPLANNING`은 확장할 상태 이름이다.

## Local Planning 구현 순서

우선 정지 → 좌우 ±45° 서보 스캔 → 새 측정 확보 → 차체 폭/회전 공간 검사 →
짧고 제한된 회피 → 최신 pose와 지도 기반으로 접근 가능한 기존 경유점 선택 순서로 만든다.
진행 중 장애물 감지나 pose 만료 시에는 즉시 정지한다. 지도상 시야 밖/Unknown에 진입하지 않는다.
앞 초음파 한 개는 회전 시 측면/후면 안전을 보장하지 않는다. 단순히 더 먼 쪽으로 회전하면 안 된다.

복귀가 불가능하면 pose + 센서 방향 + 거리로 동적 장애물 후보를 차량 로컬 지도에 투영하고,
불확실도 확장 및 TTL을 적용한 뒤 A*를 재실행한다. 원본 정적 지도를 덮어쓰지 않는다.
갱신 없이 A*를 반복하는 것은 같은 경로를 만들 수 있다.

## 이후 제어

PID는 방향 오차에 따른 회전 출력을, Pure Pursuit는 차체 특성에 맞는 좌우 출력을 만든다.
현재 모터 출력은 정규화된 duty 명령이고 m/s가 아니다. 출력/속도·정지거리·회전 특성을 실측한다.
차량의 실제 바퀴가 일반형인지 메카넘인지 확인하고 어댑터를 선택한다.

캡스톤의 작업 배정/선택적 정보 공유는 향후 goal 입력과 상태/관측 출력 계층에 붙인다.
모터 제어와 통신 정책을 분리하여 로컬 안전 정지가 상위 작업 명령보다 우선하도록 유지한다.
