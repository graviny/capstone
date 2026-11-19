# 실차 목표점 주행 결과와 재실행 방법

2026-11-19 실제 D455와 Adeept 차량 1대로 시험했다. PC가 MOVE를 보내고 Pi가 A*를
실행한 뒤 모터로 이동해 ARRIVED를 보고했다. 검증 범위는 **짧은 직선 목표 1건**이다.
큰 회전·장애물 우회·임의 방향의 클릭 주행 전체를 검증한 것은 아니다.

## 관측 결과

위치와 거리는 AprilTag의 카메라 추정값이다. 독립적인 실측 기준으로 정확도를 평가하지 않았다.

| 항목 | 결과 |
| --- | --- |
| 차량 / 실제 태그 | robot_id `1` / AprilTag 36h11 ID `42`, 검은 변 7cm |
| 최초 위치 (m) | x=-0.02630, y=-0.07185 |
| 목표 (m) | x=-0.00346, y=-0.01637: 최초 위치에서 전방 6cm |
| 최종 위치 (m) | x=-0.00861, y=-0.02608 |
| 최초→최종 변위 | 약 4.91cm: 여러 시험에 걸친 위치 차이이며 경로 길이는 아님 |
| 마지막 성공 시험 | 약 2.8cm 이동, 약 3.21초 |
| 최종 목표 오차 / 도착 반경 | 약 1.10cm / 1.35cm |
| 결과 | `ARRIVED`, `reason=GOAL_REACHED` |
| 성공 command_id | `real-test.990c61c546664f778da696db2041ee48` |
| 최대 모터 출력 | `--max-output 0.20`: 속도 m/s가 아닌 출력 한도 |
| 종료 | 추가 STOP 후 active command 없음, watchdog 비활성 확인; 실행기 종료 |

![실제 출발·도착 사진과 관측 경로](../data/real_drive_result/actual_before_after.png)

로컬 증거 파일:

- `data/real_drive_result/actual_before_after.png`: 실제 출발·도착 촬영과 좌표 경로
- `data/real_drive_result/actual_before_after.gif`: 실제 두 프레임의 교대 비교. 연속 영상은 아님
- `data/real_drive_result/summary.json`: 좌표와 ARRIVED 원문
- `data/real_trial_01/` ~ `06/`: 각 시험 보고서와 실제 RGB-D 캡처

`data/`는 Git 제외 대상이어서 clone으로 복사되지 않는다. 이 문서에 주요 수치를 별도로 남겼다.

## 문제 원인과 수정 내용

1. 기존 Adeept WebServer가 초음파 GPIO를 사용하면 `GPIO busy`가 발생한다.
   실제 주행 전 `Adeept_Robot.service`를 중지하고 실행기는 하나만 사용한다.
2. PC와 Pi가 서로 다른 Wi-Fi 대역이어서 이번 시험은 PC Tailscale 주소
   **100.124.4.8**로 연결했다. `192.168.0.10`은 이 환경의 실제 PC 주소가 아니다.
3. D455 재시작 후 focal 값이 약 0.3% 변해 기존 완전 일치 검사에서 Pose가 거부됐다.
   동일 장치·해상도에서 focal 0.5%, principal point 1px 이내 변동만 허용하고
   현재 프레임의 K로 PnP를 계산한다. 이는 실측 정확도 보장이 아니다.
   카메라를 물리적으로 옮기면 지도 좌표 보정을 다시 해야 한다.
4. 출력 0.15에서는 정지 마찰로 움직이지 않는 시험이 있었다. 0.20에서 전진을 관측했다.
   0.20에서도 회전이 원활하지 않아 회전 출력·극성·차체 응답 보정이 남아 있다.
5. 짧은 직선 목표에도 격자 중심을 따라 불필요한 회전을 요구했다. Pi의 A*는 유지하고,
   시작→목표 선분이 확장 지도에서 완전히 비어 있을 때만 목표를 직접 waypoint로 사용한다.
   선분과 닿는 셀 사각형을 검사하여 모서리 통과·Unknown·지도 밖을 차단한다.
   직선이 막히면 기존 A* 경로를 사용한다. PC는 경로를 만들지 않는다.
6. STATUS와 패널의 `pose_error`로 미검출·보정·품질 오류를 표시한다.
   선택 옵션 `fleet-server --snapshot-dir data/live`는 진단용 실제 RGB-D를 저장한다.

Pose timeout 0.3초, 초음파 최신성 0.2초, 장애물 정지 거리 0.4m,
motor watchdog 0.25초, clearance 0.20m는 이번 주행을 위해 완화하지 않았다.
성공 전 초음파 BLOCKED와 진전 없음으로 중단한 시험도 있었다.

추가 파일은 `src/pi_car/vehicle/planning/line_of_sight.py`,
`tests/test_line_of_sight.py`, `tests/test_calibration_drift.py`, 이 문서다.
기존 소스 수정은 `src/pi_car/fleet_cli.py`, `src/pi_car/pc/`의
`calibration.py`, `fleet.py`, `live.py`, `localization.py`, `panel.py`, `server.py`,
`src/pi_car/vehicle/agent.py`, `tests/test_manual_panel.py`다.
설치·단일 차량 문서도 갱신했다. 루트 README는 수정하지 않았다.

## 지도가 작아서 중앙만 가능했던 이유

현재 지도는 약 0.99×0.96m, 해상도 3cm다. 20cm 확장 후 주행 가능한 셀이 25개뿐이다.
차량이 항상 중앙에 있어야 하는 원리는 없지만 현재 지도에서는 좁은 중앙만 사용할 수 있다.
다음 실험은 카메라를 높여 넓은 주행면을 관측하고 새 지도·보정을 생성하는 것이 적절하다.
지도 촬영 때 차량과 손을 빼고, 촬영 후 카메라를 고정한다.

PC 지도 `data/drive/map/`, 보정 `data/drive/calibration.json`, Pi 지도 `data/map/`.
양쪽 지도는 동일하다.

```text
frame_id:    7959030b-4f8e-43b5-ae68-6943982c9180
map_version: 210e69e6-d9e5-4452-93bc-98e70c3ade31
```

## 직접 재실행하는 순서

제가 시험용으로 실행한 PC 서버와 Pi 진단 실행기는 종료했다. 사용자 SSH 연결은 유지했다.
카메라를 움직이지 않고 현재 지도를 유지한 경우 아래대로 실행한다.
기존 패널이 열려 있다면 닫고 다시 열면 된다.

이번 Pi의 변경 소스 두 개는 시험 중 직접 복사했다. 아직 commit/push하지 않았다.
다른 장비로 clone하려면 변경을 먼저 commit/push하고 지도 파일도 별도로 복사해야 한다.
가상환경은 Git으로 복사하지 않는다. [최초 설치 안내](pc_pi_quickstart.md)를 참고한다.

**1. PC 터미널: 실제 카메라와 Fleet 서버**

```bash
cd ~/capstone/capstone/pi_car
source .venv/bin/activate
pi-car fleet-server \
  --map data/drive/map \
  --calibration data/drive/calibration.json \
  --robots config/robots.json \
  --host 0.0.0.0 \
  --clearance 0.20 \
  --journal data/fleet.sqlite3
```

`address already in use`는 기존 서버가 포트 8765/8766을 쓰는 경우다.
기존 서버 터미널에서 Ctrl+C로 종료한 뒤 하나만 실행한다.

**2. Pi 터미널: 실제 차량**

```bash
cd ~/robotics/capstone/pi_car
source .venv/bin/activate
sudo systemctl stop Adeept_Robot.service
pi-car robot-client \
  --robot-id 1 \
  --host 100.124.4.8 \
  --map data/map \
  --clearance 0.20 \
  --journal data/robot_1.sqlite3 \
  --enable-motors \
  --vendor-dir "$HOME/ADR036-Adeept_4WD_Smart_Car_Kit_for_RPi-20260317/Code/Adeept_4WD_Smart_Car_for_RPi/Server/Server_OrdinaryWheels" \
  --max-output 0.20 \
  --servo-angle 90
```

양쪽 Tailscale가 연결돼 있어야 한다. 같은 LAN으로 바꾸면 Pi에서 접근 가능한 PC IP를
`--host`에 넣는다. `REAL HARDWARE ENABLED`만으로 연결 성공을 판단하지 않는다.
PWMSoftwareFallback 경고 자체는 치명적 오류가 아니다.

**3. PC 다른 터미널: 연결 확인 후 클릭 창**

```bash
cd ~/capstone/capstone/pi_car
source .venv/bin/activate
pi-car send-command --json '{"type":"STATUS"}'
MPLBACKEND=TkAgg pi-car fleet-panel --map data/drive/map --clearance 0.20
```

차량 `1`의 `connected: true`, `pose.valid: true`, `pose_error: null`을 확인한다.
화면의 위치·방향을 실제 차량과 비교한 뒤 `1`을 선택하고 흰 영역의 가까운 전방 목표를 클릭한다.
**클릭 즉시 실제 MOVE가 전송된다.** 이전 목표는 현재 위치의 도착 반경 안이므로 다시 보내면
곧바로 ARRIVED할 수 있다. STOP selected 또는 아래 명령으로 정지한다.

```bash
pi-car send-command --json '{"type":"STOP","robot_id":"1","command_id":"manual-stop-001"}'
```

새 명령에는 새 command_id를 사용한다. 같은 ID 재전송은 다시 실행하지 않는다.
도착 후 다음 목표를 자동 발급하지 않는다. WES용 MOVE API는 그대로 유지된다.

## 코드 검증과 남은 작업

```bash
cd ~/capstone/capstone/pi_car
source .venv/bin/activate
MPLBACKEND=Agg python -m unittest discover -s tests -q
```

56개 실행: **55개 통과, 선택적 GPIO 의존성 검사 1개 생략**.
기존 다중 차량 TCP·명령 교체·중복·STOP 검사와 새 보정 변동·선분 충돌 검사를 포함한다.
GUI 통합 검사는 렌더링 뒤 최신 Pose를 재수신하도록 수정했고 실제 timeout은 유지했다.

넓은 지도 생성, 독립적인 좌표 실측, 회전 출력·극성 보정, 여러 방향·거리의 반복 주행,
실제 장애물 정지거리 평가가 남아 있다. 장애물 처리는 정지까지이며 우회·경로 복귀는
아직 구현·검증하지 않았다.
