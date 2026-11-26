# 2026-11-22 실차 직진 시험과 출력 설정

사용자의 촬영 시작 확인을 받은 뒤, 새 지도와 실제 차량 1대로 주행했다.
PC는 위치와 목표만 전달하고 Pi가 A*와 기존 규칙 기반 WaypointFollower를 실행했다.
이번 시험에서 지도·위치 timeout·장애물 정지 기준은 완화하지 않았다.

## 실제 주행 결과

| 항목 | 관측 결과 |
| --- | --- |
| 차량 / 태그 | robot_id `1`, AprilTag 36h11 ID `42`, 검은 변 7cm |
| 출발 위치 (m) | (-0.11053, 0.00651) |
| 목표 위치 (m) | (-0.01201, 0.01131), 전방 약 10cm |
| 최종 위치 (m) | (-0.02415, 0.01044) |
| 관측 변위 | 약 8.65cm |
| 목표 오차 / 도착 반경 | 약 1.22cm / 1.35cm |
| 도착 보고까지 시간 | 약 9.31초 |
| 출력 | 경로 추종 0.20, 모터 상한 0.20 |
| 실행 결과 | `ARRIVED / GOAL_REACHED`, 이어서 STOP 확인 |
| 명령 ID | `real-drive.b8922ec7b7d4482ba330a30793d43c12` |

거리는 AprilTag를 이용한 카메라 추정값이다. 별도 실측 기준으로 좌표 정확도를 평가하지 않았다.
변위를 전체 실행 시간으로 나눈 값은 약 0.93cm/s다. 실제 최고 속도 측정치는 아니다.
이번 결과는 짧은 직선 1회이며 큰 회전·장애물 우회·장시간 주행 검증은 아니다.

![실제 출발 전·주행 말기 사진과 관측 경로](../data/session_20261122/drive_result.png)

위 사진은 실제 저장 프레임이다. 오른쪽 촬영은 도착 직전의 프레임이며 연속 영상은 아니다.
목표 오차와 최종 좌표는 사진 시각이 아닌 주행 보고서의 최종 Pose로 계산했다.

## 새 지도와 위치 확인

최초 촬영은 깊이 유효 비율이 약 31%였고 안전 확장 후 이동 가능한 셀이 없었다.
카메라 배치 변경 후 약 79%로 개선됐고, 마지막 위치 변경에 맞춰 다시 생성한 지도의
깊이 유효 비율은 약 72.5%였다. 바닥 평면 fitting RMS는 약 2.26mm다.
이는 평면의 적합 잔차이며 지도 전체의 실측 정확도를 뜻하지 않는다.

최종 지도는 35×27셀, 해상도 3cm, 크기는 약 1.05×0.81m다.
clearance 20cm 적용 후 주행 가능한 셀은 14개로, 주행 범위는 여전히 좁다.
실제 Pose와 주행 가능한 셀, 출발→목표 선분을 확인하고 출발했다.

```text
PC 지도: data/session_20261122/drive_retry_02/map
PC 보정: data/session_20261122/drive_retry_02/calibration.json
Pi 지도: data/session_20261122/map
frame_id: 4f1c87a7-8faa-4989-814f-8e1169ebd271
map_version: 84b40305-e15b-4228-9dc4-aaa89ca52ce3
```

카메라가 움직이면 새 좌표 보정과 지도가 필요하다. 11월 19일 지도를 이번 시험에 사용하지 않았다.
Pi에 코드 44개를 비교해 PC와 같은 상태임을 확인한 뒤 새 지도를 별도 폴더에 복사했다.

## 주행 출력 옵션 추가

실차 시험 후 사용자가 속도 증가를 요청하여 `robot-client --drive-output`을 추가했다.
기존 `--max-output`은 하드웨어 상한이므로 이것만 높여도 기본 0.20인 추종 명령은 커지지 않았다.

```bash
--drive-output 0.25 --max-output 0.25
```

두 값은 속도 m/s가 아닌 정규화된 모터 출력이다.
`--drive-output` 생략 시 기본은 0.20이며, 기존처럼 더 낮은 `--max-output`을 지정하면
그 상한을 따른다. 명시적인 drive-output은 하드웨어 상한 이하여야 하며 현재 실차 상한 0.30은 유지했다.

사용자 요청에 따라 주행 중에는 설정 출력을 일정하게 사용한다.
작은 경로 오차에 따른 감속, 자동 속도 조절 기능은 추가하지 않았다.
기존 방향 오차 기반 회전·전진과 장애물·위치 유실·통신 끊김 안전 정지는 유지한다.
**위 0.25 설정의 실제 주행 속도는 아직 시험하지 않았다.**

변경 파일:

- `src/pi_car/fleet_cli.py`: 옵션, 입력 검증, Pi 실행기로 출력 전달
- `src/pi_car/vehicle/agent.py`: Runtime에 설정 출력 전달
- `src/pi_car/vehicle/runtime.py`: 기존 Follower에 설정 출력 전달
- `tests/test_drive_output.py`: 출력 반영, 장애물 정지 유지, 기존 낮은 상한 호환, 잘못된 출력 거부 검사

기본 Adeept vendor 코드와 루트 README는 수정하지 않았다. 이번 변경의 commit/push는 하지 않았다.
수정 소스 3개는 Pi에도 직접 복사했고, 원본은 Pi의 `/tmp/pi-car-before-speed-20261122.tar`에 보관했다.

## 재실행 명령

PC의 기존 Fleet 서버가 켜져 있으면 중복 실행하지 않는다. 아래는 이번 지도 기준 명령이다.
카메라를 현재 위치에 고정한 상태여야 한다.

PC 터미널:

```bash
cd ~/capstone/capstone/pi_car
source .venv/bin/activate
pi-car fleet-server \
  --map data/session_20261122/drive_retry_02/map \
  --calibration data/session_20261122/drive_retry_02/calibration.json \
  --robots config/robots.json --host 0.0.0.0 --clearance 0.20 \
  --journal data/session_20261122/fleet.sqlite3
```

Pi 터미널, 출력 0.25를 선택할 경우:

```bash
cd ~/robotics/capstone/pi_car
source .venv/bin/activate
sudo systemctl stop Adeept_Robot.service
pi-car robot-client \
  --robot-id 1 --host 100.124.4.8 \
  --map data/session_20261122/map --clearance 0.20 \
  --journal data/session_20261122/robot_1.sqlite3 --enable-motors \
  --vendor-dir "$HOME/ADR036-Adeept_4WD_Smart_Car_Kit_for_RPi-20260317/Code/Adeept_4WD_Smart_Car_for_RPi/Server/Server_OrdinaryWheels" \
  --drive-output 0.25 --max-output 0.25 --servo-angle 90
```

기존 속도로 재현하려면 두 출력을 모두 0.20으로 지정한다.
실행만으로 출발하지 않으며 새로운 MOVE를 받아야 이동한다.

PC의 별도 터미널, 실제 클릭 명령 창:

```bash
cd ~/capstone/capstone/pi_car
source .venv/bin/activate
pi-car send-command --json '{"type":"STATUS"}'
MPLBACKEND=TkAgg pi-car fleet-panel --map data/session_20261122/drive_retry_02/map --clearance 0.20
```

연결과 유효한 최신 Pose를 확인하고 목표를 클릭한다. 클릭은 즉시 MOVE를 보낸다.
지금 차량은 첫 목표 근처에 있어 같은 방향으로 10cm를 더 갈 지도 공간이 없다.
다음 속도 시험은 먼저 출발 위치로 수동 복귀한 뒤 현재 Pose와 목표를 다시 검사해야 한다.

## Adeept 기본 자동주행과 전환

기본 자동주행을 사용하겠다는 요청에 따라 시험용 Pi 실행기를 종료했다.
그 상태에서 아래처럼 기존 서비스를 다시 켤 수 있다.

```bash
sudo systemctl start Adeept_Robot.service
```

두 차량 실행기가 GPIO·모터를 동시에 사용하지 않도록 지도 기반 실행기와 기존 서비스를 번갈아 사용한다.
PC 코드 수정은 Pi 기본 자동주행의 vendor 파일을 변경하지 않는다.

## 검사와 기록

```bash
cd ~/capstone/capstone/pi_car
source .venv/bin/activate
MPLBACKEND=Agg python -m unittest discover -s tests -q
```

최종 검사: **59개 실행, 58개 통과, 선택적 GPIO 검사 1개 생략**.
실차 로그와 사진은 `data/session_20261122/`에 보관했다. `data/`는 Git 제외 대상이다.
`drive_result_summary.json`에 관측 수치, `actual_drive_223645/report.json`에 전체 상태 기록이 있다.
`final_telemetry_20.json`은 도착 순간이 아니라 실행기를 종료하기 직전의 정지 상태 기록이다.
