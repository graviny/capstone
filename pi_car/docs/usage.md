# 실차 자율주행 스택

PC/Pi별 설치·실행과 지도 클릭 주행 안내: [pc_pi_quickstart.md](pc_pi_quickstart.md).

외부 MOVE/STOP 기반 멀티로봇 TCP와 Pi 실행기 사용법은
[multi_robot_commands.md](multi_robot_commands.md)에 있다. 아래는 지도·좌표 검사 도구 안내를 포함한다.

Adeept ADR036 + Raspberry Pi 4B + 천장 RealSense D455 + AprilTag 구성이다.
현재 `feature/pi_car1`의 `pi_car/`에서 개발하며, `webots/`와 코드 의존성을 공유하지 않는다.

각 알고리즘의 원논문·공식 활용 사례와 실제 구현 범위는
[알고리즘 근거](algorithm_references.md)에 정리했다.

**현재 우선 목표: 실제 Depth로 지도 생성 → 지도 좌표 확인 → AprilTag 차량 위치 확인 → 목표점 선택.**
PC의 검사 도구에는 모터 실행이나 차량으로 주행 명령을 보내는 기능이 없다.

## 구현 상태

| 부분 | 현재 제공 | 이후 할 일 |
| --- | --- | --- |
| Mapping | RGB-D 저장, 실측 기준점 보정 또는 바닥 ROI RANSAC 실험, 높이 기반 분리, 점유 지도 저장 | 실제 높이 임계값 조절, 다중 프레임 통합, 바닥 자동 선택 |
| Localization | AprilTag 36h11 검출, PnP, 월드 좌표 변환, 차량 중심/방향 보정 | 실측 오차 평가, 검출 품질·지연 기준 조절, 자세 모호성 추가 필터링 |
| 확인 화면 | RGB/지도 병렬 표시, 클릭 좌표 확인, 목표 저장, A* 미리보기 | 실험별 오차 기록, 경로/위치 로그 UI |
| Global Planning | 4방향 A*, Unknown 차단, 차체 여유 공간 확장 | Pi 성능 측정, 경로 간소화·충돌 검증 |
| Perception | Adeept 초음파 cm→m 어댑터, 거리 유효성 검사 | 실장 센서 갱신 시각·고장 검출, 주기 측정 작업 |
| Control | 규칙 기반 경유점 추종 계산, 위치/초음파 단절 시 정지 로직 | 실차 파라미터 조정, PID, Pure Pursuit |
| Local Planning | 장애물 발견 시 정지하는 확장 지점 | 좌우 스캔 → 제한적 우회 → 충돌 검사 → 기존 경로 복귀 |
| Hardware | 기본 비활성 모터, 명시적으로 주입하는 Adeept 어댑터 | 바퀴 방향 확인, 별도 모터 watchdog, 실차 통합 |
| Communication | 다중 TCP·재연결·heartbeat·외부 목표·상태·영구 ID 기록 | 실험망 지연 평가, 지도 전송 자동화 |

차량 전체 실차 주행은 아직 검증하지 않았다. `robot-client`는 Runtime을 사용하는
실행기를 제공하며 CLI의 모터는 비활성이다. 회피·경로 복귀·동적 장애물 재탐색은 후속 구현이다.

## 폴더

```text
pi_car/
├── config/
│   ├── mapping.json              공간 크기, 지도 해상도, 장애물 높이 기준
│   ├── floor_references.json     직접 측정한 바닥 기준점 XY
│   └── robot.json                태그 ID/실측 크기, 차량 중심·방향 보정
├── src/pi_car/
│   ├── common/                   미터/라디안 좌표, 지도, Pose·모터 명령·상태
│   ├── pc/
│   │   ├── camera.py             D455 RGB-D 수신/정렬/왜곡 보정/파일 저장
│   │   ├── calibration.py        카메라→주행장 강체 변환
│   │   ├── mapping.py            깊이→월드 XYZ→Occupancy Grid
│   │   ├── localization.py       AprilTag→차량 중심 Pose
│   │   └── viewer.py             지도·위치·목표점 검사 화면
│   ├── vehicle/
│   │   ├── planning/astar.py     Raspberry Pi에서 재사용할 A*
│   │   ├── planning/local.py     회피·복귀 정책 확장 지점
│   │   ├── perception/           초음파 측정 모델
│   │   ├── control/waypoint.py   규칙 기반 추종
│   │   ├── hardware/adeept.py    원본 모터/서보/초음파 어댑터
│   │   └── runtime.py            상태·안전 검사 후 하나의 모터 명령 적용
│   ├── transport/                통신 메시지 직렬화 및 향후 TCP 연결 자리
│   └── cli.py                    현재 PC 검사 명령
├── docs/                         좌표/통신 계약, Adeept 참고, 다음 구현 순서
├── tests/                        장비 없이 검증하는 회귀 테스트
└── data/                         캡처·보정·지도·선택 결과 (Git 제외)
```

## 1. PC 설치

저장소 루트에서 시작한다. Python 3.10 이상, GUI 가능한 Ubuntu 데스크톱 환경을 기준으로 한다.

```bash
cd pi_car
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[pc]'
pi-car --help
```

RealSense SDK의 장치 접근 권한/udev 설정이 필요하다. 먼저 공식 `realsense-viewer`에서
D455 RGB와 Depth가 나오는지 확인한다. `pyrealsense2` wheel이 없는 플랫폼에서는
해당 PC 환경에 맞게 공식 librealsense Python 바인딩을 설치해야 한다.
ROS는 현재 필요하지 않다. PC에서 OpenCV GUI 패키지는 하나만 설치한다.

카메라 방향과 깊이를 보정 전에 확인하려면 `pi-car preview --live`를 사용한다.
저장 캡처는 `pi-car preview --capture data/scene.npz --out data/hardware_check --show`로
열 수 있으며 RGB/Depth 이미지와 깊이 유효 비율이 저장된다. GUI 없는 환경에서는
`MPLBACKEND=Agg`를 지정하고 `--show`를 생략해 PNG만 생성한다.
실장 D455에서 확인한 inverse Brown-Conrady 렌즈 모델도 SDK 투영으로 처리한다.

카메라 없이 저장된 자료를 확인하는 PC에는 `pip install -e '.[vision]'`,
계획·제어 계산만 사용할 Pi에는 `pip install -e .`로 시작할 수 있다.

## 2. 실제 공간과 설정 준비

1. 카메라를 고정한다. 보정 이후 카메라를 움직이면 재보정 및 지도 재생성이 필요하다.
2. 주행장 원점과 +X/+Y 방향을 정한다. Z는 위쪽, 좌표계는 오른손 좌표계다.
3. 넓게 떨어진 바닥 기준점 4개 이상을 줄자로 측정해 `config/floor_references.json`에 넣는다.
   예시의 3×2m를 그대로 쓰지 말고 실제 값으로 교체한다. 점들은 일직선이면 안 된다.
4. `config/mapping.json`의 공간 크기·원점·해상도를 실제 주행장에 맞춘다.
5. `config/robot.json`의 태그 크기/ID/중심 오프셋을 실측값으로 바꾼다.

태그 크기는 **바깥 흰 여백을 제외한 검은 정사각형의 한 변**이다. 예시 8cm는 가정이다.
태그 좌표의 +X는 인쇄 태그의 오른쪽, +Y는 위쪽, +Z는 인쇄면 바깥이다.
`center_in_tag_m`는 태그에서 차량 기준 중심까지의 벡터다. 예시 `[0,0,-0.10]`은
태그 아래 10cm를 가정한다. `heading_offset_rad`는 태그 +X에서 차량 전방까지의 CCW 각도다.
차량을 실제 +X, +Y로 향하게 놓고 화면의 0°, 90°를 반드시 비교한다.

## 3. 기준점 보정

바닥 기준점이 모두 보이는 상태로 캡처한다.

```bash
pi-car capture --out data/floor.npz
pi-car calibrate --capture data/floor.npz \
  --references config/floor_references.json --out data/calibration.json
```

이미지에서 JSON에 적은 순서대로 기준점을 클릭한다. 기준점은 모두 실제 바닥 위에 있어야 한다.
깊이 패치에 장애물 모서리나 다른 높이가 섞이지 않게 한다. 창을 닫으면 취소된다.
4개 이상의 RGB 픽셀 깊이를 이용해 카메라 XYZ와 실제 바닥 XYZ(Z=0)의 강체 변환을 구한다.
이는 **실측 기준점으로 바닥 평면을 정의하는 방식**이다. 실측값 없이 실행하는
바닥 ROI RANSAC 실험은 아래 별도 명령을 사용한다.
보정 RMSE가 기본 3cm보다 크거나 좌표계가 뒤집혔으면 저장을 거부한다.
RMSE는 사용한 점들의 맞춤 오차이므로 별도 검증점에서의 정확도를 보장하지 않는다.

`calibration.json`에는 변환 행렬, 장치 serial, 내부 파라미터, 좌표계 ID가 저장된다.
카메라 위치나 기준점을 바꿨다면 새 보정 파일로 지도를 다시 만든다.

## 4. RealSense 지도 생성

장애물을 배치하고 **차량·사람을 주행장에서 빼고** 캡처한다.
차량이 있으면 차량도 정적 장애물로 저장된다. 카메라는 보정할 때의 위치를 유지한다.

```bash
pi-car capture --out data/scene.npz
pi-car map --capture data/scene.npz --calibration data/calibration.json \
  --config config/mapping.json --out data/map
pi-car inspect --map data/map --clearance 0.20
```

출력은 `map.npy`, `map_metadata.json`, 보정·매핑 설정 사본이다.
원본 지도는 -1 Unknown, 0 Free, 1 Occupied이며 배열은 `[row=Y, column=X]`다.
깊이가 유효하고 바닥 높이와 가까운 관측이 있어야 Free가 된다.
높이 구간 사이의 애매한 점, 깊이 구멍, 시야 밖, 가려진 곳은 임의로 Free로 채우지 않는다.
동일 셀에서 장애물이 관측되면 바닥보다 장애물을 우선한다.

`--clearance`는 **차체를 감싸는 원의 반지름 + 안전 여유(m)**다. 0.20m는 실측값이 아니다.
계획 시 장애물·Unknown·지도 경계를 확장하며, 격자 양자화 여유도 추가한다.
지도 파일은 원본 관측을 보존하고 확장은 별도로 계산한다.
지도 오른쪽/위쪽 끝 좌표 자체는 범위 밖이다. 3×2m, 5cm 해상도이면
유효 셀 중심은 마지막 X=2.975, Y=1.975m다.

초기 지도는 단일 프레임 기반이다. 보이지 않는 장애물의 밑면, 얇은 장애물, 탁자 아래 공간은
완전히 추론하지 못한다. 첫 실험은 상자로 진행하며 실제 점유 영역과 비교한다.
잡음 때문에 Free 공간이 많이 끊기면 깊이·보정·해상도를 먼저 확인한다.

## 5. 지도에서 좌표와 목표 확인

검사 창의 지도에서:

- **우클릭:** 시험용 출발점 설정.
- **좌클릭:** 목표점 설정. Free 및 차체 여유가 확보된 셀만 선택된다.
- 출발점과 목표점이 있으면 A* 경로를 파란색으로 표시한다.
- `r`: 수동 출발점을 해제하고 AprilTag 위치를 사용한다.
- 회색은 Unknown, 검정은 장애물, 주황은 계획에서 확장해 막은 공간이다.

목표는 선택한 셀의 중심으로 맞추며 `data/inspection/goal.json`에 저장한다.
사용자가 클릭한 원래 좌표도 함께 기록한다. `path.json`은 미리보기 경유점이다.
이 도구에서 PC의 A*는 **검증용 미리보기**다. 실제 주행 경로 계산은 이후 Pi의
동일 `vehicle/planning/astar.py`를 사용한다. 파일 저장은 주행 시작을 의미하지 않는다.

RGB를 함께 보고 픽셀→실제 좌표를 확인하려면:

```bash
pi-car inspect --map data/map --capture data/scene.npz \
  --calibration data/calibration.json --robot-config config/robot.json
```

RGB의 한 점을 클릭하면 터미널에 해당 표면의 XYZ(m)를 출력한다.
바닥에 가깝고 지도에서도 안전한 점이면 목표로 저장한다. 장애물 위를 클릭하면
표면 높이가 0이 아니므로 목표 선택을 거부한다.

## 6. 실시간 AprilTag 위치 확인

지도를 만든 뒤 태그를 부착한 차량을 놓는다. 모터 전원은 필요 없다.

```bash
pi-car inspect --map data/map --live \
  --calibration data/calibration.json --robot-config config/robot.json \
  --clearance 0.20 --out data/inspection
```

RGB와 지도 위 차량 중심(초록 점), 전방 방향(초록 선)을 표시한다.
위치가 보이면 해당 위치를 출발점으로 목표까지의 경로를 미리 계산한다.
태그가 사라지면 초록 표시와 해당 출발점 경로를 지우고 `pose.json`을 `valid:false`로 바꾼다.
검사에는 최근 영상만 사용하며 실제 차량 주행 메시지는 전송하지 않는다.
`pose.json`은 검사 결과 파일이지 통신 채널이 아니다. 프로세스 종료 시 파일은 남으므로
후속 소비자는 반드시 타임스탬프와 유효성을 함께 확인해야 한다.

기록 캡처를 사용하는 모드의 pose 타임스탬프는 원래 캡처 시각을 유지한다.
실시간 위치로 취급하면 안 된다. 실시간 SDK는 global/system 시계 기반 프레임만 허용한다.

## 7. 첫 실험 체크

- 바닥의 독립적인 5개 이상 검증점을 클릭하고 줄자로 측정한 좌표와 차이를 기록한다.
- 상자의 네 모서리와 지도의 검정 영역을 비교한다.
- 차량을 검증점 위에 놓고 `(x,y)` 오차를 기록한다.
- 차량을 +X/+Y/-X/-Y 방향으로 놓고 방향 표시를 비교한다.
- 장애물·Unknown·경계 가까이 목표를 클릭했을 때 거부되는지 확인한다.
- 태그를 가리면 이전 위치를 계속 유효하게 표시하지 않는지 확인한다.
- 검증점 위치 오차, 방향 오차, 태그 검출 실패 사례를 남긴 뒤 주행으로 넘어간다.

## 실측 기준점 없이 먼저 끝까지 시험하기

현재 영상의 **바닥임이 확인된 영역**을 RGB 픽셀 사각형으로 지정하면 RANSAC으로
바닥 평면을 추정하고 로컬 좌표계·지도·시험 목표·A* 경로·보고서 이미지를 생성한다.
예시 ROI는 실험 01의 카메라 배치에만 해당한다. 위치가 바뀌면 새 캡처에서 다시 선택한다.

```bash
pi-car capture --out data/experiment_01/scene.npz
MPLBACKEND=Agg pi-car experiment --capture data/experiment_01/scene.npz \
  --floor-roi 320 180 450 400 --out data/experiment_01 \
  --robot-config config/robot.json --clearance 0.20
```

`experiment.png`의 초록 바닥, 노란 fitting 영역, 높이 이미지, 실제 지도/경로를 확인한다.
`report.json`에는 평면 맞춤 잔차와 지도·경로·태그 검출 결과가 저장된다.
경로가 안 나오면 `planning_error`를 기록하고 경로는 빈 목록으로 저장한다.
흰색 Free, 검정 장애물, 회색 Unknown, 주황은 차체 여유 확장 영역이다.
이후 `pi-car inspect --map data/experiment_01/map`으로 다른 목표를 선택할 수 있다.

원점은 선택 ROI의 바닥 점들 중앙 부근이며, +X는 카메라 오른쪽 방향을 바닥에 투영한 축이다.
**기존 실측 월드 좌표계와 다르다.** 평면 맞춤 잔차는 실측 좌표 정확도를 의미하지 않는다.
최대 연결 Free 영역에서 자동으로 선택한 시험 출발점은 차량 위치가 아니다.
AprilTag가 검출되지 않으면 위치 추정은 검증 완료로 표시하지 않는다.
실험 결과와 한계는 [실장 실험 01](experiment_01.md)에 기록했다.

## 테스트

태그 실제 크기를 모를 때에는 저장 RGB-D로 중심 위치를 확인할 수 있다.

```bash
MPLBACKEND=Agg pi-car tagcheck --capture data/experiment_02/scene.npz \
  --calibration data/experiment_02/calibration.json --map data/experiment_02/map \
  --out data/experiment_02
```

`tagcheck.png`, `tag_report.json`에 ID·검출 픽셀·Depth 중심 좌표가 저장된다.
태그의 방향 표시는 태그가 바닥과 평행하다는 가정의 근사값이다.
실측한 검은 정사각형 변 길이가 있으면 `--tag-size 0.05`처럼 m로 전달하여 PnP도 비교한다.
예시 크기를 실제 측정 없이 사용하면 안 된다. 단일 캡처의 Depth 분포는 시간적 안정성이나
실제 위치 오차를 의미하지 않는다.

지도는 캡처 뒤에도 저장돼 있지만, **카메라를 움직이면 기존 카메라→지도 보정을 그대로
실시간 Localization에 사용할 수 없다.** 저장 영상 분석 후에는 카메라를 옮겨도 된다.
실시간 위치 추정을 재개하려면 카메라를 원래 고정 위치에 두거나, 지도에 알려진 기준점으로
새 카메라 자세를 다시 보정해야 한다. 기준점 없이 평면 보정을 새로 하면 원점·축도 바뀌므로
기존 지도/목표 좌표와 직접 비교할 수 없다.

장비 없이 좌표 보정·Depth 매핑·지도 저장·A*·목표 선택·합성 AprilTag 영상·정지 로직을 검증한다.

```bash
MPLBACKEND=Agg python -m unittest discover -s tests -v
```

테스트 통과는 실장 D455/모터의 동작 확인을 대신하지 않는다.
후속 작업은 [architecture.md](architecture.md), 원본 연결 정보는
[adeept.md](adeept.md)를 참고한다.

공식 참고: [RealSense RGB/Depth 정렬 예제](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/align-depth2color.py),
[RealSense 투영 좌표](https://dev.realsenseai.com/docs/projection-in-realsense-sdk-2-0/),
[AprilTag 자세 추정](https://github.com/AprilRobotics/apriltag).
