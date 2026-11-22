# 차량 1대·중앙 장착·7cm 태그 설정

최신 실제 결과와 현재 PC/Pi 경로·IP를 반영한 명령은
[11월 22일 실차 시험 및 출력 설정](real_drive_20261122.md)에 정리했다.
아래의 `data/drive` 경로는 이전 시험 지도이므로 최신 지도 경로와 구분한다.

현재 config/robots.json과 config/robot.json은 다음 조건에 맞췄다.

- 차량 이름(robot_id): 문자열 `"1"`. Pi도 `--robot-id 1`로 실행한다.
- 태그: AprilTag 36h11, **ID 42**. 사용자가 흰 여백을 추가한 인터넷 태그를 실제 촬영하여 확인했다.
- 크기: 흰 여백 제외 검은 정사각형 한 변 7cm (`0.07m`). 실제 출력 후 재측정한다.
- 장착: 태그 중심을 차량 평면 중심으로 사용한다. 미측정 예시 높이 -10cm는 제거했다.
- 방향: 사용자가 촬영 사진의 왼쪽을 차량 전방으로 확인했다.
  검출 꼭짓점 순서에서 태그 +X는 사진 아래, +Y는 사진 오른쪽이므로
  차량 전방은 태그 -Y다. **heading_offset_rad=-π/2 (-90°)**로 맞췄다.
  이는 태그와 차체 사이의 고정 장착 보정이며 월드 방향 theta가 항상 -90°라는 뜻은 아니다.

방향 표시는 저장 영상에 재투영해 왼쪽을 향하는지 확인한다.
실제 바퀴 회전/모터 polarity는 별도로 확인해야 하며, 이 설정 검사가 실차 주행 검증은 아니다.

## 현재 촬영 결과

### 흰 여백 추가 후 재촬영: 검출 성공

`data/tag_setup/white_margin.npz`를 새로 저장하고 원본 640×480에서 ID 42를 검출했다.
사전은 DICT_APRILTAG_36h11, 중심 픽셀은 (332,225)이다.
네 꼭짓점의 canonical 순서는 (354,198), (358,248), (310,252), (306,202)이다.
검출 이미지는 `white_margin_detection.png`, 사전별 결과는 `white_margin_detection.json`에 있다.
크기는 사용자 제공값 7cm를 사용하며, 인쇄물 실측 오차는 별도로 확인하지 않았다.
현재 config의 tag_id를 1에서 실제 검출값 42로 변경했다. robot_id는 이름 `"1"`을 유지한다.

### 흰 여백 추가 전 촬영: 검출 실패

사용자 요청으로 RGB-D를 다시 저장했다 (`data/tag_setup/scene_retry.npz`).
차량 위 마커는 보이지만 기본 검출기로 AprilTag 36h11/36h10/25h9/16h5 및 여러 ArUco
사전을 검사했을 때 ID가 검출되지 않았다. 현재 부착물의 실제 ID가 1이라는 뜻이 아니다.
검은 바깥 영역이 어두운 차체와 이어져 외곽 사각형 후보가 잡히지 않는 문제가 의심된다.
이것만으로 단일 원인을 확정하지는 않았다.

## 검증된 출력 파일

아래는 대체용 **ID 1** 출력물이다. 현재 장착한 **ID 42**와 다른 태그이며,
아래 파일로 교체하면 config의 tag_id도 1로 다시 맞춰야 한다.

- [A4 PDF: ID 1, 검은 변 70mm](assets/apriltag_36h11_id1_70mm.pdf)
- [PNG: ID 1, 흰 여백 포함](assets/apriltag_36h11_id1_70mm.png)

OpenCV 사전에서 생성했고 PNG를 0/90/180/270°로 돌린 경우 모두 ID 1로 검출되는 것을 확인했다.
이 검사는 생성 영상의 인식 검사이며 실제 인쇄·촬영 검증은 아니다.
PDF는 A4, 실제 크기 100%로 인쇄한다. 페이지 맞춤을 끄고 검은 변 70mm를 확인한다.
검은 정사각형 주위에 흰 여백 최소 10mm를 남겨 부착한다. 검은 테두리를 자르지 않는다.
현재 마커를 계속 쓰려면 원본의 태그 family/ID와 외곽 흰 여백을 먼저 확인해야 한다.

태그가 검출되기 전에는 Fleet가 유효 Pose를 만들 수 없어 차량은 목표를 실행하지 않는다.
현재 차량은 책상 위에 있어 보이며, 태그 인식 확인과 실제 바닥 주행 지도 생성은 별도 단계다.
이 설정을 반영하려면 실행 중인 Fleet 서버/검사 도구를 다시 시작해야 한다.

## 실시간 Pose null 문제 수정

실제 Fleet에서 Pi는 연결됐으나 pose가 null인 상황을 확인했다.
저장된 RGB fx=385.6702, fy=385.2582에 비해 현재 장치의 값은
fx=386.8249, fy=386.4117로 약 0.3% 달랐다. 기존의 사전 완전 일치 검사가 이 변동을 거부했다.
고정된 카메라 광학 프레임의 extrinsic은 유지하고, 역투영/PnP에는 프레임의 현재 K를 사용한다.
동일 serial·해상도에서 focal 0.5% 이내, principal point 1px 이내 변동을 허용하도록 수정했다.
이는 초기 보수적 검사 경계이며 실측 위치 정확도를 보장하는 수치가 아니다.
큰 변동·다른 장치는 계속 거부하며, 카메라를 물리적으로 옮겨도 된다는 의미는 아니다.

관측 오류는 STATUS의 pose_error에 나타나도록 추가했다. 수정 서버에서 태그 42의
유효 Pose가 다시 나오는 것을 확인했다. 이 보정 검사 단계에서는 모터/목표 명령을 발급하지 않았다.
이후 실제 주행 결과와 최신 테스트 결과는 위 실차 시험 문서에 기록했다.

현재 지도는 약 0.99×0.96m이며 20cm clearance 확장 후 사용 가능한 셀이 25개뿐이다.
카메라를 높여 더 넓은 주행장을 관측하거나, 실제 차체 반경·여유를 측정해 설정해야 한다.
단지 움직이게 하려고 clearance나 Unknown 차단을 제거하면 안 된다.

## 실측 기준점 없이 지도 생성 → 수동 목표 주행

PC의 `pi_car/`에서 가상환경을 활성화한다. 카메라를 고정하고 차량·사람을 지도 촬영 영역에서 뺀다.
기존 Mock/Fleet/카메라 프로세스와 Pi 주행 실행기를 종료한 뒤 새 지도를 만든다.

```bash
pi-car capture --out data/drive/scene.npz
MPLBACKEND=TkAgg pi-car experiment --capture data/drive/scene.npz \
  --pick-floor --out data/drive --clearance 0.20
```

열린 사진에서 물체 없는 **평평한 주행면의 한 부분**을 사각형으로 잡고 대각선 두 모서리를 클릭한다.
전체 영역 경계를 찍는 것이 아니라 바닥 평면 fitting에 쓸 패치를 고르는 것이다.
Depth로 좌표계·관측 범위·지도 크기를 자동 계산한다. 이후 `data/drive/map/`과
`data/drive/calibration.json`이 생성된다. 이 도구의 시험 경로는 검사 파일일 뿐 로봇으로 전송되지 않는다.
`inspect --map data/drive/map --clearance 0.20`으로 원본/확장 지도를 확인할 수 있다.

Pi의 `data/map/` 폴더에 새 `map.npy`, `map_metadata.json`을 복사한다.
PC에서는 아래 두 명령을 별도 터미널에서 실행한다.

```bash
pi-car fleet-server --map data/drive/map --calibration data/drive/calibration.json \
  --robots config/robots.json --host 0.0.0.0 --clearance 0.20 --journal data/fleet.sqlite3
```

```bash
MPLBACKEND=TkAgg pi-car fleet-panel --map data/drive/map --clearance 0.20
```

Pi에서는 현재 Tailscale PC 주소와 실제 설치된 vendor 경로로 다음 명령을 실행한다.

```bash
pi-car robot-client --robot-id 1 --host 100.124.4.8 --map data/map \
  --clearance 0.20 --journal data/robot_1.sqlite3 --enable-motors \
  --vendor-dir "$HOME/ADR036-Adeept_4WD_Smart_Car_Kit_for_RPi-20260317/Code/Adeept_4WD_Smart_Car_for_RPi/Server/Server_OrdinaryWheels" \
  --max-output 0.20 --servo-angle 90
```

사진의 태그를 차량에 그대로 붙여 주행장에 놓고, PC 화면에서 차량 `1` 위치/방향이 맞는지 확인한다.
그다음 흰 영역의 가까운 목표를 클릭한다. STOP selected가 정지 명령이다.
카메라를 움직이면 이 지도·좌표 보정을 다시 사용할 수 없다.
짧은 직선 주행은 확인했으며 큰 회전·정지거리·반복 주행 검증은 남아 있다.
