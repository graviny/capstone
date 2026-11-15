# 알고리즘 근거와 현재 구현 범위

현재 스택은 기존 기하·경로 계획 기법을 조합한 초기 구현이다.
RealSense SDK/OpenCV 문서는 구현 중 직접 확인했고, 아래 원논문과 활용 사례는
사용자의 근거 요청에 따라 현재 코드와 대조하여 정리했다.
모든 원논문을 처음부터 읽고 그대로 재현한 구현이라는 뜻은 아니다.

## 1. RGB-D 정렬과 픽셀 역투영

코드: `src/pi_car/pc/camera.py`, `src/pi_car/common/geometry.py`.

근거: [RealSense 공식 Projection 문서](https://dev.realsenseai.com/docs/projection-in-realsense-sdk-2-0/),
[공식 RGB/Depth 정렬 예제](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/align-depth2color.py).

Depth를 RGB로 정렬하고 SDK 렌즈 모델로 두 영상을 보정한 다음,
보정된 pinhole 영상에서 `X=(u-cx)Z/fx`, `Y=(v-cy)Z/fy`로 카메라 XYZ를 구한다.
이후 강체 변환으로 바닥 좌표에 옮긴다. SDK를 실제로 호출하는 부분과 직접 작성한
벡터화 역투영 계산이 함께 있다. optical Z와 카메라에서 점까지의 직선거리는 다르다.

발표 문장: “RealSense SDK의 영상 정렬 및 카메라 투영 모델에 따라 RGB-D 관측을
미터 단위 3차원 좌표로 변환하였다.”

## 2. RANSAC 바닥 평면 추정 + SVD 보정

코드: `src/pi_car/pc/floor.py`의 `estimate_floor()`.

원논문: M. A. Fischler and R. C. Bolles, “Random Sample Consensus: A Paradigm for
Model Fitting with Applications to Image Analysis and Automated Cartography,” CACM, 1981.
[원문](https://graphics.stanford.edu/courses/cs233-20-spring/ReferencedPapers/ransac.pdf).

실제 소프트웨어 사례: [PCL Plane model segmentation](https://pointclouds.org/documentation/tutorials/planar_segmentation.html)은
점군 평면 모델과 RANSAC을 사용한다.
[Open3D point-cloud 문서](https://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html)에도
RANSAC 기반 평면 분리가 제공된다.

우리 코드는 사용자가 바닥으로 확인한 ROI에서 무작위 3점으로 평면 후보를 만들고,
거리 기준으로 지지점을 세어 최다 지지 평면을 선택한 뒤 SVD로 다시 맞춘다.
PCL/Open3D를 실행하거나 코드를 복사한 것이 아니라 같은 기본 원리를 NumPy로 구현했다.
바닥 ROI 자체는 수동 선택이며, 장면 전체에서 바닥을 자동 판별하는 기능은 아니다.
거리 1.5cm, 반복 300회 등은 프로젝트의 초기 설정값으로 실험 조정 대상이다.

발표 문장: “RANSAC 기반 평면 추정을 바닥 ROI에 적용하고, 지지점의 최소제곱 평면으로
정제하여 바닥 좌표계를 구성하였다.”

## 3. 측정 기준점의 강체 정합

코드: `src/pi_car/common/geometry.py`의 `fit_rigid()`, `pc/calibration.py`.

원논문: W. Kabsch, “A solution for the best rotation to relate two sets of vectors,”
Acta Crystallographica A, 1976, 32:922–923.
[DOI](https://doi.org/10.1107/S0567739476001873).

코드의 중심화·SVD·반사 제거·이동량 계산은 Kabsch 계열의 강체 최소제곱 정합에 해당한다.
카메라에서 얻은 기준점과 실측 바닥 기준점의 대응으로 회전·이동을 계산한다.
이 경로는 구현돼 있지만 이번 실장 실험에는 실측 기준점이 없어 사용하지 않았다.
실험의 로컬 평면 좌표계와 실측 월드 좌표 보정을 구분해야 한다.

## 4. 높이 기반 2D 격자 지도

코드: `src/pi_car/pc/mapping.py`, `src/pi_car/common/grid.py`.

표현의 근거: A. Elfes, “Using Occupancy Grids for Mobile Robot Perception and
Navigation,” Computer, 1989.
[원문](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated4/elfes_occup_grids.pdf).

이 논문은 로봇의 공간 표현과 항법에 확률적 점유 격자를 사용한다.
현재 코드는 격자 표현을 채택하지만 논문의 Bayesian occupancy 갱신을 구현하지 않았다.
바닥 높이에 가까운 관측은 Free, 지정 높이 이상은 Occupied, 관측 부족은 Unknown으로
분류하는 단일 캡처 기반 지도다. 높이 임계값과 최소 관측 수는 직접 설계한 규칙이다.
SLAM, 다중 시점 확률 융합, 동적 지도 갱신을 완료했다고 표현하면 안 된다.

발표 문장: “점유 격자 표현을 채택하고, 바닥 기준 높이 임계값으로 정적 3상태
격자 지도(Free/Occupied/Unknown)를 생성하였다.”

## 5. 차량 크기를 반영하는 장애물 확장

코드: `src/pi_car/common/grid.py`의 `inflated()`.

실제 항법 스택 사례: [Nav2 InflationLayer API](https://api.nav2.org/nav2-jazzy/html/classnav2__costmap__2d_1_1InflationLayer.html),
[Nav2 footprint 설정](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/footprint/setup_footprint/).

Nav2는 차량 크기와 장애물 주변 비용을 고려하는 inflation 계층을 제공한다.
우리 코드는 그 개념에 대응하는 단순한 이진 장애물 확장이다. 차체를 감싸는 원의 반지름과
여유 및 격자 양자화 여유를 사용하며 Unknown과 지도 밖도 막는다.
Nav2를 연결하거나 Nav2의 지수 감쇠 비용장·다각형 footprint 검사를 그대로 구현하지는 않았다.
이번 20cm는 실측 차체 크기가 아닌 실험 가정이다.

## 6. AprilTag 검출과 두 가지 좌표 계산 경로

코드: `src/pi_car/pc/localization.py`, `src/pi_car/pc/tagcheck.py`.

태그 원논문: E. Olson, “AprilTag: A Robust and Flexible Visual Fiducial System,” ICRA, 2011.
[원문](https://april.eecs.umich.edu/pdfs/olson2011a.pdf).

실제 코드 의존성은 OpenCV `ArucoDetector`의 `DICT_APRILTAG_36h11`이다.
[OpenCV 검출 문서](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html).
AprilTag C 라이브러리 원본 검출기를 직접 연결한 것으로 설명하면 안 된다.

### 실측 크기를 아는 경우: PnP/IPPE

코드의 `solvePnPGeneric(..., SOLVEPNP_IPPE_SQUARE)`는 OpenCV 기능을 호출한다.
근거: T. Collins and A. Bartoli, “Infinitesimal Plane-Based Pose Estimation,” IJCV,
2014, 109:252–286, DOI 10.1007/s11263-014-0725-5.
[저자 공식 구현·서지정보](https://github.com/tobycollins/IPPE),
[OpenCV PnP 설명](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html).

태그의 실제 크기와 네 꼭짓점으로 자세를 추정하고 후보의 재투영 오차 등을 검사한다.
IPPE는 평면 마커의 자세 추정에 쓰이며 자세 후보의 모호성을 다룬다.
이번 휴대폰 태그는 실측 크기를 몰라 이 경로의 실제 좌표 결과를 채택하지 않았다.

### 이번 실장 실험: 태그 검출 + Depth 중심 역투영

검출한 태그 내부의 유효 깊이 중앙값을 중심 픽셀에 적용하고, 바닥 좌표계로 변환했다.
크기를 몰라도 RGB-D를 이용해 중심의 미터 좌표를 얻기 위한 진단용 조합이다.
이는 AprilTag 논문 전체나 IPPE를 그대로 재현한 위치 추정 결과가 아니다.
방향은 태그가 바닥과 평행하다는 가정으로 근사했다. 기울어진 태그의 완전한 자세 추정은 아니다.

실제 활용 사례: [FIRST Tech Challenge 공식 AprilTag Localization](https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_localization/apriltag-localization.html)은
태그·카메라·로봇·경기장 좌표의 변환을 통해 로봇 위치를 계산한다.
이 사례는 로봇 탑재 카메라가 고정 태그를 보는 구성이며, 우리의 외부 카메라가 차량 태그를
보는 구성과 배치가 다르다. 태그 관측과 좌표 변환을 이용하는 공통 근거로 제시한다.

발표 문장: “OpenCV의 AprilTag 사전을 이용해 ID와 꼭짓점을 검출하였다. 초기 실험에서는
RGB-D 중심 역투영으로 태그 위치를 확인했고, 실측 태그 크기를 사용하는 IPPE 기반
자세 추정 경로를 별도로 구성하였다.”

## 7. A* 전역 경로 계획

코드: `src/pi_car/vehicle/planning/astar.py`.

원논문: P. E. Hart, N. J. Nilsson, and B. Raphael, “A Formal Basis for the Heuristic
Determination of Minimum Cost Paths,” IEEE Transactions on Systems Science and Cybernetics,
1968, 4(2):100–107.
[저자 공개 원문](https://ai.stanford.edu/~nilsson/OnlinePubs-Nils/PublishedPapers/astar.pdf).

현재는 4방향·이동 비용 1의 격자에서 맨해튼 거리 휴리스틱을 사용하는 직접 구현이다.
Unknown을 막고, 팽창된 지도에서 경로를 만든다. 해당 이산 그래프의 최소 비용 경로와
실제 차량의 최단·최속·주행 가능한 곡선 경로는 다르다. 회전 비용·차량 운동학은 고려하지 않는다.
PC 검사 도구에서 미리 실행했고 Pi 배포·실차 추종은 아직 검증하지 않았다.

## 8. 규칙 기반 제어·초음파 어댑터·정지 정책

코드: `vehicle/control/waypoint.py`, `vehicle/runtime.py`, `vehicle/hardware/adeept.py`.

경유점 방향 오차가 크면 회전하고 작으면 전진하는 초기 제어 규칙은 프로젝트에서 작성했다.
특정 논문 제어기를 재현한 것이 아니다. PID·Pure Pursuit는 아직 구현하지 않았다.
초음파·서보·모터 인터페이스는 사용자가 제공한 Adeept 로컬 원본을 읽어 맞췄다.
원본 위치·단위·import 부작용은 [adeept.md](adeept.md)에 기록돼 있다.
위치/통신/초음파가 무효면 정지하는 것은 프로젝트의 안전 우선 정책이며,
검증된 전체 안전 시스템이나 완성된 장애물 회피 알고리즘으로 표현하면 안 된다.

## 발표에서 구분할 것

- **직접 사용한 도구:** RealSense SDK, OpenCV, NumPy.
- **알고리즘의 근거:** RANSAC, SVD 강체 정합, 격자 표현, obstacle inflation, AprilTag/IPPE, A*.
- **프로젝트에서 정한 규칙:** 바닥 ROI·원점, 높이/거리 임계값, Depth 중앙값 조합,
  태그의 바닥 평행 가정, 시험점 선정, 상태 전환과 초기 경유점 추종.
- **아직 검증하지 않은 것:** 독립 실측 좌표 오차, 실시간 검출 안정성, 실차 경로 추종·회피.

“기존에 사용된 기법”이라는 근거와 “우리 장비에서 정확도·안전성이 검증됨”은 별개다.
캡스톤의 독창성은 기본 A*/AprilTag를 새로 제안했다고 주장하기보다,
향후 이종 로봇 협력·선택적 정보 공유 정책 및 그 실험적 효과에서 설명하는 것이 적절하다.
