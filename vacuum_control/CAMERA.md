# 아이폰으로 먼저 마커 인식하기

개발용 카메라는 아이폰, 최종 시연용은 천장에 고정할 RealSense D455를 기준으로
합니다. 이 절차는 픽셀 단위 인식을 확인하는 데 사용합니다.

## 1. 아이폰 영상을 Windows로 연결

아이폰에 **Camo Camera**, Windows에 **Camo Studio**를 설치하고 USB로 연결합니다.
처음에는 Camo Studio에서 후면 카메라 영상이 정상적으로 나오는지 확인합니다.
다른 앱이 카메라를 쓸 수 있게 하는 호환 구성 요소 설치 안내가 나오면 안내에
따릅니다. Windows에서 사용하는 카메라 번호는 환경에 따라 달라집니다.

공식 설치 안내: https://camo.com/support/camo/camo-getting-started

자동 프레이밍, 인물 추적, 배경 효과, 영상 뒤집기/회전, 디지털 줌은 꺼두고
영상이 고정되게 합니다. 보정 후에는 렌즈, 해상도, 줌과 카메라 위치를 유지해야
합니다. 이번에는 사람 손에 들고 종이 마커를 비춰 인식만 확인해도 됩니다.

## 2. 인쇄용 마커

이미 생성된 `markers/robot_0.svg`를 브라우저로 열고 100% 배율로 인쇄합니다.
검은 사각형 전체 한 변이 12cm인지 자로 확인합니다. 흰 여백과 위쪽 화살표도
남겨두세요. 검은 부분 위에 반사되는 테이프를 붙이지 마세요.

화살표 `ROBOT FRONT`가 실제 청소기의 앞쪽을 가리키도록 고정합니다. 라이다,
버튼이나 센서를 가리지 않는 평평한 판을 사용합니다. 마커 중심과 로봇 중심의
위치가 다르면 나중에 그 오프셋도 보정해야 합니다.

마커를 다시 만들려면 PowerShell에서:

```powershell
.\.venv-camera\Scripts\python.exe camera_demo.py marker
```

## 3. 마커 인식 실행

Camo Studio가 아이폰 영상을 받고 있는 상태에서:

```powershell
.\.venv-camera\Scripts\python.exe camera_demo.py track --source 0
```

노트북 내장 카메라가 뜨면 Q로 닫고 번호를 1, 2 등으로 바꿔 확인합니다.

```powershell
.\.venv-camera\Scripts\python.exe camera_demo.py track --source 1
```

Camo는 보이는데 영상을 열지 못하면 캡처 방식을 바꿔 봅니다.

```powershell
.\.venv-camera\Scripts\python.exe camera_demo.py track --source 1 --backend msmf
```

마커를 비추면 ID 0의 테두리, 앞쪽 화살표, u/v 픽셀 좌표가 표시됩니다.
마커를 움직이거나 돌렸을 때 좌표와 화살표도 바뀌는지 확인하세요. 창에서 Q,
Esc 또는 닫기 버튼으로 종료합니다. 이 프로그램은 로봇 명령을 보내지 않습니다.

OpenCV가 없는 다른 Python 환경에서 사용할 때는 아래 패키지가 필요합니다.
현재 이 작업 환경에는 OpenCV 4.13과 ArUco가 설치되어 있어 추가 설치하지 않았습니다.

```powershell
python -m pip install "opencv-python>=4.7,<5"
```

## 출력과 다음 단계

`runtime/latest_detection.json`에는 가장 최근 검출 결과를 씁니다. 마커가
안 보이거나 중복 ID가 있거나 프로그램이 종료되면 `valid: false`로 바뀌며
이전 좌표를 남기지 않습니다. `received_at_unix_s`는 PC 수신 시각입니다.
스마트폰/가상 카메라 내부 지연은 측정하지 않으므로 실제 촬영 시각과 다릅니다.
소비 프로그램은 `valid`뿐 아니라 파일 갱신 시각도 검사해야 합니다.

u/v는 이미지 픽셀이고 `heading_image_deg`는 화면 오른쪽 0도, 위쪽 +90도입니다.
창고 좌표 x/y나 세계 좌표계 yaw가 아닙니다. 이 출력으로 로봇을 자동 운전하지 않습니다.

최종 공간과 D455 설치가 정해지면 실제 치수, 카메라 내부 파라미터, 바닥과
마커의 높이 차이, 마커와 로봇 중심의 오프셋을 반영해 미터 좌표로 보정합니다.
아이폰으로 한 위치 보정을 D455에 그대로 쓰면 안 됩니다. 목표점 이동은 그 뒤에
측정 오차와 영상 지연을 확인한 다음 연결합니다.

검출 API: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
