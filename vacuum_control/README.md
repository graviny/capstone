# 샤오미 로봇청소기 제어

SDJQR01RR (`rockrobo.vacuum.v1`)의 로컬 네트워크 제어와 RealSense D455 기반 목표점 주행을 제공합니다. D455 주행은 ArUco로 청소기 위치를 추적하고 깊이 영상의 장애물 정보를 이용해 A* 경로를 만들거나 다시 계산합니다.

## Windows에서 설치

저장소의 `vacuum_control` 폴더에서 PowerShell을 실행합니다. Windows의 Python 3.9 환경에서 확인한 의존성입니다.

```powershell
python -m venv .venv-camera
.\.venv-camera\Scripts\python.exe -m pip install -r requirements-navigation.txt
```

카메라 없이 기본 제어만 쓸 경우 `requirements.txt`로 설치할 수 있습니다.

## 상태 조회와 직진

청소기와 노트북을 서로 통신할 수 있는 같은 로컬 네트워크에 연결합니다. 토큰은 명령 실행 뒤 입력하며 화면에 표시되지 않습니다.

```powershell
$vacuumIp = Read-Host "청소기 IPv4 주소"
.\.venv-camera\Scripts\python.exe demo.py status --ip $vacuumIp
```

앞뒤가 빈 평평한 구간에서 한 번에 한 명령씩 시험합니다.

```powershell
.\.venv-camera\Scripts\python.exe demo.py forward --ip $vacuumIp --speed 0.1 --seconds 1
.\.venv-camera\Scripts\python.exe demo.py stop --ip $vacuumIp
```

수동 모드 진입 뒤 약 5초 준비 시간이 있습니다. 명령이 끝났는지와 실제 바퀴 정지는 현장에서 확인하세요. 통신 오류가 나면 본체 시작/일시정지 버튼을 사용하세요.

## D455로 목표점 주행

1. [D455 연결](D455.md)과 [바닥·로봇 마커 배치](FLOOR_MARKER_SETUP.md)를 완료합니다.
2. 미리보기에서 바닥을 보정하고 로봇 위치·방향과 목표 경로를 확인합니다.
3. 미리보기를 닫고 실물 모드를 실행합니다. 보정을 불러온 뒤 목표를 클릭하고 **실물 주행 시작**을 누릅니다. `Esc`는 정지 요청입니다.

```powershell
.\.venv-camera\Scripts\python.exe navigation_app.py --resolution 1280x720
.\.venv-camera\Scripts\python.exe navigation_app.py --resolution 1280x720 --drive --ip $vacuumIp
```

정지 확인 뒤 새 경로가 확보되면 우회 주행을 시도합니다. 깊이·마커가 불안정하거나 정지 통신이 실패하면 창의 상태 메시지를 확인하세요. [보정·장애물 시험 안내](NAVIGATION.md)

## 장비 없이 확인

```powershell
.\.venv-camera\Scripts\python.exe demo.py forward --dry-run
.\.venv-camera\Scripts\python.exe navigation_app.py --simulate
.\.venv-camera\Scripts\python.exe -m unittest discover -s tests -q
```
