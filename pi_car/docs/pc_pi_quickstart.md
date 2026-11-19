# PC 지도 클릭 → Raspberry Pi 주행 실행 안내

현재 차량 1대의 실제 경로·IP·명령과 시험 결과는
[실차 시험 및 재실행 안내](real_drive_trial.md)를 먼저 참고한다.

## 1. 지금 실제 주행이 가능한가

현재 코드는 PC 목표 클릭 → 해당 Pi의 A*·경로 추종 → 모터 출력까지 연결할 실행 옵션을 제공한다.
기본 `robot-client`는 여전히 모터 비활성이고, **`--enable-motors`를 명시해야 하드웨어를 연다.**
WES는 필요 없으며 기존 WES용 외부 명령 API는 유지한다.

실제 Pi·차량의 짧은 직선 목표 도착을 1회 확인했다. 큰 회전, 모터 출력/속도,
정지거리와 여러 목표의 반복 주행 검증은 남아 있다.
현재는 장애물 감지 시 정지하는 단계다. 장애물 우회·경로 복귀·로봇 간 충돌 회피는 없다.

이 문서의 변경은 작업 폴더에 있으며 다른 장비에서 `git pull`로 받으려면 먼저 commit/push해야 한다.
실험 영상·지도는 Git 제외이므로 코드 clone과 별도로 지도 파일을 복사한다.

## 2. 어느 저장 공간에 둘 것인가

PC는 사용자 홈의 SSD/HDD, Pi는 사용자 홈의 SD카드/SSD에 놓으면 된다.
예시는 양쪽 모두 `~/robotics/capstone`에 clone하는 방식이다.
현재 작업 중인 PC는 `/home/misys/capstone/capstone/pi_car`를 그대로 써도 된다.
그 경우 아래 `cd ~/robotics/capstone/pi_car`만 현재 경로로 바꾼다.

```text
PC: ~/robotics/capstone/pi_car/
    .venv/                    PC Python 환경
    config/robots.json        태그별 ID·크기·차량 중심·방향
    data/map/map.npy
    data/map/map_metadata.json
    data/calibration.json     고정 카메라 보정
    data/fleet.sqlite3        PC 명령 기록

Pi A/B 각각: ~/robotics/capstone/pi_car/
    .venv/                    Pi Python 환경
    data/map/map.npy          PC와 같은 원본 지도
    data/map/map_metadata.json
    data/robot_A.sqlite3      A는 이 파일, B는 robot_B.sqlite3
```

Pi에는 RealSense·보정 파일·OpenCV가 필요하지 않다. PC가 카메라를 열어 Pose를 보낸다.
카메라를 지도 생성 후 옮겼다면 기존 지도 좌표에 맞춰 다시 보정해야 한다.
예전에 손으로 들고 촬영한 실험 지도를 새로운 카메라 위치에 그대로 적용하면 안 된다.

## 3. PC 최초 설치와 지도 준비

Ubuntu 데스크톱, Python 3.10 이상 기준이다.

```bash
mkdir -p ~/robotics
git clone --branch feature/pi_car1 https://github.com/graviny/capstone.git ~/robotics/capstone
cd ~/robotics/capstone/pi_car
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[pc]'
```

이미 clone했다면 다시 clone하지 말고 기존 저장소에서 branch/pull과 설치를 진행한다.
화면 창에 필요한 Tk가 없으면 `sudo apt install python3-tk`를 설치한다.
panel은 PC의 데스크톱 터미널에서 실행한다.

1. D455를 고정한다. 실제 공간 치수를 `mapping.json`, 바닥 실측 기준점을 `floor_references.json`에 입력한다.
2. 실측 기준점으로 보정하고 차량·사람을 뺀 캡처로 지도를 만든다.
3. `config/robots.json`의 태그 ID·실측 크기·차량 중심·방향 오프셋을 실제 값으로 수정한다.
   예시 A=ID 1, B=ID 2이며 8cm·중심 아래 10cm는 실측값이 아니다.
4. 차체 반경+안전 여유를 측정하고 PC/Pi/panel의 `--clearance`를 같은 값으로 설정한다.

```bash
pi-car capture --out data/floor.npz
pi-car calibrate --capture data/floor.npz --references config/floor_references.json --out data/calibration.json
pi-car capture --out data/scene.npz
pi-car map --capture data/scene.npz --calibration data/calibration.json --config config/mapping.json --out data/map
```

실시간 Fleet는 실측 태그 크기 기반 PnP를 사용한다. 크기 미상 휴대폰 태그의 이전 Depth 진단과 다르다.
자세한 보정 절차는 [usage.md](usage.md)를 참고한다.

## 4. 각 Pi 설치와 지도 복사

Pi 터미널:

```bash
mkdir -p ~/robotics
git clone --branch feature/pi_car1 https://github.com/graviny/capstone.git ~/robotics/capstone
cd ~/robotics/capstone/pi_car
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -e '.[pi]'
mkdir -p data/map
```

Raspberry Pi OS에서 I2C를 활성화하고 기존 Adeept 구동 환경을 준비한다.
GPIO Zero의 Pi 핀 backend가 필요하다. OS의 `python3-lgpio` 등 시스템 GPIO 패키지를
재사용하도록 위 환경은 `--system-site-packages`로 생성한다.
기존 Adeept 서버/자동주행 프로그램이 GPIO·PCA9685를 함께 쓰지 않도록 종료한다.

Pi에 원본 폴더가 없다면, PC에서 예를 들어 다음처럼 복사할 수 있다.
PC의 실제 원본 경로와 Pi 사용자/IP를 맞춘다.

```bash
ssh pi@192.168.0.21 'mkdir -p ~/adeept'
scp -r ~/다운로드/ADR036-Adeept_4WD_Smart_Car_Kit_for_RPi-20260317/Code/Adeept_4WD_Smart_Car_for_RPi/Server/Server_OrdinaryWheels \
  pi@192.168.0.21:adeept/
```

PC에서 지도 파일을 복사한다. 아래 사용자 `pi`, IP `.21/.22`는 실제 값으로 바꾼다.

```bash
cd ~/robotics/capstone/pi_car
scp data/map/map.npy data/map/map_metadata.json pi@192.168.0.21:robotics/capstone/pi_car/data/map/
scp data/map/map.npy data/map/map_metadata.json pi@192.168.0.22:robotics/capstone/pi_car/data/map/
```

PC/Pi는 서로 통신 가능한 Wi-Fi와 동기화된 시계를 사용한다.
PC의 `hostname -I`로 주소를 확인한다. 아래 PC IP `192.168.0.10`은 예시이며 실제 주소로 바꾼다.
Pi가 PC TCP 8765에 접근할 수 있어야 한다.

## 5. PC에서 켜둘 명령: 터미널 두 개

### 터미널 1 — D455·Fleet 서버

```bash
cd ~/robotics/capstone/pi_car
source .venv/bin/activate
pi-car fleet-server --map data/map --calibration data/calibration.json \
  --robots config/robots.json --host 0.0.0.0 --port 8765 \
  --command-port 8766 --clearance 0.20 --journal data/fleet.sqlite3
```

이 프로세스가 D455를 계속 읽어 차량별 Pose를 보낸다. `preview --live`나 RealSense Viewer로
같은 카메라를 동시에 열지 않는다.

### 터미널 2 — 목표 클릭 화면

```bash
cd ~/robotics/capstone/pi_car
source .venv/bin/activate
MPLBACKEND=TkAgg pi-car fleet-panel --map data/map --host 127.0.0.1 --port 8766 --clearance 0.20
```

- 오른쪽 A/B 선택 → 지도 흰색의 유효 영역 **좌클릭 시 즉시 MOVE 전송**.
- 차량 위치·목표 별표·상태·마지막 결과를 표시한다.
- 회색 Unknown, 검정 장애물, 주황 여유 영역은 목표로 선택할 수 없다.
- 연결/위치가 없거나 오래된 차량에는 새 목표를 보내지 않는다.
- `STOP selected` 또는 Escape는 선택 차량에 STOP을 보낸다.
- 이동 중 다른 목표를 클릭하면 이전 명령을 취소하고 Pi가 새 목표를 계산한다.
- 도착 후에는 다음 클릭/외부 명령을 기다린다.
- panel은 별도 프로세스의 명령 클라이언트다. 카메라를 열거나 A*를 실행하지 않는다.
  Pi가 계산한 경로를 panel에 그리는 기능은 아직 없으며 위치·목표를 표시한다.
- panel을 닫는 것만으로 진행 명령을 취소하지 않는다. 먼저 STOP으로 정지한다.
  명령 전송 중에는 안내에 따라 완료 후 STOP을 다시 누른다.

카메라·Pi 없이 화면부터 보려면:

```bash
# PC 터미널 1
pi-car mock-fleet
# PC 터미널 2
MPLBACKEND=TkAgg pi-car fleet-panel --mock-map
```

클릭 후 `Expected a finite number (not boolean)`가 표시됐던 초기 버그는
Matplotlib이 전달한 NumPy float 좌표를 기본 숫자 검사에서 거부한 문제였다.
GUI 경계에서 float로 변환하도록 수정했고 실제 `MouseEvent`부터 TCP·도착까지 검사했다.
이미 실행 중인 panel은 이전 코드를 메모리에 갖고 있으므로 창을 닫고 다시 실행한다.
패키지가 editable 설치 상태이면 재설치는 필요 없으며 Mock 서버는 계속 켜둬도 된다.

## 6. Pi에서 켜둘 명령

### 먼저 모터 비활성으로 통신 확인

Pi A:

```bash
cd ~/robotics/capstone/pi_car
source .venv/bin/activate
pi-car robot-client --robot-id A --host 192.168.0.10 --map data/map \
  --clearance 0.20 --journal data/robot_A.sqlite3 --mock-range 1.0
```

Pi B는 `--robot-id B --journal data/robot_B.sqlite3`로 바꾼다.
이 명령은 실제 바퀴를 움직이지 않는다. 물리적 차량이 안 움직여 Pose도 변하지 않으면
도착하지 않는 것이 정상이다. 가상 이동·도착은 Mock 환경에서 확인한다.

### 실제 하드웨어 연결·구동

Pi 자체에 기존 Adeept의 `Server_OrdinaryWheels/Move.py`가 있어야 한다.
아래 `~/adeept/Server_OrdinaryWheels`는 예시다. PC 다운로드 폴더가 아닌
**Pi의 실제 디렉터리**로 바꾼다. 없다면 원본 OrdinaryWheels 폴더를 먼저 복사한다.

기존 하드웨어 인터페이스는 trigger BCM23 / echo BCM24, PCA9685 servo channel 0이다.
서보 정면을 기본 90°로 맞추며 다른 설치 각도는 `--servo-angle`로 지정한다.
모터 M1/M2 왼쪽, M3/M4 오른쪽과 원본 polarity를 사용한다.

Pi A:

```bash
cd ~/robotics/capstone/pi_car
source .venv/bin/activate
pi-car robot-client --robot-id A --host 192.168.0.10 --map data/map \
  --clearance 0.20 --journal data/robot_A.sqlite3 \
  --enable-motors --vendor-dir ~/adeept/Server_OrdinaryWheels \
  --max-output 0.15 --servo-angle 90
```

Pi B:

```bash
pi-car robot-client --robot-id B --host 192.168.0.10 --map data/map \
  --clearance 0.20 --journal data/robot_B.sqlite3 \
  --enable-motors --vendor-dir ~/adeept/Server_OrdinaryWheels \
  --max-output 0.15 --servo-angle 90
```

켜기만 하면 전진하지 않는다. 유효한 PC Pose·외부 목표·새 초음파 관측이 있어야 한다.
출력 상한 0.15는 15% 모터 출력이며 m/s가 아니다. 실제 최소 구동 출력은 실측한다.
초기 하드웨어 옵션은 최대 30%까지만 허용하며 `--mock-range`와의 혼용을 거부한다.
초음파는 실제 측정 시각을 보존하고 timeout·0·2m 최대 포화는 무효 처리한다.
넓은 공간에서 echo가 없으면 정지할 수 있다. 측정 스레드가 멈춰도 캐시값에 새 시각을 붙이지 않는다.
종료 시 모터 정지·센서 해제·서보 PWM 해제·PCA9685 종료를 수행한다.

이 명령은 실제 출력이 가능한 연결 코드지만 **실제 차량으로 검증한 결과는 아니다.**
첫 실험은 바퀴를 지면에서 띄워 방향·정지·서보 정면부터 확인하고, 이후 낮은 출력으로 진행한다.
소프트웨어 watchdog 외 하드웨어 긴급 정지를 사용할 수 있어야 한다.

## 7. WES 연결 유지

수동 panel과 WES는 같은 외부 명령 인터페이스를 사용한다.

```python
receipt = await fleet.dispatch_goal(robot_id="A", x=1.5, y=1.0, command_id="WES001")
result = await fleet.wait_result("WES001")
```

수동 명령은 `manual.<uuid>` ID를 사용한다. WES와 panel에서 같은 차량에 새 명령을 보내면
기존 정책대로 마지막 명령이 이전 주행을 취소한다. 작업 소유권·우선순위 배정은 WES 측 후속 범위다.

## 8. 코드와 검증

추가 파일은 `pc/panel.py`, `vehicle/hardware/live.py`, `tests/test_manual_panel.py`, 이 문서다.
CLI·INFO/STATUS 응답·Pi 의존성도 확장했고 루트 README는 유지했다.

- GUI 이벤트 → 실제 localhost TCP → Mock B만 이동·도착 → STOP 확인.
- 지도/버전, 장애물 목표, 오래된 Pose, 실제 모터와 가짜 센서 혼용 거부 검사.
- 실제 GPIO 대신 GPIO Zero Mock 핀으로 echo·포화·새 측정 시각 검사.
- 가짜 Adeept 모듈로 초기 정지·서보 중앙·종료 PWM 정리 검사.
- 실제 바퀴·서보·초음파 및 PC/Pi 간 Wi-Fi 주행 시험은 수행하지 않았다.

현재 검증 환경에서 기존 테스트를 포함해 **46개가 통과**했다.

```bash
MPLBACKEND=Agg python -m unittest discover -s tests -v
```

초음파 freshness hook은 GPIO Zero 2.x의 `_read()`를 사용한다.
[공식 소스](https://gpiozero.readthedocs.io/en/stable/_modules/gpiozero/input_devices.html)의
실제 echo 및 무효 반환 동작을 확인했다. protected hook이므로 버전 변경 시 재검증한다.
