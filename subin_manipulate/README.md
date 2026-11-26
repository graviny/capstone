# D455 깊이 기반 PiCar-B2 자율주행

D455를 차체 정면에 고정하고 Raspberry Pi의 USB 3 포트에 연결한다. 스크립트는 깊이 영상의 좌·중·우에서 가까운 거리(유효 픽셀의 하위 5백분위)를 추정한다. 정면이 열려 있으면 직진하고, 정면에 장애물이 다가오면 여유가 더 큰 쪽으로 조향하면서 전진한다. 어느 구역이든 너무 가깝거나 65% 미만의 깊이 픽셀만 유효하면 정지한다. 프레임이 250ms 동안 들어오지 않아도 정지한다.

저장소의 [모터 예제](../examples/04_motor.py)와 [초음파 자동주행](../web/functions.py), [웹 조종의 조향 방식](../web/WebServer.py)을 참고했다. D455 연결은 [RealSense 공식 깊이 예제](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/python-tutorial-1-depth.py)와 같은 Z16 깊이 스트림을 사용한다.

## 환경 설정

현재 장치에서 확인한 환경은 Debian 13, ARM64, Python 3.13이다. NumPy와 Adeept 모터용 Adafruit 라이브러리가 이미 설치되어 있고 `/dev/i2c-1`도 확인했다. 이 장치에는 `pyrealsense2`와 D455 연결이 추가로 필요하다. [PyPI 배포 안내](https://pypi.org/project/pyrealsense2/)에 따르면 ARM64 휠은 Jetson용이므로 아래처럼 Python 3.13에 맞춰 소스에서 빌드한다.

```bash
sudo apt update
sudo apt install git cmake build-essential python3-dev libusb-1.0-0-dev libudev-dev libssl-dev pkg-config v4l-utils

cd ~
git clone --depth 1 https://github.com/realsenseai/librealsense.git
cd librealsense
./scripts/setup_udev_rules.sh
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_TOOLS=OFF \
  -DBUILD_PYTHON_BINDINGS=ON -DPython_EXECUTABLE="$(command -v python3)" \
  -DCMAKE_INSTALL_PREFIX="$HOME/.local/realsense" -DCMAKE_INSTALL_LIBDIR=lib \
  -DPYTHON_INSTALL_DIR="$HOME/.local/realsense/python" \
  -DCMAKE_INSTALL_RPATH="$HOME/.local/realsense/lib"
cmake --build build --parallel 2
cmake --install build
```

Python이 설치 위치를 찾도록 사용자 모듈 경로에 한 번 등록한다. udev 규칙을 처음 설치한 뒤 USB를 다시 연결한다. [공식 Linux 설치 문서](https://github.com/realsenseai/librealsense/blob/master/doc/installation.md), [공식 Python 빌드 문서](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/readme.md)

```bash
rs_user_site="$(python3 -m site --user-site)"
mkdir -p "$rs_user_site"
printf '%s\n' "$HOME/.local/realsense/python" > "$rs_user_site/realsense-local.pth"
python3 -c 'import pyrealsense2 as rs; print([d.get_info(rs.camera_info.name) for d in rs.context().query_devices()])'
```

위 명령으로 D455가 표시되면 저장소 루트에서 실행한다.

## 실행

```bash
python3 subin_manipulate/d455_drive.py
```

기본 모드는 좌·중·우 거리와 판단 결과만 출력한다. 카메라 앞 장애물을 옮겨 `forward`, `left`, `right`, `stop`이 예상대로 나오는지 확인한다.

실제 구동 전 같은 모터 보드를 쓰는 웹 조종 프로그램을 종료한다. 서비스로 실행 중이면 `sudo systemctl stop Adeept_Robot.service`를 사용한다. 처음에는 바퀴를 띄워 M1이 전진 방향으로 돌고 0번 조향 서보가 좌우로 기대한 방향으로 움직이는지 확인한다.

```bash
python3 subin_manipulate/d455_drive.py --drive --speed 20 --stop-m 0.8 --avoid-m 1.3
```

`--speed`는 모터 PWM 비율이며, `--steering`은 조향 서보의 중립 기준 각도(기본 25도)다. `Ctrl+C`로 종료한다. 초기 구동은 넓고 평평한 공간에서 지켜보며 진행한다. 카메라 각도에 따라 바닥이 장애물로 보일 수 있고, 낮거나 가는 물체, 유리, 카메라 바로 아래의 근접 장애물은 감지하지 못할 수 있다. 이 예제의 범위는 전방 장애물 회피다.
