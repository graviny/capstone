# d455_drive (PiCar 쪽)

천장 D455 + AprilTag로 노트북이 A* 경로를 계산하고, PiCar는 **속도/조향 명령을 받아 실행 + 초음파로 돌발 장애물 회피**만 담당한다.

```
노트북 ──{"type":"cmd","speed":..,"steer":..}──▶ PiCar (ws://<pi-ip>:8765)
노트북 ◀──── status(10Hz) / avoid_start / avoid_done ─── PiCar
```

## 실행

```bash
cd ~/adeept_picar-b2/d455_drive
sudo python3 server.py                 # 실차
python3 server.py --dry-run -v         # 하드웨어 없이 시험
python3 test_client.py --speed 30 --steer 0 --time 2    # 다른 터미널에서 시험
```

- `web/WebServer.py`, `web/APPServer.py`와 **동시에 실행하면 안 된다** (같은 모터/서보/초음파 사용).
- 종료는 Ctrl+C (모터 정지, 서보 중앙 복귀).

## 파일

| 파일 | 역할 |
|---|---|
| `config.py` | 포트, 속도 제한, 감속/회피 거리, 서보 채널·트림 등 모든 조정값 |
| `hardware.py` | 모터(M1), 조향 서보(ch0), 초음파 스캔 서보(ch1), 초음파, 배터리 전압 |
| `safety.py` | 안전 계층: normal → slow → avoiding 상태 머신 |
| `server.py` | WebSocket 서버, 20Hz 제어 루프, 워치독 |
| `test_client.py` | 고정 명령을 보내고 상태를 출력하는 시험용 클라이언트 |

## 프로토콜 (JSON, WebSocket 텍스트 메시지)

### 노트북 → PiCar

| 메시지 | 설명 |
|---|---|
| `{"type":"cmd","speed":40,"steer":-12,"seq":123}` | speed: -100~100(%, 음수=후진, `MAX_SPEED`로 제한), steer: -30~30(도, **+ = 왼쪽**), seq: 선택 |
| `{"type":"stop"}` | 즉시 정지 명령 |
| `{"type":"ping","t":1712.3}` | `{"type":"pong","t":1712.3}` 응답 (지연 측정용) |

- **10~20Hz로 계속 보내야 한다.** 0.5초 동안 명령이 없으면 워치독으로 정지(`state:"idle"`).
- 연결이 끊겨도 정지한다. 새 클라이언트가 접속하면 기존 연결은 끊긴다(1대만 제어).

### PiCar → 노트북

상태 (10Hz):
```json
{"type":"status","state":"normal","avoid_phase":null,"dist_cm":87.4,
 "speed":40.0,"steer":-12.0,"last_seq":123,"battery_v":7.9}
```
- `state`: `idle`(명령 없음) / `normal` / `slow`(앞이 가까워 감속 중) / `avoiding`(회피 중, 노트북 명령 무시)
- `speed`, `steer`: 안전 계층을 거쳐 **실제로 적용된 값**

이벤트:
```json
{"type":"event","event":"avoid_start","dist_cm":16.2}
{"type":"event","event":"avoid_done","result":"ok","avoid_dir":"left","scan_left_cm":80.1,"scan_right_cm":21.0}
```
- `result:"blocked"`: 양쪽 다 막혔거나 선회 중 막힘 → 후진만 하고 멈춘 상태. 노트북이 재계획해야 함.

## 안전 계층 동작 (전진 중에만)

| 정면 거리 | 동작 |
|---|---|
| > 35cm | 명령 그대로 |
| 18~35cm | 거리에 비례해 감속 (최소 35%) |
| < 18cm | 회피: 정지 → 초음파 좌/우 스캔 → 빈 쪽 반대로 꺾어 후진 0.7s → 빈 쪽으로 선회 전진 0.9s → `avoid_done` |

- 회피는 약 2.5~3초 걸리고 그동안 노트북 명령은 무시된다.
- **후진은 센서가 없어 보호되지 않는다.** 노트북이 지도로 판단해야 한다.

## 실차에서 먼저 확인/조정할 것

1. `steer=+20` 보냈을 때 실제로 **왼쪽**으로 꺾이는지 → 반대면 hardware에서 부호 조정
2. `speed=+30`이 **전진**인지 → 반대면 `MOTOR_DIRECTION = -1`
3. steer=0에서 직진하는지 → `STEER_TRIM_DEG`
4. `SCAN_LEFT_DEG`가 실제로 왼쪽을 보는지 (`-v` 로그 + 눈으로 확인)
5. 차가 움직이기 시작하는 최소 speed 값 (노트북 제어기에 알려줄 값)
6. 최대 조향(±30°)에서 회전 반경, speed별 실제 속도(m/s) → 노트북 쪽 A*/Pure Pursuit 파라미터
