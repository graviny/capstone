"""d455_drive 설정값. 실차 테스트하면서 여기 값만 조정하면 된다."""

# ---- 통신 ----
HOST = "0.0.0.0"
PORT = 8765
CONTROL_HZ = 20          # 제어 루프 주기
STATUS_HZ = 10           # 노트북으로 상태 보내는 주기
WATCHDOG_TIMEOUT = 0.5   # 이 시간(초) 동안 명령이 없으면 정지

# ---- 하드웨어 (web/move.py, web/RPIservo.py, web/ultra.py 와 동일한 배선) ----
PCA9685_ADDR = 0x5F
MOTOR_IN1 = 15           # M1 (뒷바퀴 구동 모터)
MOTOR_IN2 = 14
MOTOR_DIRECTION = 1      # 전진이 반대로 돌면 -1

SERVO_CENTER = 90
STEER_CHANNEL = 0        # 조향 서보. WebServer.py 에서 +30 = 왼쪽
STEER_TRIM_DEG = 0       # 직진인데 한쪽으로 휘면 보정
STEER_MAX_DEG = 30

SCAN_CHANNEL = 1         # 초음파 센서 회전 서보
SCAN_LEFT_DEG = 60       # 실차에서 실제로 왼쪽을 보는지 확인할 것 (반대면 부호 바꾸기)
SCAN_RIGHT_DEG = -60

ULTRA_TRIG = 23
ULTRA_ECHO = 24
ULTRA_MAX_M = 2.0

ADC_ADDR = 0x48          # 배터리 전압 (web/Voltage.py 와 동일 계산)

# ---- 속도 ----
MAX_SPEED = 60           # 노트북이 100을 보내도 이 값으로 제한 (%)

# ---- 안전 계층 (전진 중에만 동작, 후진은 센서가 없어서 노트북 책임) ----
SLOW_DIST_CM = 35        # 이 거리 안이면 감속
STOP_DIST_CM = 18        # 이 거리 안이면 회피 시작
SLOW_MIN_FACTOR = 0.35   # STOP_DIST 직전 속도 배율
CLEAR_DIST_CM = 30       # 스캔 시 이 거리보다 멀면 "비어 있음"

# ---- 회피 동작 ----
AVOID_STOP_TIME = 0.3
SCAN_SETTLE_TIME = 0.35  # 스캔 서보가 도착할 때까지 대기
AVOID_SPEED = 35
AVOID_BACKUP_TIME = 0.7
AVOID_TURN_TIME = 0.9
AVOID_ABORT_CM = 10      # 선회 중 이 거리보다 가까우면 중단 (result=blocked)
