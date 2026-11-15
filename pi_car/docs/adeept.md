# 참고한 Adeept 원본

사용자가 지정한 다음 로컬 소스를 읽어 인터페이스를 확인했다. 원본은 수정하거나 실행하지 않았다.

```text
~/다운로드/ADR036-Adeept_4WD_Smart_Car_Kit_for_RPi-20260317/
  Code/Adeept_4WD_Smart_Car_for_RPi/Server/
    Server_OrdinaryWheels/{Ultra.py,Move.py,Functions.py,RPIservo.py}
    Server_MecanumWheels/Move.py
```

원본을 저장소에 복사하지 않았다. 차량에는 해당 vendor 디렉터리와 의존성이 있어야 한다.
연결 시 bootstrap에서 필요한 모듈만 명시적으로 import하고 어댑터 생성자에 주입한다.
원본 파일명은 대소문자를 구분한다: `Move`, `Ultra`, `RPIservo`.

| 원본 | 확인한 내용 | 이 스택의 연결 |
| --- | --- | --- |
| Ultra.py | gpiozero.DistanceSensor, trigger BCM23 / echo BCM24, 최대 2m | checkdist()의 cm를 m로 변환, 0/비정상/최댓값 포화는 무효 |
| Move.py | setup(), Motor(channel,direction,speed), motorStop(), destroy() | M1·M2 왼쪽, M3·M4 오른쪽, 원본 polarity 보존 |
| RPIservo.py | ServoCtrl.moveAngle(0, 상대각), 초기 90° | 0 정면, +45 왼쪽, -45 오른쪽 (실차 장착 방향 확인 필요) |
| Functions.py | 40cm 이상 전진, 20~40cm 좌우 스캔, 그 이하면 후진 | 흐름만 참고, 그대로 import/실행하지 않음 |

`Functions.py`는 import 단계에서 서보 스레드, 모터 setup, I2C ADC를 초기화한다.
따라서 자율주행 스택에서는 `Functions` 자동주행과 독립 제어 루프를 동시에 실행하지 않는다.
두 주체가 모터 명령을 쓰면 정지 명령이 덮어써질 수 있다.

원본 모터/서보 모두 PCA9685 주소 `0x5f`, 주파수 50Hz를 사용한다.
향후 통합 시 I2C 접근을 직렬화하고 서보 이동 동안 모터를 정지하는 방식부터 검증한다.
`Move.setup()`은 원본 모터 객체를 만들며, 어댑터는 setup을 자동 호출하지 않는다.
활성 드라이버 생성 시 `enabled=True`를 명시해야 하며 기본은 주행 비활성이다.

현재 어댑터는 초음파 함수의 반환 시각을 기록한다. gpiozero 내부 캐시의 마지막 실제 echo
시각을 확인하는 구현은 아직 없다. 따라서 장애/갱신 정지 감지와 독립 watchdog을 추가하기 전에는
이 어댑터의 timestamp만으로 실제 센서 freshness를 보장할 수 없다.
서보를 움직인 직후에는 이전 방향에서 측정한 캐시값을 재사용하지 않도록 해야 한다.

원본 설치 방식과 GPIO/I2C 의존성은 차량 환경을 기준으로 유지한다.
PC의 지도/위치 확인 작업에서는 이 원본 모듈들을 import할 필요가 없다.
