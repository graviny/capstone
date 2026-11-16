# 경량 시뮬레이터 (`lib/lite_sim.py`)

Webots 없이 같은 창고를 빠르게 돌리는 시뮬레이터예요. **WES 학습(강화학습)과 많은 조건을 훑는 실험**에 써요. 결과 확인은 Webots에서 해요.

- 같은 `scenario.json`, 같은 WES 인터페이스(`on_event` / `decide`), 같은 이벤트와 결정을 써요. 그래서 여기서 만든 스케줄러는 **고치지 않고** Webots에서 돌아가요 (`WES_MODULE=wes_runner WES_SCHEDULER=<모듈>`).
- 같은 `summary.json` 항목을 써요. 로그 폴더 이름은 `..._lite`예요.
- 1시간 에피소드가 **약 1초**예요 (Webots는 약 8.5분). 처음 한 번은 거리 지도를 계산하느라 10~15초가 걸려요.

## 실행

```bash
cd webots
python lib/lite_sim.py --scheduler sched_verify --policy event --seeds 0 1 2
```

코드에서 (예: 강화학습 스케줄러):

```python
from lite_sim import LiteSim
from nav import GridMap
from wes_runner import WES

grid, fields = GridMap(cfg), {}          # 여러 에피소드가 같이 쓰면 거리 지도를 한 번만 계산
wes = WES(cfg, scheduler=my_scheduler, log_dir=tmp_dir)
summary = LiteSim(cfg, wes, grid=grid, fields=fields).run()
```

강화학습 정책은 `Scheduler` 클래스로 만들어요 (`plan(state)`가 결정을 돌려줌). 경량 시뮬과 Webots에서 같은 클래스를 써요.

## Webots와 같은 것, 다른 것

| | 같음 | 다름 |
|---|---|---|
| 환경 | 주문 웨이브, 입고 트럭과 컨베이어, 출고 트럭과 마감 | |
| 오류 | 작업자 S1 이동, S2 반출, S3 반품 적치 | 같은 seed라도 발생 시각은 Webots와 달라요 (아래) |
| 로봇 | 작업 단계(이동, 확인, 집기, 이동, 확인, 놓기)와 보고, 통행 우선순위 규칙 | 바퀴, 충돌 회피 없음. 이동 시간 = 경로 길이 ÷ 속도 + 2초 + 1초 × (출발할 때 움직이고 있는, 나보다 통행 우선순위가 높은 로봇 수) |
| 시간 | | 0.5초 단위 (Webots 32 ms). 로봇 단계는 정확한 시각에 끝남 |
| 인식 | | 라벨을 항상 읽음 (UNREADABLE 없음) |

모델링하지 않은 것: 로봇 놓기 오류(S1_PLACE_ERROR), 입고 판독 실패, 순찰 로봇의 선반 훑기. 모두 기본값이 꺼져 있어요.

## 보정 (Webots 결과와 비교)

이동 시간에 두 값을 더해요.
- `leg_s` 2초: 이동마다 드는 회전과 정렬 시간
- `yield_s` 1초: 출발할 때 움직이고 있는 로봇 중, 나보다 통행 우선순위가 높은 로봇 1대당 양보 시간. 우선순위는 Webots와 같은 규칙이에요 (`nav.traffic_priority`: `task` 또는 `id`).

Webots 실행 47개(속도 0.3·0.8 m/s, 로봇 4·8대, 순찰 7+1, 정책 none·event·periodic, 오류 1·4배, 스케줄러 3종, 모두 번호 우선 통행)에 맞춰 두 값을 함께 정했어요 (2026-11-16). 아래 표는 작업 우선 통행 실행 4개를 더한 51개로 검증한 값이에요 (경량 seed 5개 평균). 처음에는 이동마다 고정 5초를 썼는데, 로봇이 적은 조건에서 가동률을 크게 높게 봤어요.

| | 고정 5초 (이전) | 2초 + 양보당 1초 (지금) |
|---|---|---|
| 운반 가동률 평균 오차 | 2.5% | 2.2% |
| 운반 가동률 최대 오차 | 18.1% | 8.2% |
| 로봇 4대에서 가동률 편향 | +16% | +5% |
| 평균 주문 처리 시간 평균 오차 | 5.7% | 5.4% |

검증은 `python tools/validate_lite.py`로 다시 돌릴 수 있어요 (결과: `logs/lite_validation.md`). 정확도, 정책 간 순위 일치, 오류 지표를 Webots 실행과 비교해요.

**쓸 수 있는 범위 (검증 결과):**
- 처리량, 처리 시간, 가동률, 배정 방식 비교: 쓸 수 있어요. 조건 단위 순위 일치는 가동률 100%, 처리 시간 92%, 출하 86%예요. 다만 로봇 4대에서 배정 방식 간 처리 시간 차이를 Webots의 절반 정도로 작게 봐요.
- 확인 정책 비교(틀린 기록, SHORT_PICK, DESTINATION_OCCUPIED): 아직 조심해야 해요. 순위 일치가 57~82%이고, Webots 쪽도 seed 3개라 흔들림이 커서 판단하기 어려워요.

**오류 관련 수치(SHORT_PICK, DESTINATION_OCCUPIED, 틀린 기록)는 Webots와 1:1로 맞지 않아요.** 작업자 오류가 매 스텝 난수로 정해져서, 시간 간격이 다르면 발생 시각과 위치가 달라지기 때문이에요. 경량 시뮬 안에서 정책끼리 비교하는 것은 공정해요 (같은 seed면 같은 결과). 오류 효과를 Webots와 직접 비교하려면 작업자 오류를 미리 정한 일정표로 바꿔야 해요.

물동량이나 로봇 수를 크게 바꾸면 Webots 실행과 다시 비교해서 `--leg-s`, `--yield-s`를 맞춰요.
