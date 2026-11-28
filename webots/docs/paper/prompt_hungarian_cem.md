# 작업: 학습한 점수 + 헝가리안 최적 매칭 (Hungarian-CEM) 구현과 평가

## 0. 배경

- **기준 브랜치:** `origin/exp/initial-record-errors`(커밋 `8e750e5` 이후)에서 새 브랜치 `exp/hungarian-cem`을 만든다. push는 내가 요청할 때만 한다.
- **현재 결과** (Webots, 처음 보는 3일 × 초기 오류 3수준, 시드 54):
  - 새 CEM 3개 평균은 팀 WES 대비 −13.0초 [−30.3, +1.5], 헝가리안 대비 −0.7초 [−7.3, +7.1]였다.
  - 차이는 주문이 많은 08-22에서만 났다. 01-24, 06-14는 모든 정책이 ±10초 안이다.
- **지금 두 방식:**

  | 정책 | 점수 | 짝짓기 |
  |---|---|---|
  | `lib/sched_learned.py` (CEM) | 작업–로봇 쌍마다 `S = w·φ` | 점수 높은 쌍부터 **탐욕적으로** |
  | `lib/sched_hungarian.py` | 거리만 (작업 우선순위 없음) | 놀고 있는 로봇 × 후보 작업을 `linear_sum_assignment`로 **전체 최적** |

- **아이디어:** CEM이 학습한 점수로 표를 만들고, 그 표를 헝가리안으로 푼다. "무엇이 중요한가"는 학습하고, "누구에게 줄까"는 전체 최적으로 정한다. 두 방식은 **동시에 고를 수 있는 로봇이 2대 이상이고 작업도 2개 이상일 때만** 다른 답을 낸다. 그래서 효과가 있다면 고부하에서 나타날 것이다.
- **결과가 어느 쪽이든 쓴다.** 차이가 없으면 "이득은 짝짓기 방식이 아니라 같은 원인에서 나온다"는 결론이 된다.

## 1. 구현

### 1.1 새 스케줄러 `webots/lib/sched_learned_hungarian.py`

- `sched_learned.Scheduler`를 상속한다. 바꾸는 것은 `Assign.plan`의 **짝짓기 부분뿐**이다. 후보 작업 창(`candidate_tasks`, 30개), `endpoints`, `reroute_waiting`, VERIFY 정책, 예외 처리는 그대로 쓴다.
- **점수 계산:** 지금 탐욕 루프가 만드는 `cands`의 점수(`base + wd·dist/10 + we·eta/60`, 바쁜 로봇은 `+ wb`와 남은 거리 포함)를 **그대로** 쓴다. 공통 함수로 뽑아내 두 스케줄러가 같은 점수를 쓰게 한다(`sched_learned.py`도 그 함수를 쓰도록 리팩터링하되 결과가 바뀌면 안 된다).
- **짝짓기:**
  - 역할(transport/patrol)별로 행 = 놀고 있는 로봇 + 바쁜 로봇(탐욕 방식과 같은 집합), 열 = 후보 작업, 값 = 점수.
  - `linear_sum_assignment(score, maximize=True)`로 푼다.
  - 놀고 있는 로봇과 짝지어진 작업만 EXECUTE로 보낸다. 바쁜 로봇과 짝지어진 작업은 이번 단계에 보내지 않는다(탐욕 방식에서 바쁜 로봇이 이긴 경우와 같은 의미). 이렇게 해야 **짝짓는 방식만 다른** 비교가 된다.
  - 같은 칸을 두 결정이 쓰지 않게, 확정 순서대로 `endpoints`를 다시 확인하고 `taken`을 갱신한다(`sched_hungarian.py`와 같은 방식). 다시 확인했을 때 안 되는 쌍은 버린다.
- **변형 하나:** 환경변수 `ASSIGN_MATCH_ROWS=idle`이면 행을 놀고 있는 로봇만으로 한다(헝가리안 비교군과 같은 행 구성). 기본은 `all`(놀고 있는 + 바쁜 로봇).
- `ASSIGN_COMMIT`(예약)과 부하 적응형 가중치(`w_high/w_low`)는 이 스케줄러에서 지원하지 않는다. 켜져 있으면 오류로 멈춘다.
- 이름: `learned-hungarian-<policy>`. `ASSIGN_WEIGHTS`가 있으면 파일 이름을 넣는다(`sched_learned`과 같은 규칙).

### 1.2 학습 도구 `webots/tools/train_assign.py`

- `--assign greedy|hungarian` 인자를 추가한다. 기본은 `greedy`로 두고 기존 결과가 바뀌면 안 된다.
- `episode()`에서 이 값에 따라 `sched_learned.Scheduler` 또는 `sched_learned_hungarian.Scheduler`를 만든다.
- 가중치 파일에 `"assign": "hungarian"`을 기록한다.

### 1.3 측정 보조 (메커니즘 확인용)

평가 실행 중 매 결정 단계에서 다음을 센다. 실행은 Hungarian-CEM 결정대로 하고, 탐욕 결정은 계산만 한다.

- 동시에 고를 수 있는 놀고 있는 로봇이 2대 이상이고 후보 작업이 2개 이상인 단계의 비율
- 그런 단계에서 Hungarian-CEM과 탐욕 방식(같은 점수)의 결정이 다른 비율

결과를 `summary.json`의 `match_stats`로 남긴다. 다른 정책에는 영향을 주지 않는다.

### 1.4 테스트 (`webots/tests/test_learned_hungarian.py`)

1. **2×2 예시** (로봇 A, B / 작업 P, Q / 점수 A–P 10, A–Q 9, B–P 9, B–Q 2): 탐욕은 A–P, B–Q, Hungarian-CEM은 A–Q, B–P를 고른다.
2. 놀고 있는 로봇이 1대이고 바쁜 로봇이 없는 단계에서는 두 방식의 결정이 같다.
3. 리팩터링 뒤 `sched_learned`의 결과가 이전과 **완전히 같다**(시나리오 3개 × 가중치 2개의 `summary.json` 주요 지표).
4. 경량 시뮬레이터 1에피소드가 예외, `DECISION_REJECTED`, `INVARIANT_VIOLATION` 증가 없이 끝난다.

기존 테스트도 모두 통과해야 한다(`python -m unittest discover -s tests -v`).

## 2. 실험

### 2.1 사전 기록 (결과 보기 전에 커밋)

`webots/docs/experiments_hungarian_cem.md`에 아래를 먼저 적고 커밋한다.

- **가설:**
  - **H-A:** Hungarian-CEM은 같은 예산으로 학습한 탐욕 CEM보다 시험 날짜의 평균 주문 리드타임이 짧다(날짜 단위 쌍대 차이의 95% CI가 0보다 작음).
  - **H-B:** 그 차이는 고부하 날짜(아래 부하 3분위 중 상위)에 몰린다.
  - **판정:** CI가 0을 포함하면 "차이 없음"으로 보고한다. 결과를 보고 가설이나 지표를 바꾸지 않는다.
- **주 지표:** 주문 리드타임(미완료 주문은 3600초까지 기다린 것으로 침)
- **가드레일 (나빠지면 안 됨):** 출하 수, 마감 놓침, p90, 미완료 주문의 최대 나이, 주문 크기별 리드타임(작은/큰 주문 2구간)
- **날짜 분할:** 아래 2.2. 시험 날짜는 마지막에 한 번만 평가한다.

### 2.2 날짜

- `data/split_days30.json`의 후보 72일 중 학습 27일과 기존 시험 3일(01-24, 06-14, 08-22)을 뺀 **42일**을 쓴다.
  - **검증 12일:** 물량 순으로 정렬해 같은 간격으로 뽑는다. 변형 선택(`ASSIGN_MATCH_ROWS` all/idle)에만 쓴다.
  - **새 시험 30일:** 나머지. 경량 시뮬레이터 주 평가에 쓴다.
- 분할을 `data/split_days30_eval.json`에 저장하고(규칙과 날짜 목록 포함), 필요한 `data/orders_<날짜>.json`을 만든다(`tools/prep_dataset.py`, `tools/make_day_split.py` 재사용 또는 확장).
- 기존 시험 3일은 Webots 확인용으로 그대로 둔다. 이미 결과를 본 날짜라는 점을 문서에 적는다.
- **부하 3분위:** 시험 30일을 1시간 에피소드 물량(`split` 파일의 `units` 방식)으로 가벼움/중간/무거움 10일씩 나눈다.

### 2.3 학습

- **설정:** 현재 days30 CEM과 같은 조건이다.
  - 학습 27일 시나리오(`logs/days30/train`), 비용 `mix`, 로봇 8대
  - `--pop 24 --elite 6 --iters 15 --batch 6`
  - 학습 시드 r0, r1, r2
- 이 조건에서 `--assign hungarian`으로 3개를 학습한다: `models/assign_weights_days30_hcem_r{0,1,2}.json`
- **비교 대상:** 이미 있는 `models/assign_weights_days30_cem_r{0,1,2}.json`(탐욕, 같은 조건)
- 검증 12일로 `ASSIGN_MATCH_ROWS` all과 idle 중 하나를 고르고, 고른 이유를 문서에 적는다.

### 2.4 평가 (경량 시뮬레이터)

- **시나리오:** 시험 30일 × 시드 54–58 × 초기 오류 {0.03, 0.065, 0.10}(mixed) × `record_slip_p` 0.01
- **정책:**

  | 정책 | 설정 | 비고 |
  |---|---|---|
  | 팀 WES | `sched_verify`, `VERIFY_POLICY=event` | |
  | 학습 시작점 규칙 | `sched_learned`, `HEURISTIC` 가중치를 **명시적으로** 넘김 | `models/assign_weights.json`이 있으면 기본값이 그 파일이 되므로 주의 |
  | 헝가리안 | `sched_hungarian` | 거리만 |
  | 탐욕 CEM | days30 r0–r2 | |
  | Hungarian-CEM | days30 r0–r2 | |

- **분석 단위는 날짜다.** 날짜마다 시드·오류 수준 평균을 낸 뒤 정책 간 쌍대 차이를 구하고, 날짜 30개를 재표집한 부트스트랩 95% CI를 붙인다(10,000회, RNG 시드 고정).
- 학습 시드별 결과와 시드 3개 평균을 둘 다 보고한다.
- 부하 3분위별 표, 가드레일 표, `match_stats` 요약(고를 수 있는 단계 비율, 결정이 다른 비율)을 같이 낸다.
- **결과 파일:** `logs/hcem_v1/results.csv`, `report.md`, `manifest.json`(실패 포함). 요약은 문서로 옮긴다.

### 2.5 Webots 확인 (경량 평가가 끝난 뒤)

- 기존 Webots 시험과 같은 9개 시나리오(3일 × 오류 3수준, 시드 54)에서 Hungarian-CEM r0–r2를 돌린다. 이미 있는 팀 WES, 헝가리안, 탐욕 CEM 결과와 같은 표 형식으로 붙인다.
- 08-22의 3개 시나리오는 정책마다 3회 반복해 실행 간 흔들림을 잰다.
- `tools/validate_lite.py`로 이 Webots 실행들과 경량 결과의 오차와 순위 일치를 같이 보고한다.

## 3. 규칙

- **커밋 단위:**
  1. 리팩터링 + 새 스케줄러 + 테스트
  2. `train_assign` 옵션
  3. 사전 기록 문서 + 날짜 분할
  4. 학습된 가중치
  5. 평가 실행기·분석기
  6. 결과 문서
- `webots/logs/`는 커밋하지 않는다. `gen_world.py`가 덮어쓰는 `webots/scenario.json`, `webots/worlds/warehouse.wbt`는 끝나면 git으로 되돌린다.
- **금지:**
  - 시험 날짜 결과를 보고 설정·가중치·비용을 바꿔 다시 돌리지 않는다.
  - 비교군만 튜닝하지 않거나 제안 방법만 튜닝하지 않는다. 학습 예산은 탐욕 CEM과 같게 한다.
  - 유의성 승패표를 만들지 않는다.
- 기존 동작(탐욕 CEM, 헝가리안, 팀 WES의 결과)이 바뀌면 멈추고 원인을 보고한다.
- **끝나면 보고할 것:**
  - 바뀐 파일, 테스트 결과
  - 학습 곡선 요약(시드별 최종 비용과 팀 WES 비용)
  - 시험 30일의 주 결과 표: 정책별 팀 WES 대비, 탐욕 CEM 대비 차이와 CI
  - 부하 3분위 표, 가드레일 표, `match_stats`
  - H-A·H-B 판정 한 줄씩
  - Webots 확인 결과
  - 예상과 달랐던 점
