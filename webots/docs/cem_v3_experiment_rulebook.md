# CEM 신발 주문 실험 운영 규약 v1

작성일: 2026-11-28. 네 명이 같은 입력과 판정 기준으로 실험하기 위한 실행 문서다. 이 문서의 수치 설정은 **우리의 사전 결정**이며 선행논문의 수치를 그대로 재현한 것이라고 주장하지 않는다. 결과를 본 뒤 바꾼 설정은 별도 실험 버전으로 남긴다.

## 1. 이번 논문의 질문과 실험 경계

질문: 신발 주문을 처리하는 합성 창고에서 WMS의 재고 기록이 틀렸을 때, CEM으로 학습한 **로봇 작업 배정 점수**가 고정 규칙보다 출고 대기를 줄이는가?

- CEM은 `lib/sched_learned.py`의 배정 가중치 11개만 학습한다. 확인 작업을 언제 만들지, 실제 물건을 어디에 놓을지는 학습하지 않는다.
- 주입 오류는 시작 시 기록 불일치(`initial_record_errors`)와 실제 위치 A에 놓고 WMS만 B로 기록하는 `record_slip_p`다. **로봇이 지시와 다른 칸에 실제로 놓는 동작은 넣지 않는다.**
- 정책 간 시나리오와 난수 시드는 같게 둔다. 적치 순서가 정책에 따라 달라져 실제 기록 미끄러짐의 시각·대상·건수는 달라질 수 있다. "모든 정책에 같은 오류 사건이 발생했다"고 쓰지 않는다.
- 신발 데이터가 제공하는 것은 주문 날짜·시각·SKU·수량이다. 입고, 초기 재고, 마감, 오류율은 합성 설정이다. 실물 창고 성능이나 실제 오류율로 해석하지 않는다.
- 경량 시뮬레이터는 CEM 후보 학습에 쓴다. 최종 정책 우열은 Webots에서 판단한다. 현재 v3 개발 조건의 주 지표에서 경량과 Webots의 정책 순위가 뒤집힌 사례가 있으므로 경량 성능만으로 승자를 선언하지 않는다.

## 2. 실험 시작 전 동결할 항목

| 항목 | 이번 버전의 규칙 |
|---|---|
| 날짜 | [`experiments/split_v3.json`](../experiments/split_v3.json): 학습 123일, 검증 24일, 시험 24일. 나머지 7일은 표본 생성 불가로 별도 기록. 날짜 간 재고는 이어받지 않는다. |
| 입력 | `tools/gen_world.py`의 v3 설정: 전체 SKU, 날짜별 주문을 통째로 최대 100개 단위까지 추출, 1시간, 웨이브 간격 0초, 로봇 8대, `stock_layout=abc`. 상세 값은 분할 파일의 `generator`에 있다. |
| 주 비교 조건 | 무오류: 초기 오류 수준 0, `record_slip_p=0`. 오류: 초기 오류 수준 0.065, `mix=mixed`, `record_slip_p=0.01`. 같은 날짜·시드는 정책 간 동일하다. 0.065와 0.01은 실측 오류율이 아닌 민감도 설정이다. |
| 오류 강도 추가 분석 | 0.03, 0.10 및 `record_slip_p=0` 분해 실험은 주 비교와 구분한다. 주 결과를 본 뒤 유리한 강도만 고르지 않는다. |
| 학습 | CEM, `--cost backlog_auc --iters 15 --pop 24 --elite 6 --batch 24 --robots 8`, 독립 최적화 난수 `--rng 0`, `1`, `2`. 현재 코드 기본 비용 `mix`는 사용하지 않는다. `batch=24`는 계산 예산에 대한 팀 결정이다. |
| 공통 정책 설정 | 모든 정책에서 `WES_MODULE=wes_runner`, `VERIFY_POLICY=event`, 동일한 WMS·로봇 수·지형·시간 제한. 다른 `VERIFY_*` 환경값도 고정하고 로그에 기록한다. |
| 대조군 | 팀 WES `sched_verify`, 매칭 대조군 `sched_hungarian`, CEM `sched_learned`의 r0/r1/r2. 세 CEM 실행 중 제일 좋은 것 하나만 골라서 학습 알고리즘 전체의 평균 성능이라고 쓰지 않는다. |
| 주 지표 | `research_metrics.unit_backlog_auc_s`(낮을수록 좋음)와 `mis_shipped`(0이어야 함). 대기 적분에는 마감 때 미완료 수량도 포함한다. |
| 보조 지표 | 출고 수량, 종료 시 미완료 주문·유닛, 완료/미완료 주문의 제한 관측 시간, 기록 불일치·복구, 확인 횟수, 이동 거리, 실행 시간. 보조 지표에서만 이긴 경우 주 지표 승리로 바꾸어 쓰지 않는다. |

**동결 기록:** 각 실행 묶음의 `manifest.json`에 Git 커밋 SHA, 분할 파일 SHA-256, 생성기 인수, 오류 설정, CEM 인수·난수, 정책 가중치 파일 SHA-256, Webots 버전, 실행 명령, 시작/종료 시각을 적는다. `scenario.json`, `run_env.json`, `summary.json`, `events.jsonl`, 실패 로그를 함께 보존한다. 모델 파일은 r0/r1/r2 각각 별도 경로에 저장한다.


## 3. 단계별 진행과 데이터 접근 규칙

1. **입력 생성:** A가 분할 파일을 복사해 고정하고 무오류/오류 시나리오를 별도 디렉터리에 생성한다. `tools/make_day_split.py`는 추적 중인 `scenario.json`과 `worlds/warehouse.wbt`를 일시적으로 덮어쓰므로 Webots가 열린 작업트리에서 실행하지 않는다. 종료 후 두 파일이 원래 해시로 복원됐는지 확인한다.
2. **학습:** B는 `train` 디렉터리만 CEM에 전달한다. `train_assign.py`의 `--test`와 `--report`는 여기서 쓰지 않는다. 이 옵션은 시험 입력을 즉시 열기 때문에 잘못 사용하면 시험 집합이 개발에 노출된다. 실행 명령과 각 반복의 비용·가중치를 보존한다.
3. **검증:** C가 검증 24일의 동일 시나리오에서 모든 후보 정책을 Webots로 짝지어 실행한다. D가 주 지표를 보고 배포 후보 한 개를 고른다. 검증 날짜는 하이퍼파라미터·후보 선택에 써도 되지만 이후부터 그 결과를 최종 미관측 성능이라고 쓰지 않는다.
4. **시험:** 후보, 대조군, 오류 조건, 지표, 분석 코드를 고정한 뒤 시험 24일을 한 번 연다. 시험 결과를 보고 CEM 비용·가중치·오류 강도를 바꾸면 새 실험 버전으로 기록하고, 기존 시험 결과는 더 이상 미관측 시험 결과가 아니다.
5. **논문 표:** Webots 시험 결과만 정책 우열의 주 근거로 쓴다. 경량 결과와 Webots 결과의 정책 순위 일치율은 별도 진단 표에 놓는다.

입력 생성 예시 (`webots/`에서, 실행 전 다른 사람이 설정을 검토):

```powershell
python tools/make_day_split.py experiments/split_v3.json --levels 0 --slip-p 0 --out logs/v3_nominal
python tools/make_day_split.py experiments/split_v3.json --levels 0.065 --mix mixed --slip-p 0.01 --out logs/v3_errors
```

학습 명령 형식(각 r마다 `--rng`와 `--out`만 바꾼다):

```powershell
python tools/train_assign.py --train 'logs/v3_nominal/train/*.json' 'logs/v3_errors/train/*.json' --cost backlog_auc --iters 15 --pop 24 --elite 6 --batch 24 --robots 8 --rng 0 --out models/cem_v3_r0.json
```

검증·시험에서 시나리오 JSON을 Webots 월드로 다시 만드는 실행기를 사용할 때는 **오류가 주입된 원본 시나리오를 supervisor가 실제 읽었는지** 확인한다. `gen_world.py --config`만 실행하면 오류 필드가 없는 새 시나리오를 쓸 수 있다. 이 확인 없이 대량 실행하지 않는다.

## 4. 판정과 통계

- **짝짓기 단위:** 날짜·시나리오 시드·오류 조건이 동일한 정책 실행을 한 쌍으로 묶는다. 한 정책만 끝난 시나리오는 성공한 정책만 남기지 않고 실패/누락으로 표시한다. 실행기 오류라면 원인 수정 후 같은 고정 입력으로 재실행한다. 정책 자체의 실패라면 결과로 보고하고 임의로 제외하지 않는다.
- **정확성 제약:** `mis_shipped>0`인 정책을 처리 시간만 보고 승자로 선택하지 않는다. 정확성 위반 건수와 원인을 별도 보고한다.
- **차이의 방향:** 각 시나리오에서 `CEM AUC - 대조군 AUC`를 계산한다. 음수는 CEM이 낫다는 뜻이다. 날짜별 차이를 먼저 만들고 그 평균과 95% 구간을 보고한다. 주문·로봇·시간 스텝을 독립 반복으로 세지 않는다.
- **불확실성:** 날짜를 재표본추출하는 짝지은 부트스트랩 10,000회, 분석 난수 고정. 같은 날짜에 여러 시드/오류 조건이 있다면 날짜를 군집 단위로 뽑는다. 구간에 0이 들어가면 우열을 단정하지 않는다. 3개 CEM 학습 난수의 결과도 각각 표시한다.
- **비교 가족:** 주 비교는 CEM 대 팀 WES, CEM 대 헝가리안의 두 개다. 두 비교와 주 지표를 미리 고정한다. 추가 대조군·오류 강도·보조 지표는 탐색/민감도 분석이라고 명시한다. 여러 비교의 95% 구간을 각각 계산해 모두 독립적인 확증이라고 주장하지 않는다.
- **범위:** 결과는 이 합성 창고·신발 주문 표본·오류 설정·Webots 구현에 대한 것이다. 물리 창고 검증 또는 sim-to-real 성공이라고 쓰지 않는다.

## 5. 실행 전/후 중단 조건

**대량 실행 전:** 분할 겹침 0, 각 조건의 날짜 수 일치, 오류 수준 0에서 초기 오류 0건, `record_slip_p=0`에서 주입 0건, 오류 조건의 실제 주입 건수 기록, `unservable_units_lower_bound` 기록, 정책별 동일 시나리오 해시, 가중치 파일 존재·해시 일치, `--cost backlog_auc` 확인. 이 중 하나라도 틀리면 수정하고 입력을 다시 생성한다.

**각 Webots 실행 후:** `summary.json`과 `events.jsonl` 존재, `duration_s=3600`, 주문 공개 수량과 시나리오 수량 일치, `run_env.json`의 정책·가중치·VERIFY 설정 일치, `mis_shipped` 기록. 중도 종료·누락 파일·로봇 정체는 숨기지 않고 원인과 재실행 여부를 manifest에 적는다.

**최종 표 전:** 정책별 날짜·시드·오류 조건 수가 같고, 시험 날짜가 학습/검증/튜닝 로그에 없는지 D와 A가 교차 확인한다. 과거 90초 웨이브·상위 SKU·S1/S2/S3 실험 결과를 v3 반복 측정으로 합치지 않는다.

## 6. 선행연구에서 가져온 원칙과 우리 선택의 구분

- [de Boer et al., *A Tutorial on the Cross-Entropy Method* (2005)](https://ris.utwente.nl/ws/portalfiles/portal/6700787/fulltext.pdf): 후보를 뽑고 우수 후보로 분포를 갱신하는 CEM 절차. `15/24/6` 같은 우리 실행 예산의 근거로 인용하지 않는다.
- [Patterson et al., *Empirical Design in Reinforcement Learning* (JMLR 2024)](https://www.jmlr.org/papers/v25/23-0183.html): 하이퍼파라미터 선택, 반복 변동, 실험자 편향, 대조군 명시. 날짜 분리와 변경 이력 기록에 반영했다.
- [Agarwal et al., *Deep Reinforcement Learning at the Edge of the Statistical Precipice* (NeurIPS 2021)](https://proceedings.neurips.cc/paper_files/paper/2021/hash/f514cec81cb148559cf475e7426eed5e-Abstract.html): 적은 반복의 점 추정만으로 결론 내리지 않고 구간을 보고한다.
- [Choudhury et al., *Dynamic Multi-Robot Task Allocation under Uncertainty and Temporal Constraints* (RSS 2020)](https://www.roboticsproceedings.org/rss16/p068.html): 유한 운영 시간의 미완료 작업을 결과에 포함하는 배정 문제. 우리 AUC 정의 자체를 그 논문에서 가져왔다고 주장하지 않는다.
- [Li et al., *Evaluating Real-World Robot Manipulation Policies in Simulation* (CoRL 2025)](https://proceedings.mlr.press/v270/li25c.html): 시뮬레이터가 정책 간 상대 성능을 보존하는지 따로 확인한다. 우리 비교의 참조는 Webots이며 실물이 아니다.
