# 작업: 에피소드 시작 시점의 재고 기록 오류(초기 IRI) 실험 추가

## 0. 배경 (먼저 읽을 것)

- **목적.** 이 저장소는 RSS 2027에 제출할 연구의 시뮬레이터다. 1단계(Extended Abstract) 마감은 2026-12-04 AoE다. 연구는 "WMS 기록과 실제 재고가 어긋나는 창고에서 다중 로봇 작업 배정"을 다룬다.
- **먼저 읽을 문서:**
  - `webots/docs/paper/rss2027_outline.md`, `webots/docs/paper/section_plan.md`: 논문 틀과 가설 H1–H5
  - `webots/docs/wms.md`: WMS 기록 구조와 이벤트
  - `webots/docs/lite_sim.md`: 경량 시뮬레이터
  - `webots/docs/research_protocol.md`: 통계·재현성 원칙
  - `webots/docs/experiments.md`: 시나리오 생성 옵션
- **지금의 문제.** 모든 에피소드가 "기록 = 실제"인 완벽한 상태에서 시작한다. 1시간 동안 작업자 오류(S1 이동, S2 반출, S3 반품 적치)로 괴리가 조금씩 쌓일 뿐이다.
  - 경량 시뮬레이터 기본 설정(03-09, 시드 20개)에서 괴리 칸 수의 시간 평균은 Ghost 약 3.5칸, Hidden 약 1.6칸, SKU 바뀜 약 0.07칸이었다(랙 160칸).
  - 실제 창고는 실사 이후 쌓인 오류를 안고 운영된다. Kök & Shang(2007)은 기록과 실제의 차이를 "마지막 검사 이후 쌓인 오류"로 정의한다.
- **근거로 쓸 범위.** Destro 등(2023, *Production* 33, e20220077, DOI 10.1590/0103-6513.20220077)의 부록 C는 WERC(2019) 창고 벤치마크의 재고 실사 정확도(Inventory Count Accuracy) 등급 기준을 싣고 있다.

  | 등급 | 정확도 | 부정확률로 환산 |
  |---|---|---|
  | MO | < 90% | > 10% |
  | DIS | 90–97% | 3–10% |
  | TYP | 97–99.01% | 약 1–3% |
  | ADV | 99.01–99.9% | 0.1–1% |
  | BIC | ≥ 99.9% | ≤ 0.1% |

  이 범위를 실험 수준을 정하는 근거로 쓴다. **단, WERC 지표의 측정 단위(위치/품목)는 확인되지 않았으므로 "현실 값을 재현한다"고 주장하지 않는다.** 실험은 미지의 오류 수준에 대한 **민감도 분석**이다.
- **용어.**

  | 이름 | 기록 | 실제 | 로봇이 보는 이벤트 | 우리 이전 명칭 |
  |---|---|---|---|---|
  | Ghost | SKU A | 빈칸 | SHORT_PICK | 유형 ① |
  | Hidden | 빈칸 | 상자 있음 | DESTINATION_OCCUPIED | 유형 ② |
  | Swap | SKU A | SKU B | ITEM_MISMATCH | 유형 ③ |

## 1. 구현 (필수)

### 1.1 새 도구 `webots/tools/add_record_errors.py`

기존 `scenario.json` 하나를 읽어, **기록 쪽 초기 오류만** 추가한 새 시나리오 파일을 쓴다.

- **`gen_world.py`는 수정하지 않는다.** `gen_world.py`는 `webots/scenario.json`과 `webots/worlds/warehouse.wbt`(Git 추적 파일)를 덮어쓴다. 실험용 시나리오는 모두 `webots/logs/` 아래(gitignore됨)에 만든다.
- **인자:**

  | 인자 | 뜻 |
  |---|---|
  | `--scenario <in.json>` | 입력 시나리오 |
  | `--out <out.json>` | 출력 시나리오 |
  | `--level <비율>` | 예: 0.03 |
  | `--mix ghost|hidden|swap|mixed` | 오류 유형 구성. `mixed`는 Ghost:Hidden = 1:1 |
  | `--err-seed <int>` | 오류 배치용 난수 시드 |

- **수준 정의:** 오류 칸 수 `k = round(level × 초기 재고 칸 수)`. 기본 시나리오의 초기 재고는 96칸이므로 1% ≈ 1칸이다. 이 정의를 문서와 출력 파일에 기록한다.
- **칸 선택 규칙** (`kind == "rack"` 칸만, 같은 칸을 두 번 고르지 않음):

  | 유형 | 고를 칸 | 기록에 쓸 내용 |
  |---|---|---|
  | Ghost | 실제로 빈 랙 칸 | SKU 하나. **같은 SKU가 이미 있는 칸에서 가까운 빈칸을 우선**하고, 없으면 무작위 |
  | Hidden | 실제로 상자가 있는 칸 | 빈칸 |
  | Swap | 실제로 상자가 있는 칸 | 다른 SKU (같은 면에서 가까운 다른 칸의 SKU 우선) |

- **선택 방식:** 대상 칸 목록을 정렬한 뒤 `random.Random(err_seed)`로 고른다. 정책과 무관하게 같은 입력이면 같은 출력이 나와야 한다.
- **출력:**
  - 시나리오에 `"initial_record_errors": [{"slot", "type", "record_sku"}...]`와 `"initial_record_errors_meta": {level, mix, err_seed, k, n_stocked, definition}`을 추가한다.
  - **물리 쪽 `initial_stock`은 절대 바꾸지 않는다.**

### 1.2 `webots/lib/wms.py`의 `MockWMS.__init__`

- `initial_stock`을 기록에 반영한 직후(현재 73–74행)에 `initial_record_errors`를 적용한다.
- **오류 칸이 정상 칸과 구별되지 않아야 한다.** 스케줄러가 `version`, `updated_t`, `observed_t`를 쓰기 때문이다(예: `lib/sched_verify.py` 195행이 `updated_t`를 씀).

  | 유형 | 기록 필드 |
  |---|---|
  | Ghost | `item=SKU, lpn=None, version=1, updated_t=0.0, observed_t=None` (초기 재고 칸과 같은 형태) |
  | Hidden | `item=None, lpn=None, version=0` (빈칸과 같은 형태) |
  | Swap | `item=다른 SKU, lpn=None, version=1` |

- 이 적용을 WES가 볼 수 있는 로그·이벤트·카운트에 남기지 않는다. `wms_log.jsonl`의 START 항목에 개수만 남기는 것은 괜찮다. 분석용 진실 기록은 시나리오 파일 자체로 충분하다.
- `lpn=None`인 기록 때문에 PICK·RELOCATE·검증 로직이 깨지지 않는지 확인한다. 깨지면 가장 작은 수정으로 고치고 이유를 커밋 메시지에 쓴다.
- 레거시 `lib/wes_standin.py`도 `initial_stock`을 직접 읽는다(32행). 같은 오류를 적용하거나, 이 기능이 `wes_runner`(WMS 경유) 경로에서만 지원된다고 docstring에 적는다.
- 물리 쪽(`lib/lite_sim.py` 105행, `controllers/supervisor/supervisor.py` 142행)은 `initial_stock`만 읽으므로 수정하지 않는다.

### 1.3 테스트 (`webots/tests/test_record_errors.py`, Webots 없이 실행)

1. 같은 입력과 `err_seed`면 출력이 같다. `level=0`이면 오류가 0개다.
2. 유형별 개수와 칸 조건(Ghost는 실제 빈칸, Hidden·Swap은 실제 상자 칸)이 맞다. `initial_stock`은 입력과 동일하다.
3. 오류 칸과 정상 칸의 `version`, `updated_t`, `observed_t`가 구별되지 않는다(`WMSView.snapshot()` 기준).
4. 경량 시뮬레이터 1에피소드를 `level=0.20, mix=mixed`로 끝까지 돌려도 예외가 없다. 또 `summary.json`의 `diagnostics`에 `INVARIANT_VIOLATION`이 새로 생기지 않는다.
5. `level=0`의 결과가 기능 추가 전과 **완전히 같다**(같은 시나리오·시드의 `summary.json` 주요 지표 비교).

기존 테스트도 모두 통과해야 한다: `cd webots && python -m unittest discover -s tests -v`.

## 2. 실험 (경량 시뮬레이터)

### 2.1 사전 기록 (실행 전에 커밋)

`webots/docs/experiments_iri.md`를 만들고, **결과를 보기 전에** 아래를 적어 커밋한다.

- 목적, 수준 정의, 요인과 수준, 지표, 분석 방법
- "현실 재현이 아니라 민감도 분석"이라는 범위
- 근거 문헌(위 0절)

### 2.2 설계

| 요인 | 수준 |
|---|---|
| 기본 시나리오 날짜 | 2023-03-09, 2023-03-22(`--order-sample 0.5`), 2023-08-22 |
| 시나리오 시드 | 48–57 (10개, CEM 학습에 쓴 42–47과 겹치지 않게) |
| 초기 오류 수준 | 0, 0.01, 0.03, 0.065, 0.10, 0.20 |
| 유형 구성 | ghost, hidden, mixed (swap은 시간이 남으면 추가) |
| 진행 중 작업자 오류 | off(`--no-faults`로 만든 기본 시나리오), on(기본 S1–S3) |
| 정책 | 아래 표 |

| 정책 | 설정 | 비고 |
|---|---|---|
| 팀 기본 WES | `sched_verify`, `VERIFY_POLICY=event` | `train_assign.py`의 stand-in과 같음 |
| 헝가리안 | `sched_hungarian` | |
| CEM (2일 학습) | `sched_learned`, `ASSIGN_WEIGHTS=models/assign_weights_fault_base_cem_r0.json` | |
| CEM (18일 학습) | `sched_learned`, `ASSIGN_WEIGHTS=models/assign_weights_fault_days18_cem_r0.json` | |

- 로봇 8대, 기본 레이아웃, 3600초.
- `err-seed`는 시나리오 시드에서 결정적으로 만든다(예: 시드 + 1000).
- **규모:** 30개 기본 시나리오 × 6 수준 × 3 구성 × 2 진행 오류 × 4 정책 = 4,320회. 1회 약 1–2초라 병렬로 1–2시간이다.
- **실행기:** `webots/tools/run_iri_sweep.py`를 만든다(`multiprocessing.Pool` 사용, `train_assign.py`의 `episode()` 구성 참고).
  - 기본 시나리오 생성: `gen_world.py --plain --date ... --seed ...`로 만든 `scenario.json`을 `logs/iri_v1/base/`로 **복사**한다.
  - 생성이 끝나면 `git checkout webots/scenario.json webots/worlds/warehouse.wbt`로 추적 파일을 되돌린다.
- **CEM 가중치에 대해:** 재학습하지 않는다. 이 실험은 기존 정책들의 초기 오류 민감도를 보는 것이다.

### 2.3 기록할 것 (실행마다 한 줄, `logs/iri_v1/results.csv`)

- 실험 조건: 날짜, 시나리오 시드, 수준, 구성, 진행 오류, 정책, `k`
- 결과 지표: `order_lead_all_s`(주 지표), `unit_lead_s`, `order_lead_p90_s`, `units_shipped/units_released`, `missed_cutoffs`, `events`의 SHORT_PICK·DESTINATION_OCCUPIED·ITEM_MISMATCH, `record_quality.settled_mismatch_mean`
- **Ghost/Hidden/Swap 칸 수의 시간 평균:** 10초마다 WMS 기록(`record_snapshot()`)과 실제(`LiteSim.gt_sku`)를 비교해 센다. `LiteSim.record_quality`를 감싸서 수집한다.

### 2.4 분석 (`webots/tools/analyze_iri.py` → `logs/iri_v1/report.md`)

- **분석 단위는 기본 시나리오(날짜 × 시드)다.** 같은 시나리오 안에서 정책 간 쌍대 차이를 계산한다: `정책 − 팀 기본 WES`, 주문 리드타임.
- 수준 × 구성 × 진행 오류 셀마다 쌍대 차이의 평균과 부트스트랩 95% CI를 구한다(시나리오 재표집 10,000회, 부트스트랩 RNG 시드 고정).
- **그림:** 가로축 초기 오류 수준, 세로축 팀 WES 대비 리드타임 차이, 선은 정책. 구성별로 패널을 나눈다.
- **확인할 질문:**
  1. 초기 오류 수준이 커지면 모든 정책의 리드타임이 얼마나 나빠지는가 (수준 0 대비)
  2. 학습 정책의 이득(팀 WES 대비)이 수준에 따라 커지는가, 줄어드는가, 그대로인가 (가설 H4)
  3. Ghost와 Hidden 중 어느 쪽이 더 큰 손실을 만드는가
  4. 날짜별(특히 학습에 안 쓴 08-22)로 결론이 같은가
- **금지:** 유의성 승패표를 만들지 않는다. 결과가 유리해질 때까지 설정을 바꿔 다시 돌리지 않는다. 실패한 실행도 manifest에 남긴다.

## 3. 선택 작업 (시간이 남으면, 별도 커밋)

**진행 중 작업자 오류를 처리량에 비례시키는 옵션**

- 지금 S1·S2는 시간당 횟수(`worker_move_per_h`, `worker_remove_per_h`)다.
- 시나리오 `faults`에 `rate_basis: "per_hour" | "per_task"`를 추가한다. `per_task`면 완료된 작업 1건당 확률로 시도한다.
- 기본값은 `per_hour`로 두고 기존 결과가 바뀌지 않게 한다.
- `lib/fault_tape.py`, `lib/lite_sim.py`, `controllers/supervisor/supervisor.py` 세 곳의 의미가 같아야 한다(`apply_worker_attempt` 공유 구조 유지).
- 근거는 Destro 등(2023)의 "작업량 × 오류율" 방식이다. 이 옵션은 작업자가 아니라 **사람 개입 빈도**를 처리량에 비례시키는 것임을 docstring에 적는다. 로봇은 오류를 내지 않는다는 가정 A1을 유지한다.

## 4. 작업 규칙

- 새 브랜치 `exp/initial-record-errors`에서 작업한다. main에 직접 커밋하지 않고, push는 내가 요청할 때만 한다.
- 커밋 단위:
  1. 도구 + WMS 적용 + 테스트
  2. 사전 기록 문서
  3. 실행기·분석기
  4. 결과 보고서
- `webots/logs/`는 커밋하지 않는다(gitignore). 보고서 요약과 표는 `webots/docs/experiments_iri.md`에 옮겨 적는다.
- 명령은 `webots/` 폴더에서 실행한다. Windows PowerShell이면 환경변수는 `$env:VERIFY_POLICY="event"`처럼 쓴다.
- 기존 기본 동작(오류 0일 때 결과)이 바뀌면 안 된다. 바뀌면 멈추고 원인을 보고한다.
- **끝나면 보고할 것:**
  - 바뀐 파일 목록
  - 테스트 결과
  - 실행 수와 실패 수
  - `report.md`의 핵심 표 1개와 그림 1개
  - 2.4의 질문 1–4에 대한 한 줄 답
  - 예상과 달랐던 점
