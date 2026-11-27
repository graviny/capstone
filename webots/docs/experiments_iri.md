# 초기 재고 기록 오류(IRI) 민감도 실험 — 사전 기록

작성 2026-11-28. **결과를 보기 전에** 커밋한다. 아래 설계, 지표, 분석은 실행 뒤 바꾸지 않는다.
바꿔야 하면 이 문서에 날짜와 이유를 덧붙이고, 원래 계획의 결과도 함께 보고한다.

## 1. 목적과 범위

- 에피소드가 "기록 = 실제"에서 시작하지 않고, 실사 이후 쌓인 오류를 안고 시작할 때 기존 배정 정책들이 얼마나 나빠지는지 본다.
- **민감도 분석이다. 현실 재현이 아니다.** WERC(2019) 재고 실사 정확도 등급(Destro 등 2023, *Production* 33, e20220077, 부록 C)을 수준을 정하는 근거로만 쓴다. WERC 지표의 측정 단위(위치/품목)를 확인하지 못했으므로 "현실 오류율을 재현한다"고 주장하지 않는다.
- 기록과 실제의 차이는 "마지막 검사 이후 쌓인 오류"라는 정의(Kök & Shang 2007)를 따른다.
- 가설 H4(학습의 이득은 기록 불일치 대응이 아니라 일반적인 작업 순서에서 나온다, `docs/paper/section_plan.md`)를 시험하는 자료로 쓴다.
- CEM 가중치는 재학습하지 않는다.

### 계획에서 바뀐 점 (작업 지시 대비, 결과를 보기 전)

- 작업 지시서(`docs/paper/prompt_initial_record_errors.md`) 2.2의 요인 "진행 중 작업자 오류(off/on)"는 없앴다. 사용자 요청으로 에피소드 중 오류(S1 이동, S2 반출, S3 반품 적치, 로봇 적치 실수, 스캐너 실패) 코드를 저장소에서 지웠기 때문이다(커밋 ee9addd). 따라서 모든 기록 오류는 t=0의 초기 오류뿐이다.
- 3절(처리량 비례 오류 옵션)도 같은 이유로 하지 않는다.
- 오류 수준 0은 구성과 무관하게 같은 시나리오이므로, 기본 시나리오 × 정책마다 한 번만 실행하고 모든 구성의 기준으로 쓴다.

## 2. 오류 정의

| 이름 | 기록 | 실제 | 로봇이 보는 이벤트 |
|---|---|---|---|
| Ghost | SKU A | 빈칸 | SHORT_PICK |
| Hidden | 빈칸 | 상자 있음 | DESTINATION_OCCUPIED |
| Swap | SKU A | SKU B | ITEM_MISMATCH |

- **수준:** `k = round(level × n_stocked)`, `n_stocked`는 `initial_stock`의 랙 칸 수(기본 레이아웃 96칸, 1% ≈ 1칸).
- **구성:** ghost, hidden, mixed(Ghost:Hidden = 1:1, k가 홀수면 Ghost가 하나 더).
- **칸 선택:** `tools/add_record_errors.py`. Ghost는 재고 칸 하나를 무작위로 고르고, 같은 면에서 가장 가까운 빈칸에 그 SKU를 기록한다(같은 면에 빈칸이 없으면 무작위 빈칸). Hidden은 무작위 재고 칸의 기록을 비운다. 한 칸은 한 번만 쓴다. 정렬한 후보에서 `random.Random(err_seed)`로 고른다.
- **err_seed = 시나리오 시드 + 1000.**
- 물리 쪽 `initial_stock`은 바꾸지 않는다. WMS는 오류 칸을 정상 칸과 같은 형태(version, updated_t, observed_t)로 기록한다.

## 3. 설계

| 요인 | 수준 |
|---|---|
| 날짜 | 2023-03-09, 2023-03-22(`--order-sample 0.5`), 2023-08-22 |
| 시나리오 시드 | 48–57 (10개) |
| 초기 오류 수준 | 0, 0.01, 0.03, 0.065, 0.10, 0.20 |
| 구성 | ghost, hidden, mixed |
| 정책 | 팀 기본 WES(`sched_verify`, `VERIFY_POLICY=event`), 헝가리안(`sched_hungarian`), CEM 2일 학습(`models/assign_weights_fault_base_cem_r0.json`), CEM 18일 학습(`models/assign_weights_fault_days18_cem_r0.json`) |

- 로봇 8대, 기본 레이아웃, 3600초, 경량 시뮬레이터(dt 0.5 s, slowdown 1, leg 2 s, yield 1 s).
- 실행 수: 30 기본 시나리오 × (수준 0 한 번 + 5 수준 × 3 구성) × 4 정책 = **1,920회**.
- 기본 시나리오: `gen_world.py --plain --date ... --seed ...`로 만들어 `logs/iri_v1/base/`에 복사한다. 생성 뒤 추적 파일(`scenario.json`, `worlds/warehouse.wbt`)을 되돌린다.
- 실행기: `tools/run_iri_sweep.py`. 결과는 `logs/iri_v1/results.csv`, 실행 목록과 실패는 `logs/iri_v1/manifest.json`.

**알려진 제약 (해석할 때 반영):**
- 같은 날짜의 시드 10개는 출고 주문이 같다. 시드는 입고 트럭과 오류 칸 배치만 바꾼다. 그래서 30개 시나리오는 독립 표본 30개가 아니다. 날짜별 결과를 따로 보고한다.
- 03-09와 03-22는 두 CEM 정책의 학습 날짜다. 08-22만 두 정책 모두 학습에 쓰지 않았다.
- 경량 시뮬레이터는 이제 같은 시나리오면 결정적이다(에피소드 중 난수 오류가 없다). 시나리오 안에서는 반복 실행하지 않는다.

## 4. 지표 (실행마다 한 줄)

- 조건: 날짜, 시나리오 시드, 수준, 구성, 정책, k
- 결과: `order_lead_all_s`(주 지표), `unit_lead_s`, `order_lead_p90_s`, `units_shipped/units_released`, `missed_cutoffs`, 이벤트 SHORT_PICK·DESTINATION_OCCUPIED·ITEM_MISMATCH 수, `record_quality.settled_mismatch_mean`
- 10초마다 랙 칸 전체에서 WMS 기록(`record_snapshot()`)과 실제(`LiteSim.gt_sku`)를 비교해 센 칸 수의 시간 평균:
  - Ghost: 기록 SKU, 실제 빈칸
  - Hidden: 기록 빈칸, 실제 상자
  - Swap: 기록과 실제 SKU가 다름(둘 다 있음)
  - Unknown: 기록이 `UNKNOWN`(점유는 알지만 품목 모름). 위 셋과 따로 센다.

## 5. 분석 (`tools/analyze_iri.py` → `logs/iri_v1/report.md`)

- 분석 단위는 기본 시나리오(날짜 × 시드). 같은 시나리오 안에서 `정책 − 팀 기본 WES`의 주문 리드타임 쌍대 차이를 구한다.
- 수준 × 구성 셀마다 쌍대 차이의 평균과 부트스트랩 95% CI(시나리오 재표집 10,000회, RNG 시드 20261128).
- 수준 0 대비 각 정책의 리드타임 증가(같은 시나리오 쌍대)도 같은 방식으로 구한다.
- 그림: 가로축 초기 오류 수준, 세로축 팀 WES 대비 리드타임 차이, 선은 정책, 구성별 패널.
- 답할 질문:
  1. 초기 오류 수준이 커지면 모든 정책의 리드타임이 얼마나 나빠지는가 (수준 0 대비)
  2. 학습 정책의 이득(팀 WES 대비)이 수준에 따라 커지는가, 줄어드는가, 그대로인가 (H4)
  3. Ghost와 Hidden 중 어느 쪽이 더 큰 손실을 만드는가
  4. 날짜별(특히 학습에 안 쓴 08-22)로 결론이 같은가
- **하지 않는 것:** 유의성 승패표를 만들지 않는다. 결과가 유리해질 때까지 설정을 바꿔 다시 돌리지 않는다. 실패한 실행도 manifest에 남긴다.

## 6. 근거 문헌

- Destro, I. R. et al. (2023). *Production* 33, e20220077. DOI 10.1590/0103-6513.20220077 (부록 C: WERC 2019 재고 실사 정확도 등급 MO < 90%, DIS 90–97%, TYP 97–99.01%, ADV 99.01–99.9%, BIC ≥ 99.9%)
- Kök, A. G. & Shang, K. H. (2007). Inspection and replenishment policies for systems with inventory record inaccuracy. *MSOM* 9(2).

## 7. 결과

(실행 뒤 `logs/iri_v1/report.md`의 요약과 표를 여기 옮긴다.)
