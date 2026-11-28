# RSS 2027 논문 틀 (근거 기반 재구성) — 2026-11-27

이 문서는 세 가지를 정리한다.

1. RSS 2024–2026 메인트랙에서 우리 분야와 가까운 논문 22편의 구성과 흐름 (원문 PDF로 확인)
2. 그 흐름과 RSS 2027의 바뀐 제출 규정에 맞춘 우리 논문의 섹션별 틀. 설정과 주장마다 근거 문헌과 그 확인 상태를 단다.
3. 지금 실험의 부족한 점과 다시 할 실험 목록

표기 규칙은 다음과 같다.

| 표기 | 뜻 |
|---|---|
| **[원문]** | 이번에 PDF 원문을 직접 읽고 확인한 내용 |
| **[서지]** | 출판사·색인 페이지에서 서지와 초록만 확인한 내용. 본문 수치를 쓰기 전에 원문을 다시 봐야 한다 |
| **[근거 없음]** | 근거 문헌을 찾지 못한 내용. 가정으로 밝히거나 민감도 실험으로 바꿔야 한다 |

---

## 0. 먼저 알아야 할 것: RSS 2027은 제출 방식이 바뀌었다

출처는 RSS 2027 공식 [Call for Papers](https://roboticsconference.org/information/cfp/), [Reviewer Guidelines](https://roboticsconference.org/information/reviewer-guidelines/), [Author FAQ](https://roboticsconference.org/information/author-faq/)이다 **[원문]**.

| 항목 | 내용 |
|---|---|
| 1단계 마감 | **2026-12-04 23:59 AoE** (Extended Abstract) |
| 1단계 분량 | **본문 5쪽 + 참고문헌 1쪽**. 부록도 5쪽 안에 들어가야 하고, 웹사이트·영상·보충자료는 보지 않는다 |
| 1단계에서 요구하는 것 | 문제 정의, **가설**, 방법론은 완성돼 있어야 한다. 실험은 없거나 예비 결과여도 된다 |
| 관련 연구 | 1단계에서는 핵심 문제와 신규성에 **직접 관련된 문헌만** 인용한다. 포괄적인 관련 연구는 2단계에서 쓴다 |
| 심사 점수 | ① Value of Hypothesis (1–4) ② Feasibility of Conclusion (1–4): 계획한 실험이 가설을 지지하거나 반증할 수 있는가 |
| 이후 일정 | 02-05 1차 결정 → 02-12 반박서 → 02-26 Extension Charter(심사위원이 정한 추가 실험 목록) → **04-16 최종본(8쪽 + 참고문헌)** |
| 부정적 결과 | "부정적 결과는 긍정적 결과와 같은 가치로 평가한다." 다만 성능이 낮았다는 보고만으로는 부족하다. **실패 원인 후보를 세우고, 그 후보들을 구별하는 실험을 설계해야 한다** (Author FAQ 2번) |
| 주의 | **존재하지 않는(환각) 참고문헌이 있으면 자동 검사로 데스크 리젝트**된다. 익명성 위반도 데스크 리젝트 사유다 |

**우리에게 주는 의미**

- 지금 결과는 "학습 날짜에서는 7% 개선, 처음 보는 날짜에서는 개선 없음"이다. 기존 형식에서는 약점이지만, RSS 2027 형식에서는 **반증 가능한 가설 + 원인을 가르는 실험 설계**로 쓰면 오히려 맞는 형태가 된다.
- 마감까지 1주일이다. 1단계에 필요한 것은 완성된 실험이 아니라 **명확한 가설, 형식화된 문제, 방법론, 예비 결과**이다. 아래 틀은 5+1쪽을 기준으로 짰다.
- 2단계 최종본(8쪽)은 1단계 틀을 확장하는 방식으로 쓴다.

---

## 1. 읽은 RSS 메인트랙 논문 22편

- **선정 방법:** 공식 논문집([RSS 2024](https://roboticsproceedings.org/rss20/), [RSS 2025](https://roboticsproceedings.org/rss21/), [RSS 2026](https://roboticsproceedings.org/rss22/))의 전체 목록 507편(134/163/210)에서 제목 키워드로 1차로 걸렀다. 키워드는 다중 로봇, 배정·스케줄링, MAPF, 물류, 불확실성하 계획, 확인/정보 수집, 시뮬레이션 평가였다. 그중 우리 문제와 가까운 22편의 PDF를 읽었다.
- **원문 출처:** RSS 2025 논문은 논문집 PDF에서 글자를 뽑을 수 없어 같은 논문의 arXiv 판을 읽었다.
- **"인기" 기준이 아님:** 인용 수 같은 인기 지표는 이 환경에서 확인할 수 없어 쓰지 않았다. 고른 기준은 주제 근접성이다.
- **눈에 띈 점:** 3년간 RSS 메인트랙에 **창고 작업 배정 + 재고 기록 불일치를 직접 다룬 논문은 없었다**. 가장 가까운 것은 물류 스케줄링(RSS 2026 p039), 도착해야 드러나는 막힘을 다루는 경로 계획(RSS 2026 p042), 가설 검증을 계획에 넣은 연구(RSS 2026 p181)이다.

| 연도·번호 | 제목 | 우리와의 관련 | 섹션 구성 (원문 목차) | 평가 규모·형태 |
|---|---|---|---|---|
| 2024 p005 | Decentralized Multi-Robot Line-of-Sight Connectivity Maintenance under Uncertainty | 위치 불확실성하 다중 로봇 | Prelim(문제 정의) → Method → 이론 → Results(시뮬, CoppeliaSim, 실물) → 증명 부록 | 20 trials, 실물 있음 |
| 2024 p029 | HumanTHOR: Simulation Platform and Benchmark for Human-Robot Collaboration | 사람-로봇 공유 작업공간 시뮬 플랫폼 | RW → System → Benchmark → Experiments(Baselines/Setup/Results/Case study) | VR 사용자 연구 |
| 2024 p087 | Experience-based multi-agent path finding with narrow corridors | MAPF(창고형 환경 언급) | RW → Method(문제 정의) → Database → Simulated Results(기존법 비교, ablation) | 20 random trials, 시뮬만 |
| 2024 p115 | iHERO: Interactive Human-oriented Exploration and Supervision Under Scarce Communication | 다중 로봇 + 사람 요청 | Intro 안에 RW → Problem Description → Solution → Numerical Exp.(비교, 하드웨어) | 하드웨어 실험 |
| 2024 p116 | Who Plays First? Optimizing the Order of Play in Stackelberg Games with Many Robots | 다중 로봇 순서 결정(배송 차량 군) | RW → Problem Formulation → 알고리즘 → Experiments(3개 도메인+하드웨어) → **Limitations** | 20 trials, 하드웨어 |
| 2024 p118 | Partially Observable Task and Motion Planning with Uncertainty and Risk Awareness (TAMPURA) | 부분 관측 + 정보 수집 | RW → Background(belief-state MDP) → 방법 → Sim Exp.(도메인, 베이스라인) → Real-world | 실물 있음 |
| 2025 p027 | Solving Multi-Agent Safe Optimal Control with Distributed Epigraph Form MARL | 다중 로봇 학습 | Problem Setting → 방법 → Sim Exp.(**Q1–Q3**, ablation) → Hardware → **Limitations** | 2개 시뮬레이터, Crazyflie |
| 2025 p043 | Bilevel Learning for Bilevel Planning | 학습 + 계획 | Problem Formulation → 방법 → Experiments → **RW를 실험 뒤에** → Limitations | 3 random seeds, 실물 |
| 2025 p087 | Optimal Interactive Learning on the Job via Facility Location Planning | 작업 배정을 시설 입지 문제로 형식화 | RW → Problem Statement → Approach → Exp. Setup → Results → 통계 분석 부록 | 10 seeds, 실물 |
| 2025 p098 | APEX-MR: Multi-Robot Asynchronous Planning and Execution | 다중 로봇 실행 불확실성 | RW → Prelim → 방법 → Results(**Q1–Q3**: 시뮬, 시스템 배치) → **Limitations** | 4 random seeds, 실물 |
| 2025 p099 | Hierarchical Temporal Logic TAMP for Multi-Robot Systems | 다중 로봇 작업 배정 포함 | RW → Prelim → Problem Formulation → Approach → 이론 → Exp.(확장성, 로봇) → **Limitations** | 실물 |
| 2025 p101 | How to Coordinate UAVs and UGVs ... Cooperative Routing with a DRL Framework | **학습 기반 다중 에이전트 배정·경로** | Intro 안에 RW → Problem Formulation(**MDP**) → 휴리스틱 → RL → Results(데이터, 비교, **일반화**, 사례, **Discussion & limitations**) | **100 test instances, mean±std, 시뮬만** |
| 2026 p033 | Adapting Execution-Time Objectives for Multi-Robot Policies via Collaborative Flow Policy Guidance | 다중 로봇 정책 학습 | Prelim → Problem → Approach → Experiments → **RW를 실험 뒤에** | 5 seeds, 실물 |
| 2026 p035 | Teaming Linear Temporal Logic | 다중 에이전트 명세 | Background → 방법 → Demonstrations | 시연 |
| 2026 p036 | A Closed-Loop Multi-Agent Framework for Robust Multi-Robot Manipulation | 배정 + 실행 검증 | RW → Method(배정, 검증) → Exp.(외란, ablation, 오류 분해) | 20 trials, 실물 |
| 2026 p039 | Event-Driven Sleep-Wake Scheduling for Heterogeneous Robots under LTL Constraints (ALIS-WC) | **물류 다중 로봇 스케줄링, 이벤트 기반 RL** | RW → Methodology(문제 정의, 이벤트 기반 MDP) → Exp.(**Q1–Q4**, 시나리오 범위 표, 베이스라인, ablation) | **3 seeds mean±std, 이산사건 시뮬만** |
| 2026 p042 | Optimal UGV–UAV Cooperative Partitioning and Inspection of Shortest Paths | **도착해야 드러나는 막힘 + 확인(inspection) 분담** | RW → Problem Formulation → 최악 경우 분석 → 최적 알고리즘 → Results(데이터, 전략, **Limitations**) | **실제 50개 도시 도로망**, 막힘 위치 무작위, 시뮬만 |
| 2026 p043 | Time-Aggregated Connectivity Maintenance for Multi-Robot Networks | 다중 로봇 | Prelim(문제 정의) → Method → Results | 시뮬 |
| 2026 p090 | Betting for Sim-to-Real Performance Evaluation | **싼 시뮬레이터로 비싼 평가를 보완** | Intro(문제, 문헌) → 방법 → Case studies → Limitations | 실제 집기 실험 포함 |
| 2026 p179 | Vec-QMDP: Vectorized POMDP Planning on CPUs | POMDP | RW → 형식화 → 방법 → Exp.(지표, 비교, ablation, 확장성) | 벤치마크 |
| 2026 p181 | Hypothesis-driven Model Expansion under Uncertainty for Open-World Robot Planning (HUME) | **가설 검증을 계획에 통합 ≈ 우리 VERIFY** | RW → Problem Formulation → 방법(가정 명시) → Exp.(**RQ1–RQ2**, 시뮬, 실물) → Discussion | 실물 |
| 2026 p183 | Informative Path Planning with Guaranteed Estimation Uncertainty | 불확실성 줄이는 경로 | RW → Prelim → Problem → 방법 → Exp.(벤치마크, 현장 시험) → **Limitations** | 현장 시험 |

### 1.1 22편에서 공통으로 보인 흐름 [원문]

**초록.** 네 단계로 쓴다: 응용의 필요 → 기존 연구가 둔 **구체적 가정**과 그 한계 → 제안과 핵심 아이디어 → 정량 결과와 범위.

- p005: "In contrast to most existing work that assumes perfect positional information..."
- p118: "the typical TAMP problem formulation assumes full observability and deterministic action effects."
- 둘 다 **기존 가정을 한 문장으로 지목**한 뒤 자기 문제로 넘어간다.

**서론.**

1. 응용 맥락을 인용과 함께 제시한다(`[1–3]` 식).
2. 이 문제의 구체적 어려움을 말한다.
3. **가장 가까운 고전 문제와 그 가정**을 짚는다.
   - p042: 확률형 Canadian Traveller Problem은 막힘 확률을 안다고 가정하지만, 재난 현장에서는 그 확률을 믿을 수 없다. 그리고 원래 CTP는 막힘이 꼭짓점에 닿는 순간 드러난다고 가정한다.
   - p101: VRP/MILP가 규모가 커지면 풀 수 없다.
4. 질문 한 문장을 던진다. p039: "This raises a critical question: ..."
5. 접근법을 소개하고, 번호 붙인 기여 3개를 쓴다. 마지막 기여는 대개 평가이다.
6. 그림 1은 1쪽에 둔다(p101, p039, p098, 2025 p087).

**관련 연구.**

- 대부분 2절에 둔다.
- 서론 안에 짧게 넣은 논문(p101, p115)과 실험 뒤로 뺀 논문(2025 p043, 2026 p033)도 있다.
- **RSS 2027 1단계는 직접 관련 문헌만 요구하므로** 서론 안이나 짧은 2절이 맞다.

**문제 정의.** 형식 모델 + 명시적 가정 + "Problem 1" 형태의 문제 진술로 쓴다.

- MDP: p101 / 이벤트 기반 MDP: p039 / CMDP: p027 / belief-state MDP: p118 / 그래프: p042
- p181은 "Necessary Assumptions"라는 소절을 따로 둔다.

**실험.** "다음 질문에 답하도록 설계했다"로 시작하는 Q1–Qn 구조가 많다(p027, p098, p039, p181). 구성 요소는 다음과 같다.

- **시나리오 생성 파라미터 범위를 표로** 준다. p039 Table I: 로봇 수, 작업 수, 시간 제한 범위.
- **베이스라인을 성격별로 묶고**, 우리 목적함수에 맞게 어떻게 고쳤는지 쓴다. p039는 offline(GA, AVNR)과 online(Greedy, 제안)으로 묶었다.
- **시뮬레이터 파라미터는 문헌에서 가져온다.** p101: "fuel consumption follows a profile modeled after [40]".
- **일반화 실험을 따로 둔다.** p101은 더 큰 문제, 다른 팀 구성, 다른 작업 분포를 재학습 없이 시험했다.
- 제거 실험(ablation)을 하고, **Limitations를 별도 절**로 둔다(p027, p098, p099, p116, p183, p042 등).

**통계.**

- p101: 100개 시험 인스턴스의 mean ± std
- p039: 학습 시드 3개의 mean ± std
- p087(2024), p116, p036: 20 trials
- 2025 p087: 10 seeds

**실물 실험.**

- 22편 중 14편(p005, p115, p116, p118, 2025 p027·p043·p087·p098·p099, 2026 p033·p036·p090·p181·p183)에 실물·하드웨어 실험이 있다.
- **우리와 가장 비슷한 스케줄링·배정 논문(p101, p039, p042)은 시뮬레이션만 썼다.** 대신 세 가지로 보완했다.
  1. 대규모 시험 인스턴스: 100개, 실제 50개 도시
  2. 학습 분포 밖 일반화 시험
  3. 이론 분석(p042의 경쟁비)

---

## 2. 우리 논문의 새 틀 (1단계 5+1쪽 기준)

### 2.0 핵심 주장(가설) 정하기

지금 데이터가 말하는 것은 다음과 같다. 출처는 `template/main.tex`의 과적합 표와 `main.tex`의 Webots 표다.

- 학습한 날짜를 시드만 바꿔 시험하면 팀 기본 WES 대비 주문 리드타임이 −39초(경량), −57초(Webots) 줄었다.
- 처음 보는 6일에서는 +1초(2일 학습), −1초(3일 학습)로 **개선이 사라졌다**. Webots 08-22에서도 +4초, +17초였다.
- Webots 오류 조건에서는 **오류 없이 학습한 가중치가 오류 조건에서 학습한 가중치보다 나았다**(−113초 대 −57초).

그래서 논문을 "학습이 이긴다"가 아니라 **가설과 그 반증 실험**으로 짠다. RSS 2027 FAQ 2번이 요구하는 형태다: 실패 원인 후보를 세우고, 그것을 구별하는 실험을 둔다.

> **주 가설 H1.** 재고 기록 불일치가 있는 창고에서, 고속 시뮬레이터로 학습한 작업-로봇 점수 정책은 사람이 설계한 배정 규칙과 거리 최적 매칭보다 **처음 보는 날짜의 주문**에서 주문 리드타임을 줄인다.
>
> 예비 결과: 2–3일 학습에서는 지지되지 않음(새 날짜에서 개선 ≈ 0).

**원인 후보와 구별 실험** (각각 독립 가설로 쓴다)

| 가설 | 내용 | 지지되면 보일 것 | 구별 실험 |
|---|---|---|---|
| H2 (날짜 다양성 부족) | 학습 날짜의 주문 패턴을 외웠다 | 학습 날짜 수를 늘릴수록 새 날짜 개선이 커지고, 일반화 차이가 줄어든다 | 학습 날짜 수 {2, 6, 18, 48, 96} × 날짜가 겹치지 않는 시험 세트. 계산 예산은 같게 맞춘다. 근거: Cobbe 등(2019)의 학습 레벨 수 대비 일반화 실험 설계 |
| H3 (특징 표현력 부족) | 선형 점수에 주문 단위 정보(주문 크기, 남은 줄 수)가 없다 | 날짜를 늘려도 개선이 없고, 주문 특징을 넣으면 생긴다 | 같은 예산에서 특징 집합 {현재, + 주문 특징}을 비교한다 |
| H4 (불일치와 무관) | 학습의 이득은 기록 불일치 대응이 아니라 일반적인 작업 순서에서 나온다 | 학습 조건(오류 유무)이 결과를 바꾸지 않고, 진실을 아는 오라클과의 차이도 학습으로 줄지 않는다 | 학습{오류, 무오류} × 시험{오류율 0, 0.5, 1, 2, 4배}. 실제 재고를 아는 오라클 WES를 상한으로 둔다 |
| H5 (시뮬레이터 차이) | 경량 시뮬레이터의 순위가 물리 시뮬레이터와 다르다 | 경량과 Webots의 방법 순위·쌍대 차이가 어긋난다 | 처음 보는 날짜의 일부를 Webots에서 반복 실행하고, 순위 일치와 차이의 신뢰구간을 비교한다. 근거: RSS 2026 p090(싼 시뮬레이터로 평가를 보완하는 문제) |

- 이 다섯 가설은 결과가 어느 쪽으로 나와도 **결론이 나온다**. 심사 항목 "Feasibility of Conclusion"이 보는 것이 바로 이것이다.
- 2단계 Extension Charter에서 심사위원이 실험을 더 요구하더라도, 이 구조 위에 얹으면 된다.

### 2.1 Abstract (≈150단어, 마지막에 쓴다)

1단계는 예비 결과 단계이므로 숫자는 예비 결과임을 밝힌다.

1. **필요:** 창고 실행 시스템은 입고 적치, 출고 피킹, 재고 확인을 다중 로봇에 배정한다. 근거: Wurman 등 2008 [서지]
2. **기존 가정과 한계:** 배정 연구는 재고 기록이 맞다고 가정하지만, 실제 기록은 자주 틀린다. 근거: DeHoratius & Raman 2008 [서지]
3. **제안:** 고속 시뮬레이터에서 학습하는 선형 점수 정책과, 처음 보는 주문 날짜로 평가하는 프로토콜
4. **가설:** H1, 그리고 H2–H5로 원인을 가른다
5. **예비 결과:** 학습 날짜 −7%, 새 날짜 ≈ 0
6. **얻을 통찰:** 어느 결과가 나와도 남는 교훈

### 2.2 Introduction (≈1쪽)

| 문단 | 할 말 | 근거 |
|---|---|---|
| P1 맥락 | 창고 로봇 대수와 배정의 중요성. 피킹은 창고 운영비의 최대 55%로 추정된다 | Wurman 등 2008 [서지]; de Koster 등 2007, 초록의 "as much as 55%" [서지] |
| P2 문제 | 사람과 로봇이 같은 창고에서 일한다. 사람의 기록 없는 이동·반출로 기록이 틀린다 | 사람-로봇 협업 창고: Löffler 등 2023 [서지], Azadeh 등 2019 [서지]. 기록 불일치: 37개 매장 약 37만 기록 중 65%가 부정확(DeHoratius & Raman 2008) [서지]. 소비자가 찾은 품목의 16%가 재고는 있으나 잘못 놓여 못 찾음(Raman 등 2001) [서지]. 감지되지 않는 재고 손실이 작아도 기록이 크게 틀어짐(Kang & Gershwin 2005) [서지] |
| P3 가까운 고전 문제와 가정 | ① 다중 로봇 배정: 작업 결과 불확실성을 **확률 모델로 안다**고 가정(Choudhury 등 RSS 2020; Dhanaraj 등 ICRA 2024). ② 재고 불일치 연구: 검사·보충을 다루지만 로봇 이동과 경합은 없음(Kök & Shang 2007; Akkerman 등 2025). ③ 창고 MAPD: 작업 배정과 경로를 함께 풀지만 기록은 정확하다고 가정(Ma 등 2017) | ①② `related_work_matrix.md`에서 원문 확인. ③ [서지] |
| P4 질문 | "재고 기록이 틀리는 창고에서, 학습한 배정 정책은 처음 보는 날짜에도 일반화되는가?" | p039 문장 형식 [원문] |
| P5 접근 | 같은 WMS/WES 코드를 쓰는 고속 시뮬레이터 + Webots 검증, 실제 주문 데이터 178일, **날짜 단위 학습/시험 분할** | 데이터셋: de Assis, de Paula Ferreira, Ouhimmou [서지] |
| 기여 3개 | ① 날짜 분할 평가 프로토콜과 예비 증거(시드 분할은 개선을 과대평가) ② H1–H5와 그 판별 실험 설계 ③ 공개 시뮬레이터와 시나리오 생성기 | |
| 그림 1 | 왼쪽 Webots 창고, 가운데 WMS–WES–로봇 구조, 오른쪽 "시드 분할 vs 날짜 분할" 막대 | p101, p039처럼 1쪽에 둔다 |

**지금 초안에서 반드시 빼거나 고칠 문장**

- "이런 규칙은 기록이 정확하다는 가정에서 조정되었기 때문에, 확인 작업과 재작업이 섞이는 상황에서 어떤 작업을 먼저 할지를 잘 고르지 못한다." → 근거 문헌이 없다. 게다가 우리 Webots 결과(무오류 학습 가중치가 더 나음)와 맞지 않는다. 이 문장은 **H4로 바꿔 검증 대상으로** 둔다.

### 2.3 Related Work (≈0.4쪽, 1단계는 직접 관련만)

세 갈래로 쓰고, 각 갈래의 끝에 "우리와의 차이" 한 문장을 둔다.

1. **불확실성하 다중 로봇 배정**
   - Gerkey & Matarić 2004: 분류 체계
   - Choudhury 등 RSS 2020; Dhanaraj 등 ICRA 2024: 실패 확률을 알고 계획
   - 차이: 우리는 불일치의 위치와 시각을 모르고, WMS 이벤트만 본다
2. **창고 작업 배정과 경로**
   - Ma 등 AAMAS 2017(MAPD): 온라인 작업 배정
   - Li 등 AAAI 2021: 대규모 창고 lifelong MAPF
   - 차이: 우리는 경로 계획이 아니라 배정 문제이고, 기록 불일치가 있다
3. **배정·디스패칭 규칙 학습과 일반화**
   - 디스패칭 규칙 자동 설계: Branke 등 2016 [서지]
   - 우선순위 디스패칭 규칙 학습: Zhang 등 NeurIPS 2020 [서지]
   - 선형 정책 + CEM: Szita & Lőrincz 2006 [서지]
   - 학습 환경 수와 일반화: Cobbe 등 ICML 2019 [서지]
   - 차이: 우리는 주문 날짜를 단위로 일반화를 잰다

RSS 2026 p042(도착해야 드러나는 막힘)와 p181(가설 검증을 계획에 통합)은 "관측해야 드러나는 불일치"라는 우리 설정의 가장 가까운 로봇 분야 선례이다. 2단계 포괄 관련 연구에 넣는다 **[원문]**.

### 2.4 Problem (≈0.8쪽)

p039의 "이벤트 기반 MDP", p118의 "belief-state MDP" 형식을 따른다.

- **세계:**
  - 랙 칸 집합, 로봇 집합, 작업 종류 {PUTAWAY, PICK, RELOCATE, VERIFY}
  - 실제 재고 상태 `x_t`와 WMS 기록 `x̂_t`
- **관측:** WES는 `x̂_t`와 로봇 이벤트(SHORT_PICK, ITEM_MISMATCH, DESTINATION_OCCUPIED, OBSERVATION_RESULT)만 본다. 불일치는 로봇이 칸에 도착해 라벨을 읽을 때만 드러난다.
  - 이 구조는 p042의 "blockages discovered only when a robot arrives"와 같은 형태라서 그 문헌을 인용해 정당화할 수 있다 **[원문]**.
- **결정:** 매 결정 시점에 시작 가능한 작업과 로봇을 짝짓는다.
- **목적:** 주문 리드타임 평균. 미완료 주문은 에피소드 끝 시각으로 절단한다.
- **불일치 과정:** S1 이동, S2 반출, S3 목적지 점유를 마크된 포아송 과정으로 정의한다(`research_protocol.md` §1과 일치).
- **가정 소절 (p181처럼 "Assumptions"를 따로 둔다):**
  - A1 로봇은 오류를 내지 않는다
  - A2 통신은 즉시 전달된다
  - A3 불일치 발생률은 합성 가정이다 → 민감도 실험으로 다룬다
- **Problem 1:** 위 조건에서 처음 보는 날짜의 주문 분포에 대한 기대 리드타임을 최소화하는 정책을 찾는다.

### 2.5 Method (≈0.8쪽; 사용자 틀에는 없지만 RSS 22편 모두에 방법 절이 있다)

- **점수 정책:** `S(τ, r) = wᵀφ(τ, r)`, 탐욕 매칭, 특징 표.
  - 선형 정책 + CEM의 선례: Szita & Lőrincz 2006 [서지]
  - 학습한 우선순위 디스패칭 규칙의 선례: Zhang 등 2020 [서지]
- **고속 시뮬레이터:** 같은 시나리오·WMS·WES 코드를 쓰고 이동만 단순화했다. 이전 입력의 51회 보정 수치를 현재 v3 입력의 근거로 인용하지 않는다. v3 개발 점검에서 주 지표 정책 순위가 뒤집혔으므로 검증 날짜 Webots에서 정책을 선택하고 보류 시험 날짜 Webots에서 최종 평가한다.
- **학습:** CEM 설정과 비용 함수.
- **평가 프로토콜:**
  - 학습 날짜 / 검증 날짜 / 시험 날짜를 서로 겹치지 않게 나눈다.
  - 평가 단위는 날짜다.
  - 날짜별 쌍대 차이에 부트스트랩 95% CI를 붙인다. 근거: Agarwal 등 2021(`research_protocol.md`).
  - 학습 시드 수와 결과 분산을 보고한다. Henderson 등 2018은 학습 시드에 따라 결과가 다른 분포처럼 보일 수 있다고 지적했다 [서지].

### 2.6 Experiment (≈1.6쪽; 1단계는 계획 + 예비 결과)

p039·p027처럼 "다음 질문에 답하도록 설계했다"로 시작해 H2–H5를 Q로 둔다.

**설정 표** (p039 Table I 형식)

- 날짜 수, 시드, 로봇 수(8 / 14대), 창고(160 / 480칸), 오류율, 에피소드 길이

**베이스라인** (성격별로 묶는다)

| 묶음 | 방법 |
|---|---|
| 규칙 | 팀 기본 WES, 가까운 로봇 + 피킹 우선 |
| 최적화 | 헝가리안 매칭(Kuhn 1955) |
| 상한 | **실제 재고를 아는 오라클**(새로 추가) |
| 학습 | CEM |

**예비 결과** (이미 있는 것)

- 시드 분할 vs 날짜 분할 표(`template/main.tex` 과적합 표)
- Webots 18개 시나리오

**계획한 결과**

| Q | 그림·표 |
|---|---|
| Q1 (H2) | 학습 날짜 수 대비 새 날짜 개선 곡선 |
| Q2 (H3) | 특징 집합별 비교 |
| Q3 (H4) | 오류율 민감도 + 오라클 대비 격차 |
| Q4 (H5) | 경량 vs Webots 산점도와 순위 일치 |

### 2.7 Conclusion (≈0.3쪽)

- 가설별로 지지 / 반증 / 보류를 정리한다.
- 결과와 상관없이 남는 통찰을 쓴다.
- 한계를 쓴다: 실물 없음, 오류율 합성, 창고 한 종류.
- 2단계에서 할 일을 쓴다.

### 2.8 분량 배분 (본문 5쪽)

| 섹션 | 쪽 |
|---|---|
| Abstract + Introduction (그림 1 포함) | 1.2 |
| Related Work | 0.4 |
| Problem | 0.8 |
| Method | 0.8 |
| Experiment (계획 + 예비 결과) | 1.5 |
| Conclusion / Limitations | 0.3 |

---

## 3. 설정별 근거 대조표

| 우리 설정 / 주장 | 근거 | 상태 | 조치 |
|---|---|---|---|
| 재고 기록이 실제와 자주 다르다 | DeHoratius & Raman, *Management Science* 2008: 37개 매장 369,567 기록 중 65% 부정확 | [서지] 소매 매장 자료 | "소매에서 보고된" 수치라고 범위를 밝힌다. 창고(DC) 수치는 찾지 못했다 |
| 오적치(S1)가 실재한다 | Raman, DeHoratius, Ton, *California Mgmt Review* 2001: 16% 품목을 잘못 놓여 못 찾음. Camdereli & Swaminathan, *POM* 2010: 오적치 재고 모델 | [서지] | 발생 **빈도** 근거는 아님 |
| 감지되지 않는 반출(S2)이 기록을 틀어지게 한다 | Kang & Gershwin, *IIE Transactions* 37(9) 2005 | [서지] | 같음 |
| 목적지가 이미 차 있음(S3) | 찾지 못함. 검색에서는 업체 블로그의 출처 없는 수치뿐이었다 | **[근거 없음]** | 가정으로 밝히거나, S3를 끈 조건을 함께 보고한다 |
| 오류율: S1 8회/h, S2 3회/h, S3 0.1 | 없음 | **[근거 없음]** | 민감도 실험(0, 0.5, 1, 2, 4배)으로 바꾼다. p042는 막힘 위치를 무작위화했고, p101은 분포를 바꿔 시험했다 [원문] |
| 사람과 로봇이 같은 창고에서 일한다 | Löffler, Boysen, Schneider, *Transportation Science* 57(4) 2023; Azadeh, de Koster, Roy, *Transportation Science* 53(4) 2019 | [서지] | |
| 로봇이 라벨을 읽어 재고를 확인할 수 있다 | Morenza-Cinos 등: RFID 재고 로봇이 오적치를 찾는다(2017 장). 2019 IJPDLM 논문은 수작업보다 정확도·소요시간이 낫다고 보고 | [서지] | 우리는 카메라 라벨이라 "유사 사례"로만 쓴다 |
| 로봇 속도 0.8 m/s | Kiva 1.3 m/s는 위키백과 2차 출처뿐. Löffler 등 2023은 "AMR과 피커 속도가 비슷해야 한다"는 결론 | **[근거 없음]** | 상용 AMR 제품 사양서를 인용하거나, 속도 민감도(0.5 / 0.8 / 1.2)를 넣는다 |
| 피킹이 창고 비용의 큰 부분이다 | de Koster, Le-Duc, Roodbergen, *EJOR* 182(2) 2007: "as much as 55%" | [서지] | |
| 실제 주문 데이터 | de Assis 등, Mendeley Data 2024 (DOI 10.17632/pf2w725pw3.1); 관련 *Data in Brief* 2025 | [서지] | 현재 `main.tex`의 연도·저자 `\todo`를 이것으로 고친다. Data in Brief 서지는 ScienceDirect에서 다시 확인 |
| 선형 정책 + CEM | Szita & Lőrincz, *Neural Computation* 18(12) 2006; de Boer 등 2005(`research_protocol.md`) | [서지] | "새 학습 알고리즘"이라고 주장하지 않는다 |
| 사람이 만든 디스패칭 규칙을 학습으로 대체 | Branke 등, *IEEE TEVC* 20(1) 2016; Zhang 등, NeurIPS 2020 | [서지] | |
| 학습 환경이 적으면 과적합한다 | Cobbe 등, ICML 2019: 학습 레벨 4,000개 미만에서 큰 과적합 | [서지] | H2의 실험 설계 근거로 쓴다 |
| 시드 간 분산 보고 | Henderson 등, AAAI 2018; Agarwal 등, NeurIPS 2021 | [서지] | |
| 싼 시뮬레이터로 평가를 보완하는 문제 | Chen, Mahboob, Weng, RSS 2026 p090 "Betting for Sim-to-Real Performance Evaluation" | [원문] | H5의 근거로 쓴다 |
| 도착해야 드러나는 불확실성 | RSS 2026 p042 (CTP 변형) | [원문] | 문제 정의의 정당화에 쓴다 |
| 주문 웨이브 90초, 출고 마감 15분 | 없음. 데이터의 시간 압축에서 나온 우리 설계 | **[근거 없음]** | 설계 선택으로 밝히고 이유를 쓴다 |

**참고문헌 주의:** RSS 2027은 존재하지 않는 참고문헌을 자동 검사해 데스크 리젝트한다. 위 [서지] 항목은 제목·저자·권호를 색인에서 확인했지만, 본문에 수치를 쓸 때는 원문 쪽수까지 확인해야 한다. 지금 `main.tex`의 `assis2023`은 연도가 틀렸다(Mendeley 2024).

---

## 4. 지금 실험의 부족한 점과 재실험 목록

**우선순위**

| 등급 | 뜻 |
|---|---|
| A | 12/4 1단계 전에 해야 함 (경량 시뮬로 며칠 안에 가능) |
| B | 1단계에서 "계획"으로 쓰고, 2단계(~04-16)에서 실행 |

| # | 부족한 점 (레포에서 확인한 사실) | 왜 문제인가 | 다시 할 실험 | 등급 |
|---|---|---|---|---|
| 1 | 학습 18일 + 시험 6일 = 24일만 쓴다. **원본에는 178일이 있다** (`Customer_Order.csv`의 creationDate 기준 집계) | "날짜가 제한적"이라는 한계는 스스로 만든 것이다. 새 날짜 결론의 근거가 6일뿐이다 | 날짜를 학습/검증/시험으로 나누고 시험 날짜를 30일 이상 확보한다. 학습 날짜 수 곡선 {2, 6, 18, 48, 96}을 그린다 (H2) | **A** |
| 2 | 18일 학습 결과가 비어 있다(`\todo`). 학습 시드도 r0, r1 두 개뿐 | 주 가설 H1의 핵심 칸이다 | r0–r4 다섯 시드로 학습하고, 새 날짜 30일 이상에서 평가한다 | **A** |
| 3 | 평가 단위가 섞여 있다(시나리오 × 반복 5회 = 45 에피소드를 평균) | 같은 날짜의 반복은 독립 표본이 아니다 | 날짜 단위 쌍대 차이 + 부트스트랩 CI(날짜를 재표집). `research_protocol.md` §3의 원칙과 같다 | **A** |
| 4 | 오류율이 한 가지(기본값)뿐이고 근거가 없다 | 주장의 범위를 정할 수 없다. H4를 판별하지 못한다 | 오류율 배율 {0, 0.5, 1, 2, 4} × 학습 조건 {오류, 무오류} | **A**(경량), B(Webots) |
| 5 | 상한 비교가 없다 | 불일치 때문에 잃는 몫과 학습이 되찾는 몫을 나눌 수 없다 | 실제 재고를 아는 오라클 WES(같은 배정 규칙 + 진실 접근)를 상한으로 둔다 | **A** |
| 6 | 특징에 주문 단위 정보가 없다 | H3(표현력 부족)을 배제할 수 없다 | 주문 크기, 남은 줄 수, 주문 나이 특징을 추가하고 같은 예산으로 비교한다 | B |
| 7 | 경량 시뮬 보정이 Webots 비켜서기 동작을 넣기 **전**이다(`lite_sim.md`). 로봇 4대에서는 방법 간 차이를 절반으로 본다 | 경량 결과를 Webots에 옮기는 근거(H5)가 낡았다. 헝가리안이 경량 −23초, Webots −66초로 어긋난 것과도 연결된다 | 현재 코드로 `validate_lite.py`를 다시 돌린다. 새 날짜 일부를 Webots에서 시나리오당 3회 이상 반복한다 | A(보정), B(Webots) |
| 8 | Webots는 시나리오당 1회, 03-22 전체는 주문 13개뿐 | 주문 하나가 평균을 수백 초 움직인다 | 반복 횟수를 늘린다. 주문 수가 적은 날은 상자 단위 리드타임을 함께 본다 | B |
| 9 | 베이스라인에 창고 배정 문헌의 표준 방법이 없다 | RSS 독자는 MAPD(Ma 등 2017)를 안다 | MAPD 식 온라인 배정(가장 가까운 작업 / 작업 재배정 허용) 변형을 하나 추가한다. 경로 계획은 빼고 배정 부분만 비교한다고 범위를 명시한다 | B |
| 10 | 로봇 속도 0.8 m/s, S3, 웨이브 90초, 마감 15분에 근거가 없다 | 설정 정당화가 약하다 | 속도 민감도를 넣고, S3를 끈 조건을 넣고, 나머지는 설계 선택으로 명시한다 | B |
| 11 | 실물 실험이 없다 | 22편 중 14편에 실물이 있다. 다만 가장 가까운 스케줄링 논문 3편(p101, p039, p042)은 시뮬만 썼다 | 실물은 범위 밖으로 둔다. 대신 그 3편처럼 많은 시험 날짜, 분포 밖 일반화(큰 창고 14대, 피크일), 물리 시뮬레이터 검증으로 보완한다 | B |
| 12 | `main.tex`와 `template/main.tex` 두 초안이 다른 이야기를 한다 | 혼란을 낳는다 | `template/main.tex`(IEEEtran, RSS 템플릿) 하나로 합치고 이 틀로 다시 쓴다 | **A** |

**1주일 실행 순서 (제안)**

| 일차 | 할 일 |
|---|---|
| 1–2일 | 178일 전처리. 날짜 분할(학습 96 / 검증 20 / 시험 30 이상)을 고정하고 커밋한다 |
| 2–4일 | 경량 시뮬로 표의 #1 #2 #3 #5 + #4 일부를 실행한다. 1시간 에피소드가 약 1초이므로 가능한 규모다 |
| 4–5일 | `validate_lite.py`로 재보정을 확인한다 |
| 5–7일 | 5+1쪽 초안을 쓴다. 가설·방법을 완성하고, 결과는 예비로 둔다 |

---

## 5. 출처

### RSS 공식

- [RSS 2027 CFP](https://roboticsconference.org/information/cfp/)
- [Reviewer Guidelines](https://roboticsconference.org/information/reviewer-guidelines/)
- [Author FAQ](https://roboticsconference.org/information/author-faq/)
- 논문집 목록: [RSS 2024](https://roboticsproceedings.org/rss20/), [RSS 2025](https://roboticsproceedings.org/rss21/), [RSS 2026](https://roboticsproceedings.org/rss22/)
- 1절 표의 각 논문은 `https://roboticsproceedings.org/rss<20|21|22>/p<번호>.pdf`에서 읽었다.
- RSS 2025 6편은 arXiv 판으로 읽었다: [2503.15836](https://arxiv.org/abs/2503.15836), [2504.18899](https://arxiv.org/abs/2504.18899), [2504.21111](https://arxiv.org/abs/2504.21111), [2505.00490](https://arxiv.org/abs/2505.00490), [2502.08697](https://arxiv.org/abs/2502.08697), [2504.15425](https://arxiv.org/abs/2504.15425)

### 설정 근거 (서지·초록 확인)

- DeHoratius & Raman 2008, *Management Science* 54(4):627–641 — [RePEc](https://econpapers.repec.org/RePEc:inm:ormnsc:v:54:y:2008:i:4:p:627-641)
- Raman, DeHoratius, Ton 2001, *California Management Review* 43(3) — [CMR](https://cmr.berkeley.edu/2001/05/43-3-execution-the-missing-link-in-retail-operations/), [Chicago Booth 요약](https://www.chicagobooth.edu/review/first-measure-then-manage)
- Camdereli & Swaminathan 2010, *Production and Operations Management* — DOI 10.1111/j.1937-5956.2009.01057.x
- Kang & Gershwin 2005, *IIE Transactions* 37(9):843–860 — [EconBiz](https://www.econbiz.de/Record/information-inaccuracy-in-inventory-systems-stock-loss-and-stockout-kang-yun/10006160757)
- de Koster, Le-Duc, Roodbergen 2007, *EJOR* 182(2):481–501 — [EUR repository](https://repub.eur.nl/pub/11877)
- Azadeh, de Koster, Roy 2019, *Transportation Science* 53(4):917–945 — DOI 10.1287/trsc.2018.0873
- Löffler, Boysen, Schneider 2023, *Transportation Science* 57(4):979–998 — DOI 10.1287/trsc.2023.1207
- Boysen, de Koster, Weidinger 2019, *EJOR* 277(2):396–411 — DOI 10.1016/j.ejor.2018.08.023
- Ma, Li, Kumar, Koenig 2017, AAMAS — [arXiv 1705.10868](https://arxiv.org/abs/1705.10868)
- Li 등 2021, AAAI — [AAAI](https://ojs.aaai.org/index.php/AAAI/article/view/17344)
- Wurman, D'Andrea, Mountz 2008, *AI Magazine* 29(1) — DOI 10.1609/aimag.v29i1.2082
- Morenza-Cinos 등 — [UPF 저장소](https://repositori.upf.edu/items/93edd6ae-c6fc-4c9f-8cc6-65f3ff4fa97a), [IJPDLM 2019](https://emerald.com/insight/content/doi/10.1108/IJPDLM-03-2018-0151/full/html)
- de Assis 등 데이터셋 — [Mendeley Data](https://data.mendeley.com/datasets/pf2w725pw3/1)
- Szita & Lőrincz 2006, *Neural Computation* 18(12):2936–2941 — [dblp](https://dblp.uni-trier.de/rec/journals/neco/SzitaL06.html)
- Branke, Nguyen, Pickardt, Zhang 2016, *IEEE TEVC* 20(1):110–124 — DOI 10.1109/TEVC.2015.2429314
- Zhang 등 2020, NeurIPS — [proceedings](https://proceedings.neurips.cc/paper/2020/hash/11958dfee29b6709f48a9ba0387a2431-Abstract.html)
- Cobbe 등 2019, ICML — [PMLR](https://proceedings.mlr.press/v97/cobbe19a.html)
- Henderson 등 2018, AAAI — [arXiv 1709.06560](https://arxiv.org/abs/1709.06560)

이미 팀이 원문으로 확인한 문헌(Choudhury RSS 2020, Dhanaraj ICRA 2024, Akkerman EJOR 2025, Kök & Shang 2007, Gerkey & Matarić 2004, Agarwal 2021 등)은 `related_work_matrix.md`와 `research_protocol.md`를 따른다.
