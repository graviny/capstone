# IROS 2027 연구 프로토콜 — 개발 단계 v1 (2026-11-20)

이 문서는 사전등록을 주장하지 않는다. 2026-11-20의 기존 30회 결과를 이미 본 뒤
작성했으며, 아래 실행은 **개발/진단 데이터**다. 확인적 최종 실험은 별도 동결한다.
목표는 유리한 결과가 아니라 반증 가능한 주장이다. 현 단계의 코드나 표만으로
IROS 수준의 신규성·실세계 타당성이 충족되었다고 주장하지 않는다.

## 근거와 적용 범위

| 근거 | 여기서 채택한 원칙 | 이 논문이 보장하지 않는 것 |
|---|---|---|
| Patterson et al., JMLR 2024, *Empirical Design in Reinforcement Learning*, §§2–5, https://www.jmlr.org/papers/v25/23-0183.html | 평가 단위·변동 원인·튜닝·실험자 편향을 명시하고 대조군과 가설을 분리 | 우리 실험에서 특정 seed 수나 임계값이 충분하다는 보장 |
| Agarwal et al., NeurIPS 2021, *Deep RL at the Edge of the Statistical Precipice*, https://proceedings.neurips.cc/paper_files/paper/2021/hash/f514cec81cb148559cf475e7426eed5e-Abstract.html | 점추정 외에 불확실성 보고. 에피소드 단위 paired bootstrap | n=10의 정밀성 보장, 모든 지표의 동시 유의성 |
| Glasserman & Yao, Management Science 38(6), 1992, *Some Guidelines and Guarantees for Common Random Numbers*, §2.2, https://business.columbia.edu/sites/default/files-efs/pubfiles/4261/glasserman_yao_guidelines.pdf | 대응 외생 난수 동기화. 발생/대상 선택 스트림 분리 | CRN이 모든 정책/지표에서 분산을 줄인다는 보장 |
| Gerkey & Matarić, IJRR 2004, *A Formal Analysis and Taxonomy of Task Allocation in Multi-Robot Systems*, https://ai.stanford.edu/~gerkey/research/final_papers/mrta-taxonomy.pdf | 배정을 명시적 비용의 할당 문제로 비교. 단일 시점 matching 대조군 | 창고 전체의 전역 최적성이나 EDF 최적성 |
| de Boer et al., Annals of OR 2005, *A Tutorial on the Cross-Entropy Method*, https://ris.utwente.nl/ws/portalfiles/portal/6700787/fulltext.pdf | 현재 학습을 CEM 정책 탐색으로 정확히 기술 | 독자적인 새로운 강화학습 알고리즘이라는 주장 |
| Sargent, Journal of Simulation 7, 2013, https://doi.org/10.1057/jos.2012.20 | 코드 검증과 사용 목적에 대한 모델 타당성 검증을 구분 | Webots와 일치하면 실제 창고까지 검증됐다는 주장 |
| Krishna et al., ICLR 2025, *The Value of Sensory Information to a Robot*, https://proceedings.iclr.cc/paper_files/paper/2025/hash/e1126028d9f1f69c13571ec462084d31-Abstract-Conference.html | 감지 비용과 작업 성능의 관계를 검증할 동기 | 앞서 제안한 P(error)*loss-cost 식이나 우리 위험 추정기의 정확성 |
| Li et al., AAAI 2021, *Lifelong Multi-Agent Path Finding in Large-Scale Warehouses*, https://ojs.aaai.org/index.php/AAAI/article/view/17344 | 혼잡·계속 도착하는 작업의 중요성과 경로계획 문제의 범위 구분 | RHCR가 현재 재고 불일치 WES의 동등한 직접 대조군이라는 주장 |

IROS 일정: IEEE RAS 공식 행사 페이지에 논문 마감 2027-03-01,
학회 2027-09-27–10-01, Florence가 기재되어 있다.
https://www.ieee-ras.org/event/2027-ieee-rsj-international-conference-on-intelligent-robots-and-systems-iros-70525/
학회 홈페이지 본문 접근이 실패했으므로 시각/시간대·페이지 제한·심사 형식은 아직 확정하지 않는다.

## 0. 논문의 질문과 주장 범위

질문: 재고 기록과 실제 상태가 다를 때, 제한된 로봇 자원으로 확인과 운반을
배정하는 정책은 서비스와 기록 정확도의 상충을 어떻게 개선할 수 있는가?
현재 CEM은 확인 위치 생성/오류 복구를 학습하지 않고 배정 점수만 학습한다.
따라서 이를 능동 검증 전체의 신규 방법이라고 포장하지 않는다.
외부 관련연구 조사는 시작 단계이며 최근 IROS/ICRA/RAL/TRO의 직접 경쟁법 조사는 미완료다.

## 1. 먼저 고정할 실험 기반

- 출발 코드 bb769e3 + 로컬 WMS 예약 재배정 수정 + RecordQuality v2를 모든 정책에 공통 적용.
- metric_version과 fault_model을 모든 결과에 보관. 과거 결과와 합산 금지.
- 정책은 WMS 관측만 사용. truth, future fault tape, simulator timer는 정책 입력 금지.
- 새로운 fault_model은 S1/S2의 marked Poisson process. 기존 Bernoulli-per-step과
  의미가 완전히 같다고 하지 않는다. 같은 시간당 시도율의 연속시간 모델이며
  legacy 모델 비교를 민감도 실험으로 남긴다. 실제 창고 발생률을 측정한 값이 아니다.
- 이벤트별 독립 해시 난수: 시각/대상/목적지. 현재 유효 LPN을 정렬 후 고정 uniform의
  quantile로 선택한다. 정책별 유효 집합은 다르므로 실제 대상은 달라도 된다.
- S3는 pickup에 조건부인 오류다. LPN+pickup occurrence로 trigger를 고정하고
  시각과 목적지는 정책에 따라 달라짐을 인정한다. 전부 외생 사건이라 부르지 않는다.
- attempted/triggered/injected/skipped 기록을 남긴다. 오류 탐지 이벤트 수는
  인과적으로 특정 fault_id와 연결한 탐지율이 아니다. 복구 지연도 sampled mismatch
  episode 지연이며 실제 fault 원점부터의 지연이라고 주장하지 않는다.

## 2. 계측 정의

주 서비스 지표 후보: unit backlog AUC = 적분(릴리스된 수량-태그가 있는 출하 수량)dt.
정상 출하 검사(mis_shipped=0)와 함께 사용한다. 모든 미완료 수요도 horizon까지 포함한다.
이는 완료 주문만 선택한 평균의 누락 문제를 피하려는 정의상의 선택이며
그 자체로 비용이 실측되었다는 뜻은 아니다. 주문량이 다른 조건은 정규화값도 보고한다.
보조: 출하율, 미완료 주문 수/최대 age, restricted order flow
(완료는 완료시각, 미완료는 종료시각으로 자른 체류시간), 종류별 최초배정 대기시간과
한 번도 배정되지 않은 작업 수·age, 기록 불일치 AUC, 미교정 수/age, 이동거리,
마감 누락, 오출하, WMS 거부/이상, 정책 계산시간.
restricted flow는 장래의 실제 완료시간이나 일반적인 RMST 추정치라고 부르지 않는다.
종료시점 누락, 재배정 대기시간은 별도 확장 대상이다. 최초배정 대기만으로 기아가
없다고 결론 내릴 수 없다. 가동률을 무조건 높이는 것은 목표가 아니다.

## 3. 이번 개발 실험 (결과 보기 전에 설정 저장)

기본 fixture: 2023-03-09, 3600초, 기존 초기배치 seed42. 운반 4/8대, 순찰0.
VERIFY_POLICY=event; period60, max_open4, recent120, neighbors3를 전 방법에 고정.
fault seed 100–109. 이 10회는 런타임·실패 양상·분산 파악용의 계산 예산 선택이다.
논문에서 n=10이면 충분하다는 근거로 쓰지 않는다. 초기배치를 재생성하지 않으므로
초기 상태 일반화 실험이 아니다. 추가 조건을 본 뒤 개발 세트로 명시해 계속 남긴다.

비교: 기존 우선순위+nearest, EDF+nearest, 수동 점수, 배포 CEM(30개 후보),
배포 CEM(모든 후보), nominal ETA greedy, nominal ETA matching.
모든 후보 CEM은 재학습 없이 후보 제한만 바꾸는 ablation이다.
ETA는 알려진 랙 지도 최단거리/속도+look/act 시간이다. 회전/혼잡은 검증 전이라
임의로 추가하지 않는다. 모든 ETA 대조군은 동일 추정을 쓴다.
matching은 매 순간 robot-task matrix를 풀고 슬롯 충돌을 재계산한다.
이런 reconciliation을 포함하므로 전역 최적성 주장은 하지 않는다.
EDF는 이 문제에 적용한 대조 규칙이지 고전 EDF 최적성 정리를 적용한 것이 아니다.

H1: 30개 후보 제한이 종류별 실행 가능 작업을 누락하는가?
  코드 단위 반례 + 실제 excluded task-step/최대 age를 측정. 모든 후보 대조.
H2: 학습이 수동 가중치 대비 서비스 개선을 내는가?
  동일 외생 시드의 paired 차이. 출하와 기록 품질의 악화도 모두 보고.
H3: 거리 대신 경로·서비스 비용만으로 개선이 설명되는가?
  ETA greedy/matching을 추가. CEM의 구조+학습 전체 개선과 학습만의 개선을 구분.

분석: 셀별 paired 평균 차이와 95% percentile bootstrap CI(10,000 resamples,
bootstrap RNG 별도 고정). seed가 독립 반복 단위. 작업/로봇/스텝을 표본으로 부풀리지 않는다.
다중 비교의 marginal CI이므로 유의성 승패표를 만들지 않는다. 최종 주장 family와
일차 지표를 동결하고 동시 추론/다중 비교 보정을 별도로 정한다.
실패 run도 manifest에 기록. 실패를 삭제해서 성공한 seed만 평균 내지 않는다.

## 4. 단계 통과 조건과 남은 작업

1. 공통 코드 단위/반례/재현성 검증, 두 엔진에서 tape/metrics 동일 구현.
2. 개발 진단 결과로 실제 후보 누락·대기문제를 판단. age>=0나 최대 대기 상한은
   아직 미채택. 개선이 필요하면 제한/가중치 선택을 검증 세트로 옮긴다.
3. ETA를 Webots 로그에서 검증. 장면/밀도별 bias·MAE·오차 분포를 보고하며
   보정 데이터와 검증 데이터를 나눈다. 향상폭보다 예측오차가 크면 강한 주장을 보류한다.
4. 확인 가치 정책은 별도 단계: 관측 가능한 risk model의 calibration/Brier score,
   확인 비용/실패 손실 추정, 무작위/주기적/이벤트/oldest-observation 대조,
   확인 횟수/이동 예산을 맞춘 대조, 배정 x 확인 2x2 ablation.
   P(error)*loss-cost는 가설일 뿐. 논문에 있는 검증된 식으로 오인하지 않는다.
5. 학습/튜닝 seed, 시나리오 구성/날짜/레이아웃 split, 학습 반복 수와 실행 예산 동결.
   현재 가중치는 원본 학습 시나리오와 완전한 메타데이터가 없어 확인적 결과에 부적합.
   재학습은 입력과 모든 설정·코드/모델 해시·선택 과정까지 보존한다.
6. 최종 표본수는 개발 분산과 사전 정의한 효과/정밀도에서 계산. 결과가 유리해질
   때까지 반복하지 않는다. 무오류/S1/S2/S3/혼합, 부하·규모·배치 변화 포함.
7. 독립 Webots 검증, 재현 패키지와 제한사항 작성. 사용자 지시에 따라 실물 실험은
   현재 범위에서 제외한다. 시뮬레이션 결과를 실제 산업 창고로 일반화하지 않는다.

앞 단계가 실패하면 수정/재검증하고 실패도 남긴다. 최종 테스트를 개발에 재사용하면
그 테스트는 더 이상 미관측 테스트가 아니며 새 테스트가 필요하다.

## 개발 단계 후속 통제 (v2)

v1의 140회 결과를 본 후 추가. 코드의 events.jsonl 기록/독립 WMS replay 기능을
검증하는 12회 smoke를 먼저 통과했다. 알고리즘은 바꾸지 않았다.
무오류/S1/S2/S3, 운반4대, baseline/heuristic/learned30, fault seed100–109.
무오류는 다른 난수 요인이 없으므로 방법별 1회만 실행한다(3+90=93회).
같은 seed의 smoke와 중복 결과를 독립 표본처럼 합산하지 않는다.
4대는 자원 제약 조건의 기전 분해를 위한 개발 선택이며 8대나 다른 부하에 대한
확인적 일반화 근거로 쓰지 않는다. v1과 v2 모두 최종 테스트가 아니다.
계측/로그를 바꾼 버전은 snapshot으로 보존하고 전체 trace를 WMS에 재생해 비교한다.
EDF가 기존과 같은 이유: 현재 주문 마감이 release 시각의 다음 cutoff로 생성되어
시간순 우선순위와 일치한다. 이 조건에서 독립적으로 강한 대조군이라고 세지 않는다.
missed_cutoffs는 트럭 부재 중 레인에 물건이 있을 때의 횟수이며 주문 지각률이 아니다.
최종 실험에는 실제 정의된 주문 due date별 지각 수량/시간이 따로 필요하다.
