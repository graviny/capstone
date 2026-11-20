# 직접 경쟁 연구 검토 — 2026-11-20

문헌의 문제/가정이 다른 경우 성능 숫자를 우리 수치와 비교하지 않는다.
아래는 초기 조사이며 체계적 문헌고찰 완료나 신규성 증명이 아니다.

| 연구 | 확인한 내용/열람 범위 | 우리 연구에 요구되는 비교 |
|---|---|---|
| Akkerman, Prak, Mes, EJOR 321(2), 2025, *Dynamic reordering and inspection for the multi-item Inventory Record Inaccuracy problem*, [저자 기관](https://research.utwente.nl/en/publications/dynamic-reordering-and-inspection-for-the-multi-item-inventory-re/), DOI 10.1016/j.ejor.2024.09.033 | 기관의 대체 호스트에서 [원문](https://ris.utwente.nl/ws/portalfiles/portal/466019270/1-s2.0-S0377221724007367-main.pdf) 접근 성공. 도입과 §4.2/5.1 확인. 재고 감소, 검사/이동시간, 재주문+MIP 검사. | '확인 가치와 이동비용을 결합' 자체는 신규성 아님. 감소만 있는 재고·보충 문제와 우리 공간 오적치·동시 로봇 실행의 차이를 형식화할 것. 임의 구현을 해당 방법의 재현이라고 부르지 않음. |
| Kök & Shang, M&SOM 9(2), 2007, [Inspection and Replenishment Policies for Systems with Inventory Record Inaccuracy](https://pubsonline.informs.org/doi/10.1287/msom.1060.0136) | 출판사 초록 확인. 비용을 가진 검사와 보충의 결합, 상태 의존 검사/재고 정책. | 단순 주기적 검사만을 약한 대조군으로 두지 말 것. 로봇 이동/경합이라는 차이를 명확하게 보일 것. |
| DeHoratius, Mersereau, Schrage, M&SOM 2008, [Retail Inventory Management When Records Are Inaccurate](https://pubsonline.informs.org/doi/10.1287/msom.1070.0203) | 출판사 초록 확인. 실제 재고에 대한 Bayesian belief와 관측에 따른 갱신. | 관측 나이만으로 만든 점수를 확률이라고 부르면 안 됨. belief 또는 경험적 위험 확률의 calibration 필요. |
| Choudhury et al., RSS 2020, [Dynamic Multi-Robot Task Allocation under Uncertainty and Temporal Constraints](https://arxiv.org/abs/2005.13109), [공식 코드](https://github.com/sisl/SCoBA.jl) | 저자 초록과 공식 저장소 README 확인. 개별 정책 트리+상위 충돌 해소, 시간창/완료 불확실성. Julia 도메인 어댑터 제공. 논문의 보장은 해당 가정 아래에 한정. | 단순 nearest만 이기면 부족. 우리 문제의 공동 재고 상태/검사에 의한 belief 변화가 이 방법의 가정과 맞는지 확인 후 도메인 어댑터 설계. 코드를 아직 실행하지 않았음. |
| Dhanaraj et al., ICRA 2024, [Multi-Robot Task Allocation Under Uncertainty Via Hindsight Optimization](https://par.nsf.gov/servlets/purl/10651010), DOI 10.1109/ICRA57147.2024.10611370 | 저자 페이지와 논문 초록 확인. 실패/후속 복구 작업을 고려한 계층적 hindsight optimization. | '예외 재배정' 자체는 신규성 아님. 실패 확률 모델, 미래샘플 예산, 목적함수 차이를 명시하고 계산 예산을 맞춘 비교를 검토. |
| Leet et al., IROS 2023, [Task Assignment, Scheduling, and Motion Planning for Automated Warehouses for Million Product Workloads](https://idm-lab.org/bib/abstracts/papers/iros23.pdf) | 원문 초록/도입 확인. 작업 구성·배정·스케줄·모션의 결합과 계약 기반 교통 분해. | 우리는 작은 규모의 재고 불일치/확인 문제임. 산업 규모 해결 주장 금지. 배정 개선을 모션플래너 개선과 혼동 금지. |

## 신규성 가설 (아직 주장 불가)

후보: 이벤트로 드러나는 공간적 재고 불일치 아래에서 확인 행동의 의사결정 가치를
추정하고, 확인에 의해 가능한 작업과 로봇 자원 경합이 바뀌는 효과를 공동으로
반영하는 온라인 정책. 단순 CEM/age/ETA 추가는 구성요소이고 그 자체가 논문 기여는 아니다.

필요 증거: 선행 방법 대비 추가되는 상태/의사결정 구조의 수학적 정의,
정보 누설 없는 위험 추정 검증, 확인 예산을 맞춘 검사 정책 비교, 강한 배정 대조군,
배정×확인 ablation, 시뮬레이터에 의존하지 않는 지연/센서 조건 검증.
원문 세부 가정을 확인하기 전 SCoBA/EJOR 방법을 배제하거나 열등하다고 결론 내리지 않는다.

## 사람과 현장 자료가 필요한 부분

실측 오류율·작업 지연·인식 실패율·작업자 행동 자료는 아직 없다.
지도교수가 없다는 이유로 문헌의 다른 도메인 수치를 그대로 가져와 사실처럼 쓰지 않는다.
실물 측정이나 공개 자료로 보정할 수 없다면 합성 시뮬레이션 가정으로 명시하고 민감도를 보고한다.
논문 최종 제출 전 저자 전원의 주장/실험/출처 검토가 필요하며, 자동화된 검토는 독립 외부
연구자의 비판적 검토를 완전히 대체하지 못한다.
