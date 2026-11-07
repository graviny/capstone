# 개발 규격 정리

Mock WMS, WES(Baseline·Proposed), 시뮬레이터를 개발할 때 지켜야 할 형식과 규칙을 모은 문서입니다. 동작 설명은 [wms.md](wms.md), 실험 설정은 [experiments.md](experiments.md), 데이터 출처는 [dataset.md](dataset.md)를 보세요.

## 1. 범위

### 포함

| 항목 | 내용 |
| --- | --- |
| 핵심 시나리오 | S1 오적치, S2 Short Pick / 누락 재고, S3 낡은 작업 정보(목적지 점유) |
| 보조 상황 | 입고 라벨 판독 실패 → 확인(VERIFY) 후 적치 |
| 작업 종류 | PUTAWAY, PICK, VERIFY, RELOCATE |
| 로봇 | 운반(transport), 순찰(patrol) 두 역할, 여러 대 동시 작업 |
| 데이터 | 실제 주문(Footwear 데이터셋), 실제 보관 위치(선택), 나머지는 합성 |
| 비교 | 같은 환경·같은 WMS에서 WES 스케줄러만 바꿔 비교 |

### 제외

| 항목 | 비고 |
| --- | --- |
| 통신 지연·손실 | 메시지는 즉시 전달 |
| 상용 WMS 전체 기능, 결제·배송·ERP | Mock WMS는 비교에 필요한 최소 기능만 |
| 라벨 판독 실패를 핵심 시나리오로 확장 | 보조 상황으로만 사용 |
| 손상 상품 판정·격리, 구역 폐쇄, 대규모 장애 | |
| 로봇 재시작에 따른 상태 복원 | |
| 로봇 간 지식 추정, 이미지 전송량 최소화 | 이전 연구 방향 (커밋 `b95d68e`) |
| 위층 선반 | 로봇은 맨 아래 1단만 다룸 (위층은 장식) |
| 작업자가 로봇 길을 막는 것 | 작업자는 보이기만 함 |

### 단순화한 가정
- 한 칸에 상자 하나, PICK 하나 = 1켤레 = 상자 하나.
- 배송 지역(R1~R4)은 고객 코드 해시로 합성 (데이터에 주소 없음).
- 로봇 위치는 천장 카메라 측위를 가정해 시뮬레이터가 알려 줌. 상자 내용은 카메라로만 확인.

## 2. 책임 경계

| | 시뮬레이터 (Webots) | Mock WMS | WES (스케줄러) |
| --- | --- | --- | --- |
| 실제 상품 위치(Ground Truth) | **가짐** (평가용) | 보지 않음 | 보지 않음 |
| 기록(재고·주문·작업) | – | **관리** | 읽기만 |
| 작업 생성 | – | 주문·입고·미식별 상자에 대해 **생성** | `CREATE`로 요청 (VERIFY, RELOCATE) |
| 보관 위치(PUTAWAY 목적지) | – | **결정·예약** | 예외·대기 중에만 변경 |
| PICK 출발지·출고 레인 칸 | – | 검증 | **결정** |
| 로봇 배정·순서·예외 대응 | – | 검증 | **결정** |
| 이동·관측·집기·놓기 | **수행·판정** | – | – |

- **기록은 확인된 결과로만 바뀝니다.** 지시를 보냈다고 재고를 옮겨 적지 않고, 완료 보고가 와야 바꿉니다.
- WES와 WMS는 `truth.jsonl`, 시뮬레이터 내부 값을 읽으면 안 됩니다.

## 3. 식별자

| 대상 | 형식 | 예 |
| --- | --- | --- |
| 선반 칸 | `r<줄><면><번호 2자리>`. 면 `s`=남쪽, `n`=북쪽. base는 줄 0~3·번호 00~19, large는 줄 0~5·번호 00~39 | `r1n05` |
| 입고 대기 칸 | `B0`~`B5` (B0이 벨트 끝) | `B3` |
| 입고 예외 칸 | `X0`, `X1` | `X0` |
| 출고 레인 칸 | `L<지역>-<0~7>` | `LR2-3` |
| 지역 | `R1`~`R4` | |
| 로봇 | 정수. 순찰 로봇이 앞 번호, 운반 로봇이 뒤 번호 (`scenario.json` `robots`) | `2` |
| SKU | `<상품 코드>-<US 사이즈>` | `8N10W9-10.5` |
| 상자(LPN) | 초기 재고 `ST0001`, 입고 `IN0001`, 반품 `RT001` | |
| 이벤트 | `E` + 6자리 (시뮬레이터가 붙임) | `E000123` |
| 주문, 웨이브 | 데이터셋의 번호를 문자열로 | `"99515"`, `"35491"` |

### 작업 ID

| 작업 | 형식 | 예 |
| --- | --- | --- |
| PICK | `PK-<주문>-<SKU>`, 같은 주문·SKU 두 번째부터 `-2`, `-3` … | `PK-99515-SMMRK3-12-2` |
| PUTAWAY (라벨 읽힘) | `PA-<LPN>` | `PA-IN0012` |
| PUTAWAY (확인 후) | `PA-<칸>`, 반복 시 `-2` … | `PA-X0` |
| VERIFY (입고 미식별) | `VF-<칸>`, 반복 시 `-2` … | `VF-X1-2` |
| WES가 요청한 작업 | `<VF|RL>-W<4자리>` | `RL-W0003` |

같은 작업은 정책이 달라도 같은 ID를 가져야 합니다(시뮬레이터가 S3 발생을 작업 ID로 정함). 생성 순서 번호(`T00001`)는 쓰지 않습니다.

## 4. 값과 단위

| 항목 | 규칙 |
| --- | --- |
| 시간 | 시뮬레이션 초 (`t`), 에피소드 시작이 0 |
| 거리·좌표 | m, 시뮬레이터 좌표계 (x 동쪽 = 출고 쪽, y 북쪽) |
| `priority` | 작을수록 급함. PICK 1, PUTAWAY·VERIFY·RELOCATE 2 |
| `deadline` | 절대 시각(초). PICK만: 그 지역 트럭의 다음 출발 시각(15분 주기) |
| 칸 기록 `item` | SKU, `None`(빈칸), `"UNKNOWN"`(뭔가 있으나 모름) |
| 관측값 `observed` | SKU, `"EMPTY"`, `"UNREADABLE"`(못 읽음), `"UNKNOWN"`(놓기 거부, 내용 모름), `"UNREACHABLE"`(닿지 못함) |

## 5. 작업 (Task)

딕셔너리로 다룹니다. 스케줄러는 읽기만 합니다.

```python
{
  "task_id": "PA-IN0012", "task_type": "PUTAWAY", "item_id": "I1KDJ0-11",
  "source": "B3", "destination": "r1n05", "status": "PENDING",
  "priority": 2, "deadline": None, "state_version": 3,
  "created_t": 412.3, "updated_t": 412.3, "robot_id": None, "attempts": 0,
  "hold_reason": None, "fail_reason": None, "picked": False, "lpn": "IN0012",
  # PICK이면: "order": "99515", "region": "R2", "wave": "35491"
}
```

| 필드 | 누가 정함 | 비고 |
| --- | --- | --- |
| `task_id`, `task_type`, `item_id`, `priority`, `deadline` | WMS (CREATE면 WES 요청값) | |
| `source` | PUTAWAY·VERIFY: WMS / PICK: WES(배정 시) / RELOCATE: WES | |
| `destination` | PUTAWAY: **WMS** / PICK: WES(배정 시) / VERIFY·RELOCATE: 만들 때 | |
| `status`, `robot_id`, `attempts`, `picked`, `hold_reason`, `fail_reason` | WMS | |
| `state_version` | WMS | 작업이 바뀔 때마다 +1 |

### 상태

| 상태 | 뜻 | 들어오는 경우 | 나가는 경우 |
| --- | --- | --- | --- |
| `PENDING` | 배정 대기 | 생성, `REASSIGN` | `EXECUTE` → ASSIGNED, `CANCEL` |
| `ASSIGNED` | 로봇에게 지시함 | `EXECUTE` | 로봇이 시작 보고 → IN_PROGRESS |
| `IN_PROGRESS` | 수행 중 | 로봇 보고, ON_HOLD에서 `REROUTE` | 완료, 예외, 실패 |
| `ON_HOLD` | 멈춤 (예외 또는 `WAIT`) | `SHORT_PICK`, `ITEM_MISMATCH`, `DESTINATION_OCCUPIED`, `WAIT` | `REROUTE`, `REASSIGN`, `CANCEL` |
| `FAILED` | 로봇이 포기 | `TASK_FAILED` | `REASSIGN`(로봇 없이), `CANCEL` |
| `COMPLETED` | 완료 | `TASK_COMPLETED` | 끝 |
| `CANCELLED` | 취소 | `CANCEL` | 끝 |

- PENDING으로 돌아가면 PICK의 출발지·목적지는 지워집니다. PUTAWAY는 목적지가 아직 비어 있으면 유지합니다.
- 재고가 없는 PICK은 별도 상태 없이 PENDING으로 남습니다(출발지를 못 정하므로 배정 안 됨).

## 6. Decision (WES → WMS → 로봇)

```python
{"robot_id": 2, "task_id": "PA-IN0012", "action": "EXECUTE"}
{"robot_id": 3, "task_id": "PK-99515-SMMRK3-12", "action": "EXECUTE", "source": "r1s05", "destination": "LR3-2"}
{"robot_id": 2, "task_id": "PA-IN0012", "action": "REROUTE", "source": "B3", "destination": "r1n06"}
{"robot_id": 2, "task_id": "PA-IN0012", "action": "WAIT"}
{"robot_id": None, "task_id": "PK-99515-SMMRK3-12", "action": "REASSIGN"}
{"robot_id": None, "action": "CREATE", "task": {"task_type": "VERIFY", "destination": "r1n07"}}
```

| action | 뜻 | `robot_id` | 필수 필드 |
| --- | --- | --- | --- |
| `EXECUTE` | 작업 시작 | 로봇 | PICK: `source`, `destination` / 나머지: 없음(작업에 있음) |
| `REROUTE` | 출발지·목적지 변경 | 맡은 로봇 | `source`, `destination` |
| `WAIT` | 그 자리에서 대기 | 맡은 로봇 | |
| `REASSIGN` | 대기열로 되돌림 | 맡은 로봇, 없으면 `None` | |
| `CANCEL` | 작업 취소 | 맡은 로봇, 없으면 `None` | |
| `CREATE` | 새 작업 요청 (VERIFY, RELOCATE만) | `None` | `task.task_type`, VERIFY: `destination` / RELOCATE: `source`, `item_id` |

- `robot_id`가 `None`인 결정은 로봇에게 가지 않고 WMS에만 반영됩니다.
- EXECUTE의 로봇용 작업 정보(`task`)는 WMS가 채웁니다. 스케줄러가 넣어도 덮어씁니다.
- VERIFY는 action이 아니라 작업 종류입니다. 확인이 필요하면 `CREATE`로 VERIFY 작업을 만들고 `EXECUTE`합니다.
- CREATE로 만든 작업의 ID는 WMS가 붙이며, 다음 스텝부터 PENDING으로 보입니다.

### 거부되는 결정

거부된 결정은 로봇에게 가지 않고 `wms_log.jsonl`에 `DECISION_REJECTED`로 남습니다.

| 이유 | 경우 |
| --- | --- |
| `UNKNOWN_TASK`, `UNKNOWN_ACTION`, `NO_SUCH_ROBOT`, `NO_SUCH_SLOT` | 없는 대상 |
| `TASK_<상태>` | 그 상태에서 불가능 (예: COMPLETED 작업에 EXECUTE) |
| `ROBOT_BUSY` | 로봇이 다른 작업 중 (WMS 기준 또는 로봇 보고 기준) |
| `ROLE` | 순찰 로봇에게 VERIFY 외 작업 |
| `MISSING_SLOT`, `SAME_SOURCE_DESTINATION` | 필요한 칸 없음, 출발지 = 목적지 |
| `SLOT_RESERVED` | 다른 작업이 잡아 둔 칸 |
| `SOURCE_NOT_RECORDED` | 출발지에 그 상품 기록 없음 (RELOCATE는 `UNKNOWN` 허용) |
| `DESTINATION_NOT_EMPTY` | 목적지가 기록상 비어 있지 않음 |
| `DESTINATION_SET_BY_WMS` | PUTAWAY 목적지를 WMS가 정한 칸과 다르게 지시 (ON_HOLD의 REROUTE는 허용) |
| `NO_PUTAWAY_SLOT_YET` | 목적지를 아직 못 받은 PUTAWAY에 EXECUTE |
| `NOT_THIS_ROBOT`, `TASK_NOT_ASSIGNED` | 맡은 로봇이 아님 |
| `ROBOT_HOLDING_CARTON` | 상자를 든 로봇에게 CANCEL·REASSIGN |
| `CREATE_WITH_ROBOT`, `CREATE_TYPE` | CREATE에 로봇 지정, 허용 안 된 작업 종류 |

## 7. 이벤트 (시뮬레이터 → WMS)

모든 이벤트에 `event_id`, `t`, `type`이 있습니다. 로봇 이벤트에는 `robot_id`, `task_id`, `slot`, `item_id`가 있습니다.

| type | 추가 필드 | WMS 처리 |
| --- | --- | --- |
| `ORDER_RELEASED` | `wave`, `lines: [{order, customer, region, sku, qty}]` | 주문 등록, PICK 생성 |
| `TRUCK_ARRIVED` | `truck`, `asn: [{lpn, item}]` | ASN 저장 |
| `TRUCK_DEPARTED` | `truck` | – |
| `INBOUND_STAGED` | `slot`, `lpn`, `item` (못 읽었으면 둘 다 `null`) | 기록, PUTAWAY 또는 VERIFY 생성 |
| `INBOUND_READ_FAIL` | `slot` | `UNKNOWN` 기록, VERIFY 생성 |
| `OUTBOUND_DEPARTED` | `region`, `cartons: [{slot, lpn, item, order}]` | 출하 반영, 레인 비우기 |
| `OUTBOUND_TRUCK_ARRIVED` | `region` | 트럭 있음 표시 |
| `TASK_COMPLETED` | `task_type` | 출발지 비우기, 목적지 기록, 완료 |
| `TASK_FAILED` | `reason`: `BUSY`, `HOLDING_CARTON`, `PICK_<이유>` | FAILED (`HOLDING_CARTON`이면 ON_HOLD로 복귀) |
| `SHORT_PICK` | `observed: "EMPTY"` | 출발지 빈칸 기록, ON_HOLD |
| `ITEM_MISMATCH` | `observed`: SKU 또는 `UNREADABLE` | 본 대로 기록, ON_HOLD |
| `DESTINATION_OCCUPIED` | `observed`: SKU, `UNKNOWN`, `UNREACHABLE` | 본 대로 기록(`UNREACHABLE` 제외), ON_HOLD |
| `OBSERVATION_RESULT` | `observed`, `source`: `verify` 또는 `patrol` | 본 대로 기록(`UNREADABLE` 제외), 다르면 예외 |

- 로봇은 놓기 오류(S1)를 모르고 **지시받은 칸으로** 완료를 보고합니다. 그래서 기록이 틀어지는 것이 정상입니다.
- 같은 `event_id`는 한 번만 반영합니다.

## 8. 스케줄러 인터페이스

```python
class Scheduler:
    name = "fifo"                          # 로그 폴더: <날짜>_wms-fifo_seed0_...
    def __init__(self, cfg): ...           # scenario.json 내용
    def plan(self, state): return [...]    # 매 스텝(32 ms), Decision 목록
    def on_event(self, ev, state): return [...]   # 선택: WMS가 이벤트를 반영한 직후
```

실행: `WES_MODULE=wes_runner`, `WES_SCHEDULER=<lib/ 안의 모듈 이름>`.

| `state`에서 쓸 수 있는 것 | 내용 |
| --- | --- |
| `t`, `version` | 현재 시각, WMS 전체 변경 번호 |
| `task(id)`, `tasks`, `pending_tasks()`, `tasks_with(*상태)` | `pending_tasks()`는 (priority, created_t, 대기열 순서) 정렬 |
| `records`, `item_at(칸)`, `slots`, `pos(칸)` | 칸 기록(`item`, `lpn`, `version`, `updated_t`, `observed_t`, `reserved_by`)과 칸 정보 |
| `free_slots`, `slots_with(SKU)`, `stock(SKU)`, `lane_free_slot(지역)`, `putaway_slot(SKU, near)` | 예약 안 된 칸 찾기 (`exclude`로 이번에 고른 칸 제외) |
| `robots`, `idle_robots(role)` | 로봇 `{id, role, pos, task, holding, phase, assigned}` |
| `orders`, `lanes`, `lane_truck`, `exceptions`, `out_xy` | 주문, 레인, 트럭 유무, 불일치 목록, 출고 쪽 중심 |
| `dist(a, b)`, `next_cutoff(t)`, `snapshot()` | 거리, 다음 출고 마감, JSON 복사본 |

지켜야 할 것:
- `state`는 읽기 전용입니다. 바꾸려면 Decision을 돌려줍니다.
- 한 번의 `plan()`에서 같은 칸을 두 작업에 주지 않습니다. 결정은 `plan()`이 끝난 뒤 반영됩니다.
- 실제 상태(시뮬레이터 내부, `truth.jsonl`)를 읽지 않습니다.
- 난수를 쓰면 시드를 고정합니다(같은 시나리오 → 같은 결과).
- `name`은 로그 폴더 이름에 들어가므로 영문·숫자·`_`·`-`만 씁니다.

## 9. 로그

폴더: `logs/<날짜>_<WES>_seed<시드>_<faults|nofaults>[_sweep][_<run-name>]_<batch|gui>/`

| 파일 | 만드는 곳 | 내용 |
| --- | --- | --- |
| `events.jsonl` | 시뮬레이터 | 이벤트 + 통과된 결정(`"kind": "DECISION"`) |
| `truth.jsonl` | 시뮬레이터 | 주입된 오류 원인, 진단(`ACT_REJECTED`, `INVARIANT_VIOLATION`, `ROBOT_STUCK`) |
| `poses.csv`, `timeseries.csv`, `summary.json` | 시뮬레이터 | 위치(2초), 상태(10초), 결과 요약 |
| `wms_log.jsonl` | WMS | 작업·칸·주문 변화, 예외, 이상, 거부된 결정, 최종 상태 |
| `scenario.json` | WMS 연결부 | 그 실행이 쓴 시나리오 사본 |

## 10. 실험 규칙

- 정책 비교는 **같은 설정 파일 + 같은 시드**로 하고, 시드를 여러 개 돌려 평균을 봅니다.
- 실험마다 `--run-name`을 다르게 줍니다(안 주면 폴더를 덮어씀).
- 결과를 볼 때 `summary.json`의 `diagnostics`(특히 `ROBOT_STUCK`)와 `wes_stats.counts`(거부·이상)를 같이 확인합니다.
- `tools/replay_wms.py`로 재생이 일치하는지, `tools/compare_runs.py`로 주문 릴리스가 같은지 확인합니다.
- `gen_world.py`가 덮어쓰는 `scenario.json`, 월드, 라벨 그림은 실험 결과와 함께 커밋하지 않습니다.

## 11. 미정 사항

| 항목 | 현재 | 선택지 |
| --- | --- | --- |
| S3 발생 기준 | 작업 ID + 목적지 칸 (`supervisor.py`) | 목적지 빼기(LPN + 적치 횟수) / 그대로 두고 시드 평균 / 미리 정한 일정표 |
| 기록과 다른 시도 | 거부 (`SOURCE_NOT_RECORDED` 등) | 그대로 / `"speculative": true` 표시가 있으면 허용 (모든 정책에 동일하게) |
