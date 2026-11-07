# Mock WMS와 WES 연결 규격 (v1)

Mock WMS는 창고의 **기록**(재고·주문·입고·작업)을 관리하고, 입고 상자를 어디에 보관할지도 정합니다. Baseline과 Proposed WES(스케줄러)는 이 기록만 보고 판단하고, 시뮬레이터의 실제 상태(Ground Truth)는 보지 못합니다. 데이터 출처는 [dataset.md](dataset.md)에 정리돼 있습니다. 형식과 규칙만 빠르게 찾으려면 [spec.md](spec.md)를 보세요.

## 1. 구조

```text
supervisor ──이벤트──▶ lib/wes_runner.py (WES_MODULE=wes_runner)
                         ├─ wms.apply_event(ev)            기록·작업 갱신 (lib/wms.py)
                         ├─ scheduler.on_event(ev, state)  (선택) 이벤트에 바로 반응
                         ├─ wms.update_robots(t, robots)   로봇 보고 반영 (작업 시작, 상자 집음)
                         ├─ scheduler.plan(state)          매 스텝 배정·경로 변경·대기 결정
                         └─ wms.apply_decision(d)          결정마다 기록과 대조, 통과한 것만 로봇에 전달
```

| 파일 | 내용 |
| --- | --- |
| `lib/wms.py` | `MockWMS`(기록과 작업 관리), `WMSView`(스케줄러가 보는 state) |
| `lib/wes_runner.py` | 시뮬레이터가 부르는 WES 모듈. WMS와 스케줄러를 연결 |
| `lib/sched_standin.py` | 임시 스케줄러. `wes_standin.py`의 배정 규칙을 그대로 옮김 (Baseline이 나오면 교체) |
| `tools/replay_wms.py` | 실행 로그(`events.jsonl`)로 WMS를 다시 만들어 실행 결과와 비교 |
| `tools/compare_runs.py` | 여러 실행의 `summary.json`을 나란히 비교 |
| `tests/test_wms.py` | 단위 테스트 (Webots 필요 없음) |

`wes_standin.py`는 그대로 두었습니다. `WES_MODULE`을 지정하지 않으면 예전처럼 standin으로 돌아갑니다.

## 2. 실행

명령은 `webots/` 폴더에서 실행합니다.

```powershell
# 단위 테스트
python -m unittest discover -s tests -v

# WMS + 스케줄러로 시뮬레이션 (배치)
$env:QUIT_AT_END = "1"
$env:WES_MODULE = "wes_runner"
$env:WES_SCHEDULER = "sched_standin"     # lib/ 안의 스케줄러 모듈 이름
& "<Webots 설치 경로>\msys64\mingw64\bin\webots.exe" --batch --mode=fast --minimize --no-rendering --stdout --stderr worlds/warehouse.wbt

# 실행 후 확인
python tools/replay_wms.py logs/<날짜>_wms-standin_seed0_faults_batch
python tools/compare_runs.py logs/<실행 A> logs/<실행 B>
```

로그 폴더 이름의 WES 자리는 `wms-<스케줄러 name>`이 됩니다. WMS 로그 `wms_log.jsonl`과 그 실행이 쓴 `scenario.json` 사본은 시뮬레이터 로그와 같은 폴더에 생깁니다(`WMS_LOG_DIR`로 바꿀 수 있음). 오류율·입고량 같은 실험 설정은 [experiments.md](experiments.md)에 있습니다.

## 3. 작업 (Task)

### 상태

| 상태 | 뜻 | 다음 |
| --- | --- | --- |
| `PENDING` | 배정을 기다림 | `EXECUTE` → ASSIGNED |
| `ASSIGNED` | 로봇에게 지시함 | 로봇이 자기 작업으로 보고하면 IN_PROGRESS |
| `IN_PROGRESS` | 로봇이 수행 중 | 완료, 예외, 실패 |
| `ON_HOLD` | 로봇이 멈춤: 예외(`SHORT_PICK`, `ITEM_MISMATCH`, `DESTINATION_OCCUPIED`) 또는 `WAIT` | WES가 `REROUTE`(→ IN_PROGRESS) / `REASSIGN`(→ PENDING) / `CANCEL` |
| `FAILED` | 로봇이 포기함(`TASK_FAILED`) | WES가 `REASSIGN`(→ PENDING) 또는 `CANCEL` |
| `COMPLETED` | 완료, 기록 반영됨 | 끝 |
| `CANCELLED` | 취소 | 끝 |

`hold_reason`, `fail_reason`에 이유가 남습니다. 예외가 나도 **WMS는 대응을 정하지 않습니다.** 기록을 고치고 작업을 ON_HOLD로 둔 뒤 WES의 결정을 기다립니다.

### 필드

`task_id`, `task_type`(PUTAWAY, PICK, VERIFY, RELOCATE), `item_id`(SKU), `source`, `destination`, `status`, `priority`(작을수록 급함: PICK 1, 나머지 2), `deadline`(PICK: 그 지역 출고 트럭의 다음 마감 시각), `state_version`(바뀔 때마다 +1), `created_t`, `robot_id`, `attempts`, `hold_reason`, `fail_reason`, `picked`(로봇이 상자를 집었는지), PICK이면 `order`, `region`, `wave`, 상자를 알면 `lpn`.

### 작업 ID

원본 데이터에서 정해지게 해서, 정책이 달라도 같은 작업은 같은 ID를 갖습니다. 시뮬레이터의 S3 발생 여부가 작업 ID로 정해지기 때문입니다.

| 작업 | ID | 예 |
| --- | --- | --- |
| PICK | `PK-<주문>-<SKU>`, 같은 주문·SKU의 두 번째부터 `-2`, `-3` | `PK-99515-SMMRK3-12-2` |
| PUTAWAY (라벨 읽힘) | `PA-<LPN>` | `PA-IN0012` |
| PUTAWAY (확인 후) | `PA-<칸>`, 반복되면 `-2` … | `PA-X0` |
| VERIFY (입고 미식별) | `VF-<칸>` | `VF-X1-2` |
| WES가 만든 작업 | `VF-W0001`, `RL-W0002` … | |

### WMS가 스스로 만드는 작업

| 계기 | 작업 |
| --- | --- |
| `ORDER_RELEASED` | 주문 줄 수량 1개당 PICK 1개 (출발지·목적지는 WES가 배정할 때 정함) |
| `INBOUND_STAGED` (라벨 읽힘) | PUTAWAY (출발지 = 대기 칸, 목적지 = WMS가 정한 보관 칸) |
| `INBOUND_STAGED` (라벨 못 읽음), `INBOUND_READ_FAIL` | VERIFY (목적지 = 그 칸) |
| 입고 칸의 VERIFY 완료, 입고 칸 PUTAWAY 취소 | 그 칸의 기록대로 PUTAWAY(식별됨) 또는 VERIFY(여전히 UNKNOWN) |

그 밖의 작업(예: 재고 칸 확인 VERIFY, 오적치 복구 RELOCATE)은 WES가 `CREATE` 결정으로 요청합니다.

### 보관 위치 (PUTAWAY 목적지)

보관 위치는 WMS가 정합니다. 그래서 Baseline과 Proposed는 같은 보관 규칙 위에서 **배정과 재계획만** 비교됩니다.

- PUTAWAY를 만들면 WMS가 바로 빈 선반 칸을 골라 **예약**해요. 규칙(`choose_putaway_slot`)은 A등급은 출고 쪽 가까이, B는 중간, C는 멀리예요.
- 기록상 빈칸이 없으면 목적지 없이 기다리다가, 칸이 비는 순간(이벤트·결정이 반영될 때마다 확인) 오래된 PUTAWAY부터 칸을 받아요.
- WES는 EXECUTE에서 목적지를 바꿀 수 없어요. 목적지를 바꾸는 건 **작업이 ON_HOLD일 때**(목적지가 차 있었거나 WAIT 중)의 REROUTE만 허용돼요. 대체 칸은 `state.putaway_slot(SKU, near=칸)`으로 같은 규칙에서 찾을 수 있어요.
- PENDING으로 돌아간 PUTAWAY는 그 칸이 아직 비어 있으면 같은 칸을 유지해요.

## 4. 칸 기록

`state.records[칸]`:

| 필드 | 뜻 |
| --- | --- |
| `item` | 기록상 SKU, `None`(빈칸), `"UNKNOWN"`(뭔가 있지만 모름) |
| `lpn` | 상자 번호(알 때만) |
| `version` | 기록이 바뀔 때마다 +1. 새 상자가 놓이면 SKU가 같아도 +1 |
| `updated_t` | 마지막으로 바뀐 시각 |
| `observed_t` | 로봇이 마지막으로 직접 본 시각(본 결과가 기록과 같아도 갱신) |
| `reserved_by` | 이 칸을 출발지·목적지로 잡아 둔 작업 |

`version`과 `observed_t`는 불확실성 모델(정보가 얼마나 오래됐나)에 쓰라고 둔 값입니다.

## 5. 이벤트별 처리

| 이벤트 | WMS가 하는 일 |
| --- | --- |
| `ORDER_RELEASED` | 주문 등록(마감 시각 포함), PICK 생성 |
| `TRUCK_ARRIVED` | ASN(예정 상자 목록) 저장 |
| `INBOUND_STAGED` | 대기 칸 기록, ASN과 대조(다르면 이상 기록), PUTAWAY 또는 VERIFY 생성. 같은 칸에서 이전 상자를 가져간 작업은 "집음"으로 처리 |
| `INBOUND_READ_FAIL` | 예외 칸을 UNKNOWN으로, VERIFY 생성 |
| `TASK_COMPLETED` | 출발지 비우기(그 사이 새 상자가 기록됐으면 그대로 둠), 목적지에 상품 기록, PICK이면 주문의 피킹 수량 +1 |
| `TASK_FAILED` | 작업 FAILED. `BUSY`(다른 일 중이라 거절)는 그 로봇에게 준 작업만 해당. `HOLDING_CARTON`(상자를 든 채 취소 거절)은 작업을 다시 그 로봇의 ON_HOLD로 되돌림 |
| `SHORT_PICK`, `ITEM_MISMATCH` | 출발지를 본 대로 기록(EMPTY → 빈칸, 못 읽음 → UNKNOWN), 그 칸 예약 해제, 작업 ON_HOLD, 예외 목록에 추가 |
| `DESTINATION_OCCUPIED` | 목적지를 본 대로 기록(`UNREACHABLE`이면 기록은 그대로), 예약 해제, 작업 ON_HOLD, 예외 목록에 추가 |
| `OBSERVATION_RESULT` | 본 대로 기록(`UNREADABLE`이면 그대로). 기록과 달랐으면 `RECORD_DISCREPANCY` 예외 |
| `OUTBOUND_DEPARTED` | 실린 상자를 주문별 출하 수량에 반영, 그 지역 레인 기록 비우기, 트럭 없음 표시 |
| `OUTBOUND_TRUCK_ARRIVED` | 트럭 있음 표시 |

같은 `event_id`가 다시 오면 무시하고, 이미 끝난 작업의 완료 보고는 기록을 바꾸지 않습니다(둘 다 이상으로 기록).

## 6. Decision (WES → WMS → 로봇)

```python
{"robot_id": 3, "task_id": "PK-99515-SMMRK3-12", "action": "EXECUTE", "source": "r1s05", "destination": "LR3-2"}
```

| action | 뜻 | robot_id |
| --- | --- | --- |
| `EXECUTE` | 작업 시작. 로봇에게 보낼 작업 정보(`task`)는 WMS가 채움 | 로봇 |
| `REROUTE` | 출발지·목적지 변경(보내지 않은 쪽은 그대로) | 맡은 로봇 |
| `WAIT` | 그 자리에서 대기(ON_HOLD) | 맡은 로봇 |
| `REASSIGN` | 작업을 대기열로 되돌림(PENDING). 로봇이 있으면 로봇은 작업을 끝냄 | 맡은 로봇, 없으면 `None` |
| `CANCEL` | 작업 취소 | 맡은 로봇, 없으면 `None` |
| `CREATE` | 새 작업 요청: `{"robot_id": None, "action": "CREATE", "task": {"task_type": "VERIFY", "destination": "r1s07"}}` (VERIFY, RELOCATE만) | `None` |

`robot_id`가 `None`인 결정은 로봇에게 가지 않고 WMS에만 반영됩니다(시뮬레이터 로그에는 남음).

### WMS가 거부하는 결정

거부된 결정은 로봇에게 가지 않고 `wms_log.jsonl`에 `DECISION_REJECTED`로 남습니다(`summary.json`의 `wes_stats.counts`에도 집계).

| 이유 | 경우 |
| --- | --- |
| `UNKNOWN_TASK`, `UNKNOWN_ACTION`, `NO_SUCH_ROBOT`, `NO_SUCH_SLOT` | 없는 작업·결정·로봇·칸 |
| `TASK_<상태>` | 그 상태에서 할 수 없는 결정 (예: 완료된 작업에 EXECUTE) |
| `ROBOT_BUSY` | 로봇이 다른 작업 중 |
| `ROLE` | 순찰 로봇에게 VERIFY 말고 다른 작업 |
| `MISSING_SLOT`, `SAME_SOURCE_DESTINATION` | 필요한 칸이 없음, 출발지 = 목적지 |
| `SLOT_RESERVED` | 다른 작업이 잡아 둔 칸 |
| `SOURCE_NOT_RECORDED` | 출발지에 그 상품이 기록돼 있지 않음 (RELOCATE는 UNKNOWN 허용) |
| `DESTINATION_NOT_EMPTY` | 목적지가 기록상 비어 있지 않음 |
| `DESTINATION_SET_BY_WMS` | PUTAWAY 목적지를 WMS가 정한 칸과 다르게 지시 (ON_HOLD일 때의 REROUTE는 허용) |
| `NO_PUTAWAY_SLOT_YET` | 기록상 빈칸이 없어 아직 목적지를 받지 못한 PUTAWAY에 EXECUTE |
| `NOT_THIS_ROBOT`, `TASK_NOT_ASSIGNED` | 작업을 맡은 로봇이 아님 |
| `ROBOT_HOLDING_CARTON` | 상자를 든 로봇에게 CANCEL/REASSIGN (로봇이 어차피 거절함) |

## 7. 스케줄러 만들기

```python
class Scheduler:
    name = "fifo"                       # 로그 폴더: <날짜>_wms-fifo_seed0_...

    def __init__(self, cfg):            # scenario.json
        ...

    def plan(self, state):              # 매 스텝 (32 ms)
        out = []
        for task in state.pending_tasks():
            ...
        return out

    def on_event(self, ev, state):      # 선택: WMS가 이벤트를 반영한 직후
        return []
```

`state`(`WMSView`)는 읽기 전용입니다. 바꾸고 싶으면 Decision을 돌려줍니다.

| 속성·메서드 | 내용 |
| --- | --- |
| `t`, `version` | 현재 시각, WMS 전체 변경 번호 |
| `tasks`, `task(id)`, `pending_tasks()`, `tasks_with(*상태)` | 작업. `pending_tasks()`는 (우선순위, 생성 시각, 대기열에 들어온 순서)로 정렬 |
| `records`, `item_at(칸)`, `slots`, `pos(칸)` | 칸 기록과 칸 정보(종류, 좌표, 접근 지점) |
| `free_slots(kind, exclude)`, `slots_with(SKU, kind, exclude)`, `stock(SKU)` | 기록상 빈칸·그 상품이 있는 칸(예약 안 된 것), 재고 수 |
| `lane_free_slot(지역, exclude)`, `lanes`, `lane_truck` | 출고 레인의 빈칸, 트럭이 도크에 있는지 |
| `putaway_slot(SKU, near, exclude)` | WMS 보관 규칙으로 고른 빈칸: `near` 근처, 없으면 등급별. 목적지가 차 있을 때 대체 칸 찾기용 |
| `robots`, `idle_robots(role)` | 로봇 보고(위치, 수행 중 작업, 든 상품, 단계)와 WMS가 준 작업(`assigned`) |
| `orders`, `exceptions` | 주문(수량, 피킹·출하 수, 마감), 로봇이 발견한 불일치 목록 |
| `dist(a, b)`, `next_cutoff(t)`, `snapshot()` | 거리, 다음 출고 마감, JSON으로 바꿀 수 있는 복사본 |

주의할 점:

- **한 번의 `plan()`에서 같은 칸을 두 작업에 주지 않도록** 직접 챙겨야 합니다. 결정은 `plan()`이 끝난 뒤 반영되므로, `exclude`에 이번에 고른 칸을 넣으세요. 겹치면 두 번째 결정이 `SLOT_RESERVED`로 거부됩니다.
- PICK의 출발지(재고가 기록된 선반 칸)와 목적지(주문 지역의 출고 레인 칸)는 배정할 때 WES가 정합니다. PICK이 PENDING으로 돌아가면 이 값은 지워집니다.
- PUTAWAY의 출발지(대기 칸)와 목적지(보관 칸)는 WMS가 정해 둡니다. EXECUTE에는 로봇만 정해 보내면 됩니다.

## 8. 로그와 재생

`wms_log.jsonl` 한 줄 = 한 변화:

| type | 내용 |
| --- | --- |
| `START`, `FINAL` | 시작(시나리오 해시), 끝(모든 칸 기록, 작업 결과, 주문별 피킹·출하 수) |
| `TASK_CREATED`, `TASK_STATUS`, `TASK_REROUTED`, `TASK_PICKED` | 작업 생성, 상태 변화(이전 → 이후, 원인), 경로 변경, 상자 집음 |
| `SLOT` | 칸 기록 변화(이전 → 이후, 원인, 관련 작업·이벤트) |
| `ORDER_RELEASED`, `ORDER_SHIPPED`, `ASN` | 주문, 출하 완료(처리 시간), 입고 예정 |
| `EXCEPTION` | 로봇이 발견한 불일치(SHORT_PICK, ITEM_MISMATCH, DESTINATION_OCCUPIED, RECORD_DISCREPANCY) |
| `ANOMALY` | 기록이 일관되면 생기지 않아야 할 일(중복 이벤트, 끝난 작업의 보고, ASN과 다른 입고 등) |
| `DECISION_REJECTED` | 거부된 결정과 이유 |

`tools/replay_wms.py`는 `events.jsonl`(이벤트 + 통과된 결정)만으로 WMS를 다시 만들고, 실행 끝의 기록·작업 결과·주문 수량이 `FINAL`과 같은지 확인합니다. 스케줄러가 이벤트에 반응해 낸 결정에는 `after_event`가 붙어 있어 같은 순서로 재생됩니다.

## 9. standin과 다른 점

`sched_standin`은 `wes_standin`과 같은 규칙으로 배정하지만, 보관 위치를 WMS가 정하고 기록을 더 정확히 다루므로 몇 가지가 다릅니다.

| 상황 | wes_standin | Mock WMS |
| --- | --- | --- |
| 작업 ID | `T00001`처럼 생성 순서 | 원본 데이터 기준(위 표) |
| PUTAWAY 목적지 | 로봇에게 배정하는 순간 WES가 고름 | PUTAWAY를 만들 때 WMS가 고르고 예약 (같은 등급 규칙) |
| 대기 칸에서 상자를 가져간 뒤 새 상자가 들어옴 | 이전 작업이 끝날 때 새 상자 기록까지 지움 | 칸 version으로 구분해 새 상자 기록 유지 |
| 지워진 대기 칸의 미식별 상자 | VERIFY 재시도가 안 됨 | 재시도 |
| 대기열로 돌아간 PUTAWAY | 예전 목적지를 확인 없이 다시 씀 | 아직 비어 있으면 유지, 아니면 새로 받음 |
| 레인에 놓자마자 트럭이 떠나 완료 보고가 출발보다 늦게 옴 | 이미 실려 간 상자를 레인에 기록 (다음 출발까지 그 칸을 못 씀) | 출발 때 실린 것을 확인해 두고 완료 보고에서 레인 기록을 남기지 않음 |
| 상태 이름 | `DONE`, `BACKORDER` | `COMPLETED`, 재고 없는 PICK은 그냥 `PENDING` |

## 10. 확인한 실행 결과

2023-03-09, 1시간, 순찰 2대·운반 6대, 시드 0, `--plain` 월드에서 확인했습니다. 실행 한 번에 약 13분(i5-1135G7, 혼자 돌릴 때) 걸립니다. WMS 열은 보관 위치를 WMS가 정하는 지금 버전입니다.

| 항목 | 오류 끔: standin | 오류 끔: WMS | 오류 켬: standin | 오류 켬: WMS | 오류 켬: WMS, 데이터셋 배치 |
| --- | --- | --- | --- | --- | --- |
| 완료 주문 / 릴리스 주문 | 16 / 19 | 16 / 19 | 15 / 19 | 11 / 19 | 14 / 19 |
| 출하 / 출고 수량 | 126 / 133 | 127 / 133 | 125 / 133 | 112 / 133 | 125 / 133 |
| 평균 주문 처리 시간 | 716초 | 677초 | 690초 | 518초 | 851초 |
| 놓친 마감 | 1 | 1 | 1 | 3 | 1 |
| 주입된 원인: S1 놓기·S1 이동 / S2 / S3 | 0 | 0 | 3·5 / 7 / 14 | 1·4 / 4 / 6 | 3·4 / 5 / 13 |
| 기록 불일치 칸(끝) | 0 | 0 | 11 | 9 | 11 |
| 로봇 교착(`ROBOT_STUCK`) | 0 | 0 | 0 | 12 | 4 |
| 거부된 결정, 이상 기록 | – | 0, 0 | – | 0, 1 | 0, 0 |
| 재생(`replay_wms.py`) | – | 같음 | – | 같음 | 같음 |

- **오류를 끄면 WMS 버전도 작업 241건(PICK 133, PUTAWAY 108)을 모두 끝냈습니다(M2).** 보관 위치를 정하는 시점이 달라져 결정 순서는 standin과 조금 다릅니다.
- **오류를 켠 WMS 실행(4번째 열)은 수치가 크게 낮은데, WMS가 아니라 로봇 주행 교착 때문입니다.**
  - 로봇 2와 7이 대기 칸 B0의 접근 지점(x≈-5.5, y≈1.7~1.9)에서 0.3 m 거리로 마주 본 채 t≈1033부터 끝까지 움직이지 못했습니다. 운반 로봇 6대 중 2대가 빠진 셈입니다.
  - 교착 전까지 대기 칸으로 가는 로봇 수는 이전 버전과 같았습니다(t<1000 평균 1.69대, 이전 1.62대).
  - 데이터셋 배치 실행에서도 같은 자리에서 4번 멈췄다가 풀렸습니다.
  - 원인은 경로 재계획이었습니다(11절). `nav.py`를 고친 뒤 같은 조건으로 다시 돌리면 교착 없이 작업 246건을 모두 끝내고, 출하 126/133, 완료 주문 16/19가 나옵니다.
- 이상 기록 1건은 "레인에 놓자마자 트럭이 떠나 완료 보고가 늦게 온" 경우였고, 이 실행 뒤에 WMS가 처리하도록 고쳤습니다(9절 표).
- 이전 버전(WES가 보관 위치를 정함)에서는 오류를 끈 실행의 결정 241개가 standin과 하나도 다르지 않았습니다. 배정 규칙을 그대로 옮겼다는 확인입니다.
- 적치 108건 중 94건은 로봇이 대기 칸에서 상자를 가져간 뒤 그 칸에 새 상자가 들어왔습니다. standin은 이때마다 새 상자의 기록을 지웠고, Mock WMS는 유지합니다.
- 시드 하나의 결과라 정책이나 배치 방식의 우열을 말할 수는 없습니다.

## 11. 알려진 문제

- **대기 칸 앞 주행 교착 (수정됨):** 두 로봇이 0.35 m보다 가까우면, 자기 위치가 상대 로봇을 막는 원 안에 들어가 경로 재계획이 항상 실패했습니다. 그래서 둘 다 예전 경로를 든 채 서로를 기다렸습니다. 지금은 `nav.py`의 `plan()`이 출발점 주변 0.15 m를 비워 두어 빠져나갈 수 있습니다.
  - 우선순위 로봇이 정지 거리 안에서 멈추고 물러나지 않는 규칙은 그대로라, 더 좁은 곳에서는 비슷한 일이 다시 생길 수 있습니다. 실험 결과를 볼 때 `truth.jsonl`의 `ROBOT_STUCK`과 `summary.json`의 `diagnostics`를 함께 확인하세요.
  - 이 수정은 재계획 경로에 영향을 주므로 README의 standin 수치와 조금 다를 수 있습니다.

## 12. 남은 결정

### S3 발생 기준

`supervisor.py`는 로봇이 상자를 집을 때 `작업 ID + 목적지 칸`으로 S3(목적지에 반품 상자가 놓임) 여부를 정합니다. 목적지가 달라지면 S3도 다른 작업에서, 다른 횟수로 생깁니다. 보관 위치를 WMS가 정하게 되면서 정책 간 차이는 줄었지만, 목적지가 바뀌는 경우(REROUTE, 칸이 비는 순서 차이)에는 여전히 달라집니다.

| 방법 | 내용 |
| --- | --- |
| 기준에서 목적지 빼기 | `상자 LPN + 몇 번째 적치인지`로 결정 (S1 놓기 오류와 같은 방식). 어느 정책이든 같은 상자의 적치에서 S3가 남 |
| 그대로 두기 | 여러 시드의 평균으로 비교 |
| 미리 정한 일정표 | "몇 번째 적치에서 S3"를 시나리오 파일에 고정 |

### 기록과 다른 시도 허용

WMS는 기록과 맞지 않는 결정(`SOURCE_NOT_RECORDED`, `DESTINATION_NOT_EMPTY` 등)을 거부합니다. 그러면 "SHORT_PICK이 난 칸 옆에 상자가 있을 것 같으니 바로 집으러 가 보기" 같은, 기록 너머를 추론한 행동도 막힙니다.

| 방법 | 내용 |
| --- | --- |
| 지금처럼 엄격하게 | 먼저 VERIFY(CREATE)로 확인해 기록을 고친 뒤 피킹 |
| 시도 표시 허용 | 결정에 `"speculative": true`가 있으면 기록과 달라도 허용. 로봇은 집기 전에 라벨을 확인하므로 잘못 집지는 않음. 모든 정책에 같은 기능을 열어 둬야 공정함 |
