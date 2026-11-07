# 실험 설정

실험 하나는 **설정 + 시드**로 정해집니다. `tools/gen_world.py`가 설정과 시드로 `scenario.json`과 월드를 만들고, 시뮬레이터는 그 파일대로 실행합니다. 같은 설정과 시드로 만들면 같은 파일이 나옵니다.

## 1. 설정 방법

```powershell
# 명령 인자로
python tools/gen_world.py --plain --fault-scale 2 --run-name faults_x2

# 설정 파일로 (experiments/ 예시)
python tools/gen_world.py --config experiments/faults_x2.json --seed 3

# 예전 실행과 같은 설정으로 다시 만들기 (WMS로 돌린 실행은 로그 폴더에 scenario.json 사본이 있음)
python tools/gen_world.py --config logs/<실행>/scenario.json
```

- 설정 파일은 옵션 이름을 키로 쓰는 JSON입니다(`fault_scale`, `fault-scale` 둘 다 됨). 모르는 키가 있으면 오류로 멈춥니다.
- **명령 인자가 설정 파일보다 우선합니다.** 위 예에서 시드만 3으로 바꿔 쓰는 식입니다. 다만 `--no-faults`, `--plain` 같은 켜기 옵션은 설정 파일에서 켜면 명령 인자로 끌 수 없습니다.
- `scenario.json`의 `generator` 항목에 만들 때 쓴 설정이 모두 남습니다. 이 파일을 `--config`로 주면 같은 시나리오가 다시 만들어집니다.

## 2. 옵션

### 주문과 시간 (실제 데이터 재생)

| 옵션 | 기본값 | 뜻 |
| --- | --- | --- |
| `--date` | 2023-03-09 | 재생할 날짜(`data/orders_<날짜>.json`). 2023-03-22는 피크일 |
| `--duration` | 3600 | 에피소드 길이(초) |
| `--time-scale` | 0 | 실제 1초당 시뮬레이션 초. 0이면 그날 웨이브가 `release-frac` 안에 들어오도록 자동 결정 |
| `--release-frac` | 0.6 | 웨이브 릴리스를 에피소드의 앞 몇 %에 펼칠지 |
| `--warmup` | 30 | 첫 웨이브 전 대기 시간(초) |
| `--wave-interval` | 90 | 웨이브 릴리스 최소 간격(초) |
| `--order-sample` | 1.0 | 남길 주문 비율(주문 단위, 시드와 무관하게 고정) |
| `--max-line-qty` | 6 | 주문 한 줄의 최대 수량 |

`--time-scale`을 0으로 두면 `--duration`을 바꿀 때 시간 배율도 함께 바뀝니다(예: 120초면 182배). 길이만 바꿔 같은 흐름을 보려면 `--time-scale`을 고정하세요.

### 입고량과 초기 재고

| 옵션 | 기본값 | 뜻 |
| --- | --- | --- |
| `--inbound-scale` | 1.0 | SKU별 입고 상자 수 배율. 줄이면 먼저 필요한 것부터 남기고, 늘리면 추가분이 안전재고처럼 중간에 도착 |
| `--initial-cover` | 0.5 | 그날 수요 중 시작 시 선반에 있는 비율. 낮추면 입고에 더 의존 |
| `--initial-fill` | 0.6 | 시작 시 선반 칸 최대 사용률 |
| `--safety-stock` | 0.15 | 수요 대비 추가 입고 비율 |
| `--truck-cartons` | 12 | 트럭당 상자 수 (입고가 얼마나 몰려 오는지) |
| `--truck-first`, `--truck-min-gap` | 60, 120 | 첫 트럭 도착 시각, 트럭 간 최소 간격(초) |
| `--inbound-lead` | 240 | 상자가 수요보다 얼마나 먼저 도착해야 하는지(초) |
| `--stock-layout` | abc | 초기 재고 배치: `abc`(등급 규칙) 또는 `dataset`(실제 보관 위치, [dataset.md](dataset.md)) |

### 오류율 (S1~S3)

| 옵션 | 기본값 | 시나리오 | 뜻 |
| --- | --- | --- | --- |
| `--place-error-p` | 0.05 | S1 | 선반에 놓을 때 옆 칸에 놓이는 비율 |
| `--worker-move-per-h` | 8 | S1 | 작업자가 기록 없이 상자를 옮기는 횟수(시간당) |
| `--worker-move-max-slots` | 3 | S1 | 옮기는 거리(칸) |
| `--worker-remove-per-h` | 3 | S2 | 작업자가 기록 없이 상자를 가져가는 횟수(시간당) |
| `--dest-taken-p` | 0.1 | S3 | 운반 중 목적지가 차 버리는 비율 |
| `--scanner-fail-p` | 0.03 | 입고 | 스캐너가 라벨을 못 읽는 비율 |
| `--fault-scale` | 1.0 | 전체 | 위 비율·횟수 전체에 곱하는 배율(이동 거리 제외) |
| `--no-faults` | 꺼짐 | 전체 | 모든 오류를 0으로 |

### 로봇과 환경

| 옵션 | 기본값 | 뜻 |
| --- | --- | --- |
| `--seed` | 0 | 난수 시드 |
| `--patrols`, `--transports` | 2, 6 | 순찰·운반 로봇 수 |
| `--layout` | base | `base`(160칸) 또는 `large`(480칸) |
| `--patrol-sweep` | 꺼짐 | 쉬는 순찰 로봇이 선반을 계속 돌며 관측 |
| `--plain` | 꺼짐 | 장식 없는 월드 (배치 실험용, 약 3배 빠름) |
| `--run-name` | 없음 | 로그 폴더 이름에 붙는 실험 이름 |

## 3. 로그 폴더

`logs/<날짜>_<WES>_seed<시드>_<faults|nofaults>[_sweep][_<run-name>]_<batch|gui>/`

오류율이나 입고량만 바꾼 실행은 `--run-name`을 주지 않으면 폴더 이름이 같아서 앞 실행을 덮어씁니다. `WES_MODULE=wes_runner`로 돌리면 같은 폴더에 WMS 로그(`wms_log.jsonl`)와 그 실행이 쓴 `scenario.json` 사본도 생깁니다.

- 사본이 있으니 나중에 저장소의 `scenario.json`이 바뀌어도 `replay_wms.py`로 재생할 수 있습니다.
- 같은 설정으로 다시 만들 수도 있습니다(`--config`).
- 단, 이번 변경 전에 만든 시나리오에는 `generator` 항목이 없어 `--config`로 재현할 수 없습니다.

## 4. 재현성

확인한 것:

- 같은 설정과 시드로 두 번 만들면 `scenario.json`과 월드 파일이 바이트 단위로 같습니다. `--config <scenario.json>`으로 다시 만들어도 같습니다.
- 같은 `scenario.json`으로 두 번 실행하면 결과가 같습니다(교착 재현 실행에서 같은 시각·같은 위치로 확인).
- **주문 릴리스(`ORDER_RELEASED`)는 정책과 오류 설정이 달라도 같습니다.** standin, WMS 버전, 오류 끔 실행의 주문 이벤트가 시각과 내용까지 모두 같았습니다. `tools/compare_runs.py`의 `order releases vs 1st run` 줄로 확인할 수 있습니다.

실행마다 달라질 수 있는 것:

| 항목 | 이유 |
| --- | --- |
| 입고 트럭 도착 시각 | 계획된 도착 시각이 있지만, 앞 트럭이 벨트가 차서 하역을 못 끝내면 다음 트럭이 기다립니다. 정책이 적치를 빨리 하느냐에 따라 달라집니다 (`truck arrivals vs 1st run`) |
| 작업자가 건드리는 상자 | 사건이 생기는 시각은 시드로 고정되지만, 어떤 상자를 옮기거나 가져갈지는 그때 창고 상태에 따라 정해집니다 |
| S3 발생 작업 | 작업 ID와 목적지 칸으로 정해집니다([wms.md](wms.md) 12절) |

시드가 바꾸는 것: 입고 트럭 시각의 흔들림, 반품 상자 종류, 작업자 사건 시각, 오류 추첨. 주문 웨이브(실제 데이터)는 시드와 상관없이 같습니다.

정책을 비교할 때는 같은 설정 파일로 시드를 여러 개 돌려 평균을 보세요.

## 5. 예시 설정 (`experiments/`)

| 파일 | 내용 | 결과 |
| --- | --- | --- |
| `base.json` | 기본 (2023-03-09, 1시간, 순찰 2·운반 6, plain) | 트럭 9대 / 108상자 |
| `no_faults.json` | 오류 끔 | |
| `faults_x2.json` | 오류율 2배 | |
| `inbound_x1.5.json` | 입고량 1.5배 | 트럭 14대 / 164상자 |
| `dataset_layout.json` | 실제 보관 위치 기반 초기 재고 | |

여러 시드를 차례로 돌리는 예입니다(PowerShell, `webots/` 폴더에서).

```powershell
$env:QUIT_AT_END = "1"; $env:WES_MODULE = "wes_runner"; $env:WES_SCHEDULER = "sched_standin"
foreach ($seed in 0..2) {
    python tools/gen_world.py --config experiments/base.json --seed $seed
    & "<Webots 설치 경로>\msys64\mingw64\bin\webots.exe" --batch --mode=fast --minimize --no-rendering --stdout --stderr worlds/warehouse.wbt
}
python tools/compare_runs.py logs/2023-03-09_wms-standin_seed0_faults_batch logs/2023-03-09_wms-standin_seed1_faults_batch logs/2023-03-09_wms-standin_seed2_faults_batch
```

`gen_world.py`는 저장소에 있는 `scenario.json`, 월드, 라벨 그림을 덮어씁니다. 실험 뒤 커밋할 때는 이 파일들이 섞이지 않게 주의하세요.
