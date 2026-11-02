# 산업 작업공간 다중 로봇 시뮬레이터

캡스톤 디자인 팀 **404 Context Not Found**의 프로젝트입니다.

Webots로 물류센터를 시뮬레이션하고, 서버가 믿는 재고·작업 정보와 실제 상태가 달라질 때 로봇이 어떻게 소통해야 실수를 줄일 수 있는지 연구합니다.

## 현재 상태

지금 구현된 내용과 앞으로 만들 내용은 다음과 같습니다.

| 구분 | 내용 |
| --- | --- |
| 구현됨 | 20×14m 창고, 순찰·분류·운반 로봇, A* 경로 계획, 충돌 회피, 이상 상황 주입, 통신 채널 모델, 통신 정책 비교 |
| 진행 예정 | 장부와 실제 상태 분리, 중앙 WMS/WES, 출고 흐름, 벨트 입고, 라벨 판독 실패, 로봇 재시작 |

현재 코드는 로봇끼리 작업을 조율하는 구조입니다. 중앙 WMS/WES 구조로 전환할 예정이며, 기존 방식은 비교군으로 남깁니다.

## 폴더 구조

```text
webots/
├── controllers/
│   ├── agent/agent.py            로봇 컨트롤러
│   └── supervisor/supervisor.py  창고 상태, 통신 채널, 로그
├── lib/nav.py                    점유 격자, A* 경로 계획
├── protos/SimpleBot.proto        로봇 모델
├── tools/
│   ├── gen_world.py              월드와 scenario.json 생성
│   └── measure/                  Raspberry Pi 측정 스크립트와 기록
├── worlds/warehouse.wbt          생성된 Webots 월드
└── scenario.json                 생성된 실험 설정
```

## 준비

- Webots R2025a
- Python 3.11

Webots에서 Python 컨트롤러를 실행할 수 있도록 Python 경로를 설정해 주세요.

## 실행

명령은 `webots` 폴더에서 실행합니다. 아래 예시는 Windows PowerShell 기준입니다.

### 1. 월드 생성

```powershell
python tools/gen_world.py --patrols 3 --recoveries 8 --transports 6 --seed 0 --policy proposed
```

| 인자 | 기본값 | 뜻 |
| --- | --- | --- |
| `--patrols` | 3 | 순찰 로봇 수 |
| `--recoveries` | 8 | 분류 로봇 수 |
| `--transports` | 6 | 운반 로봇 수 |
| `--seed` | 0 | 난수 시드 |
| `--policy` | `proposed` | 통신 정책: `full`, `none`, `pull`, `cache`, `proposed` |

`gen_world.py`는 `worlds/warehouse.wbt`와 `scenario.json`을 다시 만듭니다.

### 2. 화면으로 실행

Webots에서 `worlds/warehouse.wbt`를 열고 시뮬레이션을 시작합니다.

### 3. 배치 실험

```powershell
$env:QUIT_AT_END = "1"
$env:POLICY = "proposed"
$env:BG_UTIL = "0.6"
& "<Webots 설치 경로>\msys64\mingw64\bin\webots.exe" --batch --mode=fast --minimize --no-rendering --stdout --stderr worlds/warehouse.wbt
```

| 환경 변수 | 뜻 |
| --- | --- |
| `QUIT_AT_END` | `1`이면 실험이 끝난 뒤 Webots를 종료 |
| `POLICY` | 월드에 지정된 통신 정책을 덮어씀 |
| `BG_UTIL` | 통신 채널의 배경 부하(예: `0.6`은 60%) |

## 실험 결과

실행 결과는 `webots/logs/`에 저장됩니다. 이 폴더는 Git에 올리지 않습니다.

| 파일 | 내용 |
| --- | --- |
| `summary_*.json` | 실험 요약 |
| `comm_*.csv` | 메시지별 통신 기록 |
| `accuracy_*.csv` | 시간별 정확도 |
| `anomalies_*.csv` | 이상 상황 기록 |

## 데이터셋

주문 패턴 참고 자료로 아래 데이터셋을 사용할 예정입니다. 저장소에는 포함하지 않았으며, 아직 시뮬레이터에 연결되어 있지 않습니다.

- de Assis, R. F., de Paula Ferreira, W., & Ouhimmou, M. (2024). *Order Picking Dataset from a Warehouse of a Footwear Manufacturing Company*. Mendeley Data, V1.
- DOI: [10.17632/pf2w725pw3.1](https://doi.org/10.17632/pf2w725pw3.1)
- 라이선스: CC BY 4.0
