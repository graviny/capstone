# 데이터셋 분석: 신발 제조사 물류창고 피킹 데이터

## 1. 출처

| 항목 | 내용 |
| --- | --- |
| 이름 | Order Picking Dataset from a Warehouse of a Footwear Manufacturing Company |
| 저자 | de Assis, de Paula Ferreira, Ouhimmou |
| 공개처 | Mendeley Data V1, DOI [10.17632/pf2w725pw3.1](https://doi.org/10.17632/pf2w725pw3.1) |
| 라이선스 | CC BY 4.0 (출처를 밝히면 사용·수정 가능) |
| 저장소 위치 | `Order Picking Dataset from a Warehouse of a Footwear Manufacturing Company/` (원본 그대로) |

수치는 모두 이 폴더의 파일을 직접 집계한 값입니다.

## 2. 파일 구성

| 파일 | 크기 | 내용 | 시뮬레이터 사용 |
| --- | --- | --- | --- |
| `Customer_Order.csv` | 122,370줄 | 고객 주문 줄(고객, 주문, 상품, 사이즈, 수량, 생성 시각, 웨이브, 작업자) | **사용**: 출고 주문, SKU 선정 |
| `Product.csv` | 208줄 | 상품(reference)별 ABC 등급, 구역 | **사용**: ABC 등급 |
| `Picking_Wave.csv` | 215,192줄 | 웨이브별 피킹 기록(1줄 = 1켤레), 피킹한 위치 | **사용**: 실제 보관 위치 (`prep_locations.py`) |
| `Storage_Location.csv` | 2,292줄 | 보관 위치 코드별 좌표(x, y, 층) | **사용**: 실제 보관 위치 좌표 |
| `Class_Based_Storage.csv` | 2,341곳 | ABC 기준 보관 배치안 | 안 씀 (아래 4절) |
| `Dedicated_Storage.csv` | 2,340곳 | 상품별 전용 보관 배치안 | 안 씀 |
| `Hybrid_Storage.csv` | 2,340곳 | 혼합 보관 배치안 | 안 씀 |
| `Random_Storage.csv` | 2,292곳 | 무작위 보관 배치안 | 안 씀 |
| `Support_Points_Navigation.csv` | 44줄 | 통로 이동 기준점 | 거리 계산의 가정에만 사용 |
| `Full layout.dwg`, `Layout_Z1.0~Z4.0.pdf/.svg` | – | 전체 도면, 층별 배치도 | 안 씀 |
| `Order_Picking_dataset.ipynb` | – | 층별 배치도를 그리는 코드(별도 Excel 파일을 읽음, 데이터셋에 없음) | 안 씀 |
| `Link Colab.txt` | 0 바이트 | 빈 파일 | – |

## 3. 파일별 내용

### Customer_Order.csv (`;` 구분)

| 컬럼 | 내용 |
| --- | --- |
| `codCustomer` | 고객 코드 (588명). 뒤에 공백이 붙어 있어 정리 필요 |
| `orderNumber` | 주문 번호 (32,634건). 주문 하나는 고객 한 명 |
| `orderToCollect` | 주문 안에서 줄의 순번으로 보임 (1이 가장 많음) |
| `Reference` | 상품 코드 (208종, 모두 `Product.csv`에 있음) |
| `Size (US)` | 사이즈. `105` = 10.5처럼 소수점이 빠진 값이 섞여 있음 (20 이상이면 10으로 나눔) |
| `quantity (units)` | 수량 (켤레). 평균 1.86, 최대 120 |
| `creationDate` | 주문 생성 시각 (`일/월/년 시:분`) |
| `waveNumber` | 피킹 웨이브 (9,784개) |
| `operator` | 작업자 (24명) |

- **기간:** 2023-01-05 ~ 2023-10-19 중 178일, 총 228,195켤레.
- **하루 물량:** 평균 1,282켤레. 가장 바쁜 날은 2023-10-02(3,028줄)와 2023-03-22(2,736줄).
- **시뮬레이터가 쓰는 날:**
  - 2023-03-09(평상시): 371줄, 주문 130건, 웨이브 48개, 1,078켤레
  - 2023-03-22(피크): 2,736줄, 주문 605건, 웨이브 251개, 5,911켤레
- **주문과 웨이브:** 주문 하나는 평균 3.7줄(최대 408줄)이고, 웨이브 하나에는 평균 4.1건(최대 24건)이 들어가요. 3,598건(11%)은 웨이브 2~3개에 나뉘어 있어요.
- **등급별 물량:** A 89,593켤레(39%), B 66,560(29%), C 72,042(32%).
- **품질 메모:**
  - 수량 0인 줄 33개, 사이즈가 빈 줄 16개가 있어요.
  - 사이즈 16 이상(16~30.5)인 줄이 2,785개 있어요. 다른 사이즈 체계로 보이지만 데이터만으로는 확인할 수 없어요. 상위 60개 SKU 중 `PY5UPB`가 23.5~28이에요.

### Product.csv (`;` 구분)

`Reference`, `ABCCOD`(A 44종, B 70종, C 94종), `Sector`(전부 `PF`) 세 컬럼이에요.

### Picking_Wave.csv (`;` 구분)

| 컬럼 | 내용 |
| --- | --- |
| `waveNumber` | 웨이브 (9,707개, 모두 `Customer_Order.csv`에 있음) |
| `reference`, `Size (US)` | 상품, 사이즈 |
| `quantityToPick (units)` | 항상 1 (한 줄 = 한 켤레) |
| `locations` | 그 켤레를 집은 위치 (1,221곳). 뒤에 공백이 붙어 있음 |
| `operator` | 작업자 (22명) |

- **주문과의 관계:** 웨이브별 켤레 수가 `Customer_Order.csv`와 같은 웨이브는 9,707개 중 7,394개(76%)예요. 나머지는 피킹 기록이 더 많거나 적어요.
- **좌표 없는 위치:** 피킹 위치 중 22곳(23,609줄)은 `Storage_Location.csv`에 좌표가 없어요. 대부분 `RC-01`(22,979줄)인데, 통로 기준점과 같은 이름이라 오른쪽 통로 앞의 바닥 보관 구역으로 추정돼요.
- **위치가 시간에 따라 바뀜:** 같은 상품·사이즈라도 여러 위치에서 피킹돼요. 2,454개 상품·사이즈 중 1,245개는 5곳 이상에서 나왔어요.

### Storage_Location.csv (`,` 구분)

`originalLocation`(위치 코드), `position`(`"x, y, z"` 문자열), `x`, `y`, `z` 컬럼이에요.

- **좌표 범위:** x 86~666, y 0~1,440, z 1~4.
- **좌표 단위:** README는 m라고 하지만 y가 1,440까지라 실제 단위는 확인이 필요해요.
- **위치 코드 구조:** `구역-열-(층)(칸)`. 예: `H-10-22` = H 구역, 10열, 2층, 2번 칸.
  - 셋째 자리의 첫 숫자는 z와 2,291/2,292곳에서 일치해요.
  - 구역 A~R은 y가 작은 쪽(앞)에서 큰 쪽(뒤) 순서예요.
  - 열 번호는 x 방향이고, 한 열에는 위치가 보통 8곳(층 × 칸) 있어요.
- **층(z):** 1층 846곳, 2층 846곳, 3층 301곳, 4층 299곳. 층별 배치도가 `Layout_Z1~Z4`예요.

### 보관 배치안 4종 (Class_Based / Dedicated / Hybrid / Random_Storage.csv)

- **형식:** 위치마다 칸 18개에 `상품;값`이 들어 있어요. README는 값이 수량이라고 하지만, 실제로는 **사이즈**예요(4.0~15.5). `Dedicated`는 한 위치의 18칸이 모두 같은 `상품;사이즈`예요.
- **분류 코드:** `Class_Based`는 ABC(C 1,036 / A 723 / B 582곳), `Dedicated`·`Hybrid`는 XYZ(Z 814 / Y 662 / C 600 / X 264곳)예요. `Random`에는 분류 코드가 없어요.
- **실제 피킹 위치와의 일치율:** 피킹 기록의 (상품, 사이즈, 위치)가 배치안과 맞는 비율이에요.

  | 배치안 | 일치하는 피킹 줄 |
  | --- | --- |
  | Class_Based | 1,210 / 215,192 (0.6%) |
  | Dedicated | 76 (0.04%) |
  | Hybrid | 5,244 (2.4%) |
  | Random | 34,665 (16.1%) |

  일치율이 모두 낮아서, **실제 운영 상태가 아니라 비교용으로 만든 배치안**으로 판단했어요. 그래서 실제 보관 위치는 `Picking_Wave.csv`에서 가져와요.

### Support_Points_Navigation.csv (`;` 구분)

통로 기준점 44개예요. 왼쪽 통로 `LC-01~17`(x=66), 가운데 `CC-01~17`(x=403), 오른쪽 `RC-08~17`(x=686)가 있고, 모두 1층(z=1)이에요. 01번이 y=-29, 그 뒤로 y가 커져요. `RC-01~07`은 기준점 목록에는 없고 피킹 위치 이름으로만 나와요.

## 4. 파일 사이의 관계

```text
Product.Reference ─┬─ Customer_Order.Reference
                   └─ Picking_Wave.reference
Customer_Order.waveNumber ── Picking_Wave.waveNumber      (피킹 기록의 날짜는 이 연결로 얻음)
Picking_Wave.locations ── Storage_Location.originalLocation ── *_Storage.Location
Support_Points (LC / CC / RC 통로) ── Storage_Location 좌표와 같은 좌표계
```

`Picking_Wave.csv`에는 날짜가 없어요. 웨이브 번호로 `Customer_Order.csv`의 생성 날짜를 붙여야 해요.

## 5. 시뮬레이터에서 쓰는 방식

| 시뮬레이터 데이터 | 원본 | 처리 | 실제 / 합성 |
| --- | --- | --- | --- |
| SKU 60개 (`data/skus.json`) | Customer_Order, Product | 상품·사이즈별 주문 수량 상위 60개(상품 14종). 전체 수량의 28.8% | 실제 |
| 출고 웨이브 (`data/orders_<날짜>.json`) | Customer_Order | 그날 웨이브를 상위 SKU로 거름, 시뮬레이터에서 한 줄 최대 6켤레, 시간 압축 | 실제 |
| 배송 지역 R1~R4 | 고객 코드 | 해시로 배정 (주소가 없음) | 합성 |
| 실제 보관 위치 (`data/locations_<날짜>.json`) | Picking_Wave, Customer_Order, Storage_Location | SKU별로 그날(없으면 ±30일, 없으면 전체 기간) 가장 많이 피킹된 위치 | 실제 |
| 초기 재고 수량 | 그날 수요 | SKU별 수요의 절반, 선반 칸의 최대 60% | 합성 (실제 수요 기반) |
| 초기 재고 배치 | 아래 참고 | `--stock-layout abc`(기본) 또는 `dataset` | 선택 |
| 입고 트럭과 ASN | 그날 수요 | 부족분 + 안전재고 15%를 12상자씩, 수요보다 240초 먼저 도착 | 합성 |
| 창고 칸 배치, 로봇 | – | 시뮬레이터 설계 (선반 160칸) | 합성 |

### 초기 재고 배치: `--stock-layout`

| 값 | 배치 규칙 |
| --- | --- |
| `abc` (기본) | A등급, 같은 등급 안에서는 수요가 큰 SKU부터 출고 쪽 가까운 칸에 배치 |
| `dataset` | 실제 창고에서 **앞쪽에 가깝게 보관됐던 SKU부터** 출고 쪽 가까운 칸에 배치 |

`dataset`의 "앞쪽에서의 거리"는 이렇게 정했어요.
- 데이터에 출하장 위치가 없어서, 통로 기준점 01번이 있는 앞쪽 통로(y=-29)를 출하 쪽으로 **가정**했어요.
- 거리 = (y + 29) + 가장 가까운 통로(x = 66, 403, 686)까지의 가로 거리. 층은 거리에 넣지 않고 같은 거리일 때 순서만 정해요.

2023-03-09 기준으로 확인한 점이에요.
- **위치의 근거:** 상위 60개 SKU 중 24개는 그날, 25개는 ±30일, 10개는 전체 기간의 피킹 위치로 정해졌어요. 1개(`PY5UPB-25`)는 좌표 있는 위치가 없어서 같은 상품 다른 사이즈의 거리를 썼어요.
- **같은 상품의 사이즈는 실제로도 붙어 있어요.** 예를 들어 `8N10W9`는 H-06·08·10열, `WRRW1W`는 I-05·07·09열이에요. `dataset` 배치에서도 같은 상품끼리 이웃 칸에 놓여요.
- **실제 배치는 ABC 규칙과 달라요.** 실제로 가장 앞에 있던 것은 C등급 `SMMRK3`와 B등급 `8D9SAW`(y=572)이고, A등급 `WRRW1W`·`I1KDJ0`은 그 뒤(y=720~778)에 있었어요. 그래서 `dataset` 배치는 `abc`와 96칸 중 95칸이 달라요(같은 수량, 같은 주문·입고).

## 6. 다시 만들기

명령은 `webots/` 폴더에서 실행합니다. 원본 폴더는 저장소 루트에 있어야 해요.

```powershell
python tools/prep_dataset.py --top-sku 60 --dates 2023-03-09,2023-03-22   # skus.json, orders_<날짜>.json
python tools/prep_locations.py --dates 2023-03-09,2023-03-22               # locations_<날짜>.json
python tools/gen_world.py --date 2023-03-09 --stock-layout dataset          # 실제 보관 순서로 초기 재고
```
