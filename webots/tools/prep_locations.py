"""Where each simulated SKU was really stored: data/locations_<date>.json from the dataset.

The dataset's storage files (Class_Based / Dedicated / Hybrid / Random_Storage.csv) are
alternative storage assignments and agree with where the picks actually happened for at
most 16 % of the pick lines, so the real storage of a SKU is taken from Picking_Wave.csv
(the location each unit was picked from), dated through Customer_Order.csv (wave ->
creation date), and placed with Storage_Location.csv (x, y, floor).

For each SKU in data/skus.json: the location it was picked from most often on the date,
else within +-WINDOW_DAYS of it, else over the whole dataset; locations without
coordinates (e.g. RC-01) are skipped. A SKU with no located pick takes the distance of
its reference's other sizes.

front_dist is a proxy for the walk from the front of the warehouse: the dataset does not
say where orders leave, so the front cross aisle (support points LC-01 / CC-01 / RC-01,
y = -29) is taken as the dispatch side, and a picker walks up the nearest of the three
corridors (x = 66, 403, 686) and then across. Floors (z) are not weighted, only used to
break ties. Units are those of Storage_Location.csv.

gen_world.py --stock-layout dataset puts the initial stock in this order: the SKUs that
were really stored closest to the front go to the slots closest to the outbound side.

Usage: python tools/prep_locations.py [--dates 2023-03-09,2023-03-22]
"""
import argparse
import collections
import csv
import json
from datetime import datetime, timedelta
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SRC = ROOT.parent / "Order Picking Dataset from a Warehouse of a Footwear Manufacturing Company"
FRONT_Y = -29.0                      # support points LC-01, CC-01, RC-01
CORRIDORS_X = (66.0, 403.0, 686.0)   # LC, CC, RC
WINDOW_DAYS = 30


def size_us(raw):
    """'105' in the dataset means 10.5 (same rule as prep_dataset.py)."""
    v = float(raw.strip().replace(",", "."))
    return v / 10 if v >= 20 else v


def front_dist(x, y):
    return round((y - FRONT_Y) + min(abs(x - cx) for cx in CORRIDORS_X), 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", type=Path, default=DEFAULT_SRC)
    ap.add_argument("--out", type=Path, default=ROOT / "data")
    ap.add_argument("--dates", default="2023-03-09,2023-03-22")
    args = ap.parse_args()

    skus = json.loads((args.out / "skus.json").read_text(encoding="utf-8"))["skus"]
    wanted = {s["sku"] for s in skus}
    wave_date = {}
    with open(args.src / "Customer_Order.csv", encoding="utf-8-sig", newline="") as f:
        for r in csv.DictReader(f, delimiter=";"):
            wave_date.setdefault(r["waveNumber"].strip(),
                                 datetime.strptime(r["creationDate"].strip(), "%d/%m/%Y %H:%M").date())
    coord = {}
    with open(args.src / "Storage_Location.csv", encoding="utf-8-sig", newline="") as f:
        for r in csv.DictReader(f):
            coord[r["originalLocation"].strip()] = (int(r["x"]), int(r["y"]), int(r["z"]))
    picks = collections.defaultdict(list)          # sku -> [(date, location)]
    with open(args.src / "Picking_Wave.csv", encoding="utf-8-sig", newline="") as f:
        for r in csv.DictReader(f, delimiter=";"):
            sku = f"{r['reference'].strip()}-{size_us(r['Size (US)']):g}"
            if sku in wanted:
                picks[sku].append((wave_date.get(r["waveNumber"].strip()), r["locations"].strip()))

    for day in args.dates.split(","):
        d = datetime.strptime(day, "%Y-%m-%d").date()
        window = (d - timedelta(days=WINDOW_DAYS), d + timedelta(days=WINDOW_DAYS))
        out = {}
        for s in skus:
            sku = s["sku"]
            for basis, keep in (("day", lambda pd: pd == d), ("window", lambda pd: pd and window[0] <= pd <= window[1]),
                                ("all", lambda pd: True)):
                count = collections.Counter(loc for pd, loc in picks[sku] if keep(pd) and loc in coord)
                if count:
                    loc, n = count.most_common(1)[0]
                    x, y, z = coord[loc]
                    out[sku] = {"sku": sku, "location": loc, "x": x, "y": y, "floor": z, "picks": n,
                                "basis": basis, "front_dist": front_dist(x, y)}
                    break
        for s in skus:                             # no located pick: like the reference's other sizes
            if s["sku"] not in out:
                same = sorted(v["front_dist"] for k, v in out.items() if k.rsplit("-", 1)[0] == s["ref"])
                out[s["sku"]] = {"sku": s["sku"], "location": None, "x": None, "y": None, "floor": None, "picks": 0,
                                 "basis": "reference", "front_dist": same[len(same) // 2] if same else None}
        rows = sorted(out.values(), key=lambda v: (v["front_dist"] is None, v["front_dist"] or 0, v["floor"] or 0,
                                                   v["sku"]))
        (args.out / f"locations_{day}.json").write_text(json.dumps({
            "date": day,
            "source": "Picking_Wave.csv + Customer_Order.csv + Storage_Location.csv (doi:10.17632/pf2w725pw3.1)",
            "assumptions": {"front_y": FRONT_Y, "corridors_x": list(CORRIDORS_X), "window_days": WINDOW_DAYS,
                            "front_dist": "(y - front_y) + distance to the nearest corridor; floors not weighted"},
            "skus": rows,
        }, indent=1), encoding="utf-8")
        basis = collections.Counter(v["basis"] for v in rows)
        print(f"locations_{day}.json: {len(rows)} SKUs, basis {dict(basis)}")


if __name__ == "__main__":
    main()
