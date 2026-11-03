"""Turn the footwear order-picking dataset into simulator inputs (webots/data/).

Source: de Assis, de Paula Ferreira & Ouhimmou, "Order Picking Dataset from a
Warehouse of a Footwear Manufacturing Company", Mendeley Data V1,
doi:10.17632/pf2w725pw3.1 (CC BY 4.0). Only Customer_Order.csv and
Product.csv are used.

Outputs
  skus.json            the top-N SKUs (reference + US size) by ordered units
  orders_<date>.json   that day's waves, restricted to those SKUs

A wave is released when its last order was created. Each order line keeps its
customer; customers are spread over destination regions by a hash, because the
dataset has no addresses. Everything in these files is real data except the
region assignment (marked in the file).

Usage: python tools/prep_dataset.py [--top-sku 60] [--dates 2023-03-09,2023-03-22]
"""
import argparse
import collections
import csv
import json
import zlib
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SRC = ROOT.parent / "Order Picking Dataset from a Warehouse of a Footwear Manufacturing Company"


def size_us(raw):
    """'105' in Customer_Order means 10.5."""
    v = float(raw.strip().replace(",", "."))
    return v / 10 if v >= 20 else v


def sku_key(ref, size):
    return f"{ref}-{size:g}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", type=Path, default=DEFAULT_SRC)
    ap.add_argument("--out", type=Path, default=ROOT / "data")
    ap.add_argument("--top-sku", type=int, default=60)
    ap.add_argument("--dates", default="2023-03-09,2023-03-22", help="typical day, busiest day")
    ap.add_argument("--regions", type=int, default=4)
    args = ap.parse_args()

    with open(args.src / "Product.csv", encoding="utf-8-sig", newline="") as f:
        abc = {r["Reference"].strip(): r["ABCCOD"].strip() for r in csv.DictReader(f, delimiter=";")}

    lines = []
    with open(args.src / "Customer_Order.csv", encoding="utf-8-sig", newline="") as f:
        for r in csv.DictReader(f, delimiter=";"):
            if not r["Size (US)"].strip():
                continue
            ref = r["Reference"].strip()
            lines.append({
                "sku": sku_key(ref, size_us(r["Size (US)"])),
                "ref": ref,
                "qty": int(float(r["quantity (units)"])),
                "order": r["orderNumber"].strip(),
                "customer": r["codCustomer"].strip(),
                "wave": r["waveNumber"].strip(),
                "created": datetime.strptime(r["creationDate"].strip(), "%d/%m/%Y %H:%M"),
            })

    units = collections.Counter()
    for ln in lines:
        units[ln["sku"]] += ln["qty"]
    total = sum(units.values())
    top = [s for s, _ in units.most_common(args.top_sku)]
    keep = set(top)
    ref_of = {ln["sku"]: ln["ref"] for ln in lines}
    skus = [{"sku": s, "ref": ref_of[s], "size_us": float(s.rsplit("-", 1)[1]), "abc": abc.get(ref_of[s], "C"),
             "units": units[s], "share": round(units[s] / total, 5)} for s in top]

    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "skus.json").write_text(json.dumps({
        "source": "Customer_Order.csv, Product.csv (doi:10.17632/pf2w725pw3.1, CC BY 4.0)",
        "top_sku": args.top_sku,
        "share_of_all_units": round(sum(units[s] for s in top) / total, 4),
        "skus": skus,
    }, indent=1), encoding="utf-8")
    print(f"skus.json: top {len(top)} SKUs ({len({s['ref'] for s in skus})} references) "
          f"= {sum(units[s] for s in top) / total:.1%} of ordered units")

    regions = [f"R{k + 1}" for k in range(args.regions)]
    for day in args.dates.split(","):
        d = datetime.strptime(day, "%Y-%m-%d").date()
        todays = [ln for ln in lines if ln["created"].date() == d]
        waves = collections.defaultdict(list)
        for ln in todays:
            waves[ln["wave"]].append(ln)
        out, dropped = [], 0
        for w, wl in waves.items():
            release = max(ln["created"] for ln in wl)
            kept = [ln for ln in wl if ln["sku"] in keep]
            dropped += sum(ln["qty"] for ln in wl if ln["sku"] not in keep)
            if not kept:
                continue
            out.append({
                "wave": w,
                "release_s": release.hour * 3600 + release.minute * 60,
                "lines": [{"order": ln["order"], "customer": ln["customer"],
                           "region": regions[zlib.crc32(ln["customer"].encode()) % len(regions)],
                           "sku": ln["sku"], "qty": ln["qty"]} for ln in kept],
            })
        out.sort(key=lambda w: (w["release_s"], w["wave"]))
        kept_units = sum(ln["qty"] for w in out for ln in w["lines"])
        (args.out / f"orders_{day}.json").write_text(json.dumps({
            "date": day,
            "source": "real (Customer_Order.csv), filtered to skus.json",
            "synthetic_fields": ["region"],
            "regions": regions,
            "waves_total": len(waves),
            "units_kept": kept_units,
            "units_dropped": dropped,
            "waves": out,
        }, indent=1), encoding="utf-8")
        span = (out[-1]["release_s"] - out[0]["release_s"]) / 3600 if out else 0
        print(f"orders_{day}.json: {len(out)}/{len(waves)} waves, {kept_units} units kept, {dropped} dropped, "
              f"releases over {span:.1f} h")


if __name__ == "__main__":
    main()
