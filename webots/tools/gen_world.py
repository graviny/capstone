"""Generate the warehouse world (worlds/warehouse.wbt) and scenario.json.

Layout (24 x 14 m)
  west  inbound: truck dock -> belt -> fixed scanner -> 6 buffer zones; unread
        cartons are diverted to 2 exception slots
  mid   4 rack rows, 8 faces x 20 slots (one carton per slot)
  east  outbound: one staging lane (8 slots) and one truck dock per region

Work comes from data/ (tools/prep_dataset.py): one real day of waves, time-
compressed into the episode. Inbound trucks are synthetic: each carton is put on
a truck that arrives before the demand it covers (plus safety stock), and the
initial stock covers part of the day's demand. Real and synthetic inputs are
labelled in scenario.json.

Faults that make S1-S3 happen are set in scenario.json "faults" (all on by
default, --no-faults turns them off for the fault-free experiment). Their rates
are options too (--place-error-p ... --scanner-fail-p, all scaled by --fault-scale).
Every default fault is caused by a person (worker moves, removals, returns);
robots do not make mistakes and the inbound scanner reads every label. The robot
placement error and scanner failures stay available but are off (0) by default.

Settings can come from a JSON file (--config): option names as keys, or a
scenario.json, whose "generator" entry records every setting it was made with.
Options given on the command line win. Same settings and seed -> same scenario.

Usage: python tools/gen_world.py [--date 2023-03-09] [--duration 3600] [--patrols 2] [--transports 6] [--seed 0]
       python tools/gen_world.py --config experiments/base.json [--seed 1]
"""
import argparse
import json
import math
import random
import re
import stat
import sys
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_textures  # noqa: E402
import world_visuals as wv  # noqa: E402

FLOOR_X, FLOOR_Y = 24.0, 14.0
ROW_YS = [-4.5, -1.5, 1.5, 4.5]    # rack row centers
RACK_X0, RACK_X1 = -3.0, 7.0
ROW_HALF_DEPTH = 0.3
RACK_TOP = 0.12
PANEL_HEIGHT = 0.7                 # center panel blocks the view into the next aisle
SLOT_PITCH = 0.5
SLOTS_PER_SIDE = 20
BOX_SIZE = (0.30, 0.20, 0.14)      # shoe carton
APPROACH_OFFSET = 0.65             # where a robot stops to handle a rack slot
PATROL_LANE = 0.95                 # inspection passes run this far from the rack center
CAMERA_FOV = 1.08                  # Pi camera, 62 deg

# Inbound: belt along y = BELT_Y, cartons flow east (+x).
BELT_Y, BELT_X0, BELT_X1 = 3.0, -10.5, -5.3
BELT_TOP = 0.12
GATE_X = -9.5                      # fixed scanner
# Buffer zones B0 (east end) .. B5 on a staging table along the south side of the belt: a
# pusher moves a carton off the belt into a free zone, so staged cartons never block the belt.
ZONE_XS = [-5.55 - 0.5 * k for k in range(6)]
ZONE_Y = BELT_Y - 0.55
EXC_POS = [(-9.6, 2.15), (-9.1, 2.15)]          # exception slots X0, X1
STAGE_APPROACH = 0.75

# Outbound: per region a staging lane (2 x 4 floor slots) and a truck dock.
LANE_YC = [4.5, 1.5, -1.5, -4.5]
LANE_XS = [9.35, 9.75]
LANE_DY = [-0.6, -0.2, 0.2, 0.6]
LANE_APPROACH_X = 8.75
OUT_TRUCK_X = 12.05                # rear of a docked truck, just outside the east wall

TRANSPORT_HOMES = [(-10.5 + 0.8 * k, -6.3) for k in range(8)]
PATROL_HOMES = [(-6.0, 5.8), (-7.2, 5.8), (-8.4, 5.8)]
# West / east shift of the inbound / outbound areas (0 for the base layout).
WX, EX = 0.0, 0.0
ROBOT_COLORS = {"patrol": (0.9, 0.6, 0.1), "transport": (0.2, 0.4, 0.9)}

PALETTE = [(0.8, 0.2, 0.15), (0.15, 0.35, 0.8), (0.2, 0.6, 0.25), (0.55, 0.38, 0.2), (0.85, 0.7, 0.15),
           (0.55, 0.25, 0.65), (0.1, 0.6, 0.65), (0.9, 0.45, 0.6), (0.35, 0.35, 0.35), (0.6, 0.75, 0.3),
           (0.95, 0.55, 0.2), (0.25, 0.25, 0.6), (0.7, 0.55, 0.45), (0.45, 0.15, 0.2), (0.3, 0.5, 0.45),
           (0.75, 0.75, 0.75)]


def use_large_layout():
    """--layout large: 34 x 20 m (about 2x the floor), 6 rack rows x 2 faces x 40 slots;
    inbound moved 5 m west, outbound 5 m east, room for 14 transports and 5 patrols."""
    global FLOOR_X, FLOOR_Y, ROW_YS, RACK_X0, RACK_X1, SLOTS_PER_SIDE, BELT_X0, BELT_X1, GATE_X, ZONE_XS
    global EXC_POS, LANE_YC, LANE_XS, LANE_APPROACH_X, OUT_TRUCK_X, TRANSPORT_HOMES, PATROL_HOMES, WX, EX
    WX, EX = -5.0, 5.0
    FLOOR_X, FLOOR_Y = 34.0, 20.0
    ROW_YS = [-7.5, -4.5, -1.5, 1.5, 4.5, 7.5]
    RACK_X0, RACK_X1 = -3.0 + WX, 7.0 + EX
    SLOTS_PER_SIDE = 40
    BELT_X0, BELT_X1, GATE_X = BELT_X0 + WX, BELT_X1 + WX, GATE_X + WX
    ZONE_XS = [x + WX for x in ZONE_XS]
    EXC_POS = [(x + WX, y) for x, y in EXC_POS]
    LANE_YC = [4.5, 1.5, -1.5, -4.5, 7.5, -7.5]      # the first regions get the middle docks
    LANE_XS = [x + EX for x in LANE_XS]
    LANE_APPROACH_X, OUT_TRUCK_X = LANE_APPROACH_X + EX, OUT_TRUCK_X + EX
    TRANSPORT_HOMES = [(-10.5 + WX + 0.8 * k, -FLOOR_Y / 2 + 0.7) for k in range(14)]
    PATROL_HOMES = [(-6.0 + WX - 1.2 * k, FLOOR_Y / 2 - 1.2) for k in range(5)]


def look_at(eye, target):
    """Axis-angle rotation of a camera at eye looking at target (x forward, z up)."""
    fx, fy, fz = (t - e for t, e in zip(target, eye))
    n = math.sqrt(fx * fx + fy * fy + fz * fz)
    fx, fy, fz = fx / n, fy / n, fz / n
    lx, ly, lz = -fy, fx, 0.0                         # left = up x forward
    n = math.hypot(lx, ly)
    lx, ly = lx / n, ly / n
    ux, uy, uz = fy * lz - fz * ly, fz * lx - fx * lz, fx * ly - fy * lx
    r = [[fx, lx, ux], [fy, ly, uy], [fz, lz, uz]]
    angle = math.acos(max(-1.0, min(1.0, (r[0][0] + r[1][1] + r[2][2] - 1) / 2)))
    s = 2 * math.sin(angle)
    axis = ((r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s)
    return (*axis, angle)


def unit_hash(*parts):
    return zlib.crc32("-".join(str(p) for p in parts).encode()) / 2 ** 32


def build_orders(args, skus):
    """Real waves of one day, sampled and time-compressed into the episode."""
    data = json.loads((ROOT / "data" / f"orders_{args.date}.json").read_text(encoding="utf-8"))
    known = {s["sku"] for s in skus}
    waves, capped = [], 0
    for w in data["waves"]:
        lines = []
        for ln in w["lines"]:
            if ln["sku"] not in known or unit_hash(args.date, ln["order"]) >= args.order_sample:
                continue
            q = min(ln["qty"], args.max_line_qty)
            capped += ln["qty"] - q
            lines.append({**ln, "qty": q})
        if lines:
            waves.append({"wave": w["wave"], "release_s": w["release_s"], "lines": lines})
    if not waves:
        raise SystemExit("no waves left after sampling")
    t0 = waves[0]["release_s"]
    span = waves[-1]["release_s"] - t0
    scale = args.time_scale or max(1.0, span / (args.release_frac * args.duration))
    # Waves are released one at a time on a fixed cadence (wave picking), never before
    # their orders exist (creation time, compressed).
    prev = -1e9
    for w in waves:
        created = args.warmup + (w["release_s"] - t0) / scale
        w["created_t"] = round(created, 1)
        w["release_t"] = round(max(created, prev + args.wave_interval), 1)
        prev = w["release_t"]
    meta = {"date": args.date, "source": data["source"], "synthetic_fields": data["synthetic_fields"],
            "time_scale": round(scale, 2), "wave_interval_s": args.wave_interval, "order_sample": args.order_sample,
            "max_line_qty": args.max_line_qty, "units_capped": capped,
            "units": sum(ln["qty"] for w in waves for ln in w["lines"]), "waves": len(waves)}
    return waves, data["regions"], meta


def plan_stock(args, skus, waves, rack_slots, rng):
    """Initial stock (on the shelves at t=0) and synthetic inbound trucks."""
    demand = []                                     # (t, sku) per unit, in release order
    for w in waves:
        for ln in w["lines"]:
            demand += [(w["release_t"], ln["sku"])] * ln["qty"]
    per_sku = {}
    for _, s in demand:
        per_sku[s] = per_sku.get(s, 0) + 1

    # Every SKU has a carton on the shelf; SKUs with demand get part of it up front.
    init = {s["sku"]: 1 for s in skus}
    for s, d in per_sku.items():
        init[s] = max(1, math.ceil(args.initial_cover * d))
    usable = [s for k, s in enumerate(rack_slots) if k % 4 != 3]   # every 4th slot stays free
    cap = int(args.initial_fill * len(rack_slots))
    while sum(init.values()) > min(cap, len(usable)):
        s = max(init, key=lambda k: init[k] - per_sku.get(k, 0) * 0.01)
        init[s] -= 1
        if init[s] == 0:
            del init[s]

    # Fastest movers (class A first) nearest the outbound side, or (--stock-layout dataset)
    # the SKUs really stored closest to the front of the warehouse (tools/prep_locations.py).
    rank = {"A": 0, "B": 1, "C": 2}
    share = {s["sku"]: s["share"] for s in skus}
    abc = {s["sku"]: s["abc"] for s in skus}
    order = sorted(init, key=lambda s: (rank[abc[s]], -share[s]))
    if args.stock_layout == "dataset":
        path = ROOT / "data" / f"locations_{args.date}.json"
        if not path.exists():
            raise SystemExit(f"{path.name} not found: run tools/prep_locations.py --dates {args.date}")
        real = {v["sku"]: v["front_dist"] for v in json.loads(path.read_text(encoding="utf-8"))["skus"]
                if v["front_dist"] is not None}
        order = sorted(order, key=lambda s: real.get(s, float("inf")))   # stable: ABC order breaks ties
    near_out = sorted(usable, key=lambda s: (-s["pos"][0], abs(s["pos"][1])))
    stock, k = [], 0
    for s in order:
        for _ in range(init[s]):
            stock.append({"slot": near_out[k]["id"], "sku": s})
            k += 1

    # Inbound: demand the initial stock does not cover, plus safety stock.
    covered = dict(init)
    need = []
    for t, s in demand:
        if covered.get(s, 0) > 0:
            covered[s] -= 1
        else:
            need.append((t - args.inbound_lead, s))
    for s, d in per_sku.items():
        need += [(args.duration * 0.5, s)] * math.ceil(args.safety_stock * d)
    need.sort()
    if args.inbound_scale != 1.0:
        # More or fewer inbound cartons per SKU: the earliest-due ones are kept, extra ones
        # arrive like safety stock (due mid-episode).
        per = {}
        for _, s in need:
            per[s] = per.get(s, 0) + 1
        want = {s: max(0, round(n * args.inbound_scale)) for s, n in per.items()}
        kept, have = [], {}
        for due, s in need:
            if have.get(s, 0) < want[s]:
                kept.append((due, s))
                have[s] = have.get(s, 0) + 1
        for s in sorted(want):
            kept += [(args.duration * 0.5, s)] * (want[s] - have.get(s, 0))
        need = sorted(kept)
    n_trucks = max(1, math.ceil(len(need) / args.truck_cartons))
    last = max(args.truck_first + 1, min(args.duration * args.release_frac, need[-1][0] if need else 0))
    gap = max(args.truck_min_gap, (last - args.truck_first) / max(1, n_trucks - 1))
    times = [round(args.truck_first + k * gap + rng.uniform(-0.15, 0.15) * gap * (k > 0), 1) for k in range(n_trucks)]
    while times[-1] > args.duration * 0.85 and len(times) > 1:
        times.pop()
    loads = [[] for _ in times]
    for due, s in need:
        # Latest truck that still arrives in time; if none can, the earliest one with room.
        ok = [k for k, tt in enumerate(times) if tt <= due and len(loads[k]) < args.truck_cartons]
        if ok:
            k = ok[-1]
        else:
            room = [k for k in range(len(times)) if len(loads[k]) < args.truck_cartons]
            if not room:
                times.append(times[-1] + args.truck_min_gap)
                loads.append([])
                room = [len(times) - 1]
            k = room[0]
        loads[k].append(s)
    trucks = []
    lpn = 0
    for k, (tt, load) in enumerate(zip(times, loads)):
        if not load:
            continue
        rng.shuffle(load)
        cartons = []
        for s in load:
            lpn += 1
            cartons.append({"lpn": f"IN{lpn:04d}", "sku": s})
        trucks.append({"truck": f"T{k + 1:02d}", "arrive_t": tt, "cartons": cartons})
    return stock, trucks, per_sku


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--date", default="2023-03-09")
    ap.add_argument("--duration", type=float, default=3600.0, help="episode length, s")
    ap.add_argument("--patrols", type=int, default=2)
    ap.add_argument("--transports", type=int, default=6)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--order-sample", type=float, default=1.0, help="share of orders kept (whole orders)")
    ap.add_argument("--max-line-qty", type=int, default=6)
    ap.add_argument("--time-scale", type=float, default=0.0, help="real s per sim s (0: fit releases)")
    ap.add_argument("--release-frac", type=float, default=0.6, help="releases spread over this share")
    ap.add_argument("--warmup", type=float, default=30.0)
    ap.add_argument("--wave-interval", type=float, default=90.0, help="min gap between wave releases, s")
    ap.add_argument("--initial-cover", type=float, default=0.5, help="share of demand on the shelf at t=0")
    ap.add_argument("--initial-fill", type=float, default=0.6, help="max share of rack slots filled at t=0")
    ap.add_argument("--safety-stock", type=float, default=0.15)
    ap.add_argument("--truck-cartons", type=int, default=12)
    ap.add_argument("--truck-first", type=float, default=60.0)
    ap.add_argument("--truck-min-gap", type=float, default=120.0)
    ap.add_argument("--inbound-lead", type=float, default=240.0, help="carton should arrive this long before demand")
    ap.add_argument("--no-faults", action="store_true")
    ap.add_argument("--plain", action="store_true", help="no decoration (faster batch experiments)")
    ap.add_argument("--layout", choices=["base", "large"], default="base",
                    help="base: 24 x 14 m, 160 rack slots; large: 34 x 20 m, 480 rack slots")
    ap.add_argument("--patrol-sweep", action="store_true",
                    help="idle patrols sweep the rack faces (periodic cycle count)")
    ap.add_argument("--stock-layout", choices=["abc", "dataset"], default="abc",
                    help="initial stock: abc = class A nearest the outbound side; dataset = in the order of "
                         "the SKUs' real storage locations (data/locations_<date>.json)")
    ap.add_argument("--inbound-scale", type=float, default=1.0,
                    help="inbound cartons per SKU x this (1: the day's uncovered demand + safety stock)")
    # Fault rates (scenario.json "faults"); --fault-scale multiplies all of them.
    # Off: faults are caused by people only (team decision); kept for later comparisons.
    ap.add_argument("--place-error-p", type=float, default=0.0,
                    help="S1 robot error: share of rack placements off by a slot (default off)")
    ap.add_argument("--worker-move-per-h", type=float, default=8.0, help="S1: unrecorded moves per hour")
    ap.add_argument("--worker-move-max-slots", type=int, default=3, help="S1: how far a worker moves a carton")
    ap.add_argument("--worker-remove-per-h", type=float, default=3.0, help="S2: unrecorded removals per hour")
    ap.add_argument("--dest-taken-p", type=float, default=0.1, help="S3: share of putaways whose slot gets taken")
    # TODO: inbound label read failures are left out for now (default 0); the supervisor,
    # the WMS (VERIFY of exception slots) and the stand-in policies still handle them.
    ap.add_argument("--scanner-fail-p", type=float, default=0.0,
                    help="inbound: share of labels not read (default off, TODO)")
    ap.add_argument("--fault-scale", type=float, default=1.0, help="multiplies every fault rate above")
    ap.add_argument("--run-name", default="",
                    help="added to the log folder name (logs/<date>_<WES>_seed<n>_<faults>_<run-name>_<mode>)")
    ap.add_argument("--config", type=Path,
                    help="JSON with option values (option names, - or _), or a scenario.json to repeat its settings")
    pre, _ = ap.parse_known_args()
    if pre.config:
        conf = json.loads(pre.config.read_text(encoding="utf-8"))
        conf = conf.get("generator", conf)            # a scenario.json carries its settings here
        known = {a.dest for a in ap._actions}
        conf = {k.replace("-", "_"): v for k, v in conf.items() if k not in ("config",)}
        unknown = sorted(set(conf) - known)
        if unknown:
            raise SystemExit(f"{pre.config}: unknown settings {unknown}")
        ap.set_defaults(**conf)                       # the command line still wins
    args = ap.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9_.-]*", args.run_name):
        raise SystemExit("--run-name: letters, digits, _ . - only")
    if args.layout == "large":
        use_large_layout()
    if not 0 <= args.patrols <= len(PATROL_HOMES) or not 1 <= args.transports <= len(TRANSPORT_HOMES):
        raise SystemExit(f"patrols 0..{len(PATROL_HOMES)}, transports 1..{len(TRANSPORT_HOMES)}")
    rng = random.Random(args.seed)

    sku_data = json.loads((ROOT / "data" / "skus.json").read_text(encoding="utf-8"))
    skus = sku_data["skus"]
    refs = sorted({s["ref"] for s in skus})
    color_of_ref = {r: PALETTE[k % len(PALETTE)] for k, r in enumerate(refs)}
    for s in skus:
        s["color"] = color_of_ref[s["ref"]]       # same model, other size: looks alike

    # ---- slots ----
    slots, rack_slots = [], []
    for ri, ry in enumerate(ROW_YS):
        for side, sy, ay in (("s", ry - 0.17, ry - APPROACH_OFFSET), ("n", ry + 0.17, ry + APPROACH_OFFSET)):
            for si in range(SLOTS_PER_SIDE):
                sx = RACK_X0 + SLOT_PITCH * (si + 0.5)
                s = {"id": f"r{ri}{side}{si:02d}", "kind": "rack", "face": f"r{ri}{side}",
                     "pos": [round(sx, 3), round(sy, 3)], "approach": [round(sx, 3), round(ay, 3)], "z": RACK_TOP}
                slots.append(s)
                rack_slots.append(s)
    for k, zx in enumerate(ZONE_XS):
        slots.append({"id": f"B{k}", "kind": "buffer", "pos": [zx, ZONE_Y],
                      "approach": [zx, round(ZONE_Y - STAGE_APPROACH, 3)], "z": BELT_TOP})
    for k, (ex, ey) in enumerate(EXC_POS):
        slots.append({"id": f"X{k}", "kind": "exception", "pos": [ex, ey],
                      "approach": [ex, round(ey - STAGE_APPROACH, 3)], "z": BELT_TOP})

    waves, regions, order_meta = build_orders(args, skus)
    if len(regions) > len(LANE_YC):
        raise SystemExit(f"{len(regions)} regions in the data, the layout has {len(LANE_YC)} outbound lanes")
    lanes = {}
    for k, region in enumerate(regions):
        yc = LANE_YC[k]
        ids = []
        for lx in LANE_XS:
            for dy in LANE_DY:
                sid = f"L{region}-{len(ids)}"
                slots.append({"id": sid, "kind": "outbound", "lane": region, "pos": [lx, round(yc + dy, 3)],
                              "approach": [LANE_APPROACH_X, round(yc + dy, 3)], "z": 0.0})
                ids.append(sid)
        lanes[region] = {"y": yc, "slots": ids, "truck_pos": [OUT_TRUCK_X, yc]}

    stock, trucks, per_sku = plan_stock(args, skus, waves, rack_slots, rng)
    # Unrecorded cartons a worker may put into a slot (returns): S3 and S1 sources.
    returns = [{"lpn": f"RT{k + 1:03d}", "sku": rng.choices([s["sku"] for s in skus],
                                                          weights=[s["share"] for s in skus])[0]}
               for k in range(40)]

    # ---- world ----
    hx, hy = FLOOR_X / 2, FLOOR_Y / 2
    make_textures.make_all(ROOT / "textures" / "gen", [s["id"] for s in rack_slots], regions)
    looks = wv.Looks()
    # --plain leaves out the decoration (lights, markings, rack structure, rollers, signs, cages):
    # same behavior, faster batch runs.
    deco = not args.plain
    deco_rng = random.Random(1000 + args.seed)
    slot_of_id = {s["id"]: s for s in slots}
    k_view = FLOOR_X / 24.0
    eye, target = (1.0, -13.0 * k_view, 20.0 * k_view), (1.0, 0.0, 0.0)
    parts = [
        "#VRML_SIM R2025a utf8\n",
        'EXTERNPROTO "../protos/SimpleBot.proto"\n\n',
        "WorldInfo {\n  basicTimeStep 32\n"
        '  contactProperties [ ContactProperties { material2 "slippery" coulombFriction [ 0 ] } ]\n}\n',
        f"Viewpoint {{\n  orientation {wv.vec(*look_at(eye, target))}\n  position {wv.vec(*eye)}\n}}\n",
        "Background { skyColor [ 0.62 0.72 0.82 ] }\n",
        "DirectionalLight { direction -0.3 0.4 -1 intensity 2.2 castShadows TRUE }\n",
        "DirectionalLight { direction 0.4 -0.3 -1 intensity 0.9 }\n",
        wv.floor(looks, FLOOR_X, FLOOR_Y),
        wv.yard(looks),
        wv.walls(looks, hx, hy, west_doors=[(BELT_Y - 1.0, BELT_Y + 1.0)],
                 east_doors=[(yc - 0.9, yc + 0.9) for yc in LANE_YC[:len(regions)]]),
    ]
    if deco:
        if args.layout == "base":
            light_xs, light_ys = [-9.0, -5.0, -1.0, 3.0, 7.0, 10.5], [-6.0, -3.0, 0.0, 3.0, 6.0]
        else:
            light_xs = [-hx + 3.0 + 4.0 * k for k in range(int((2 * hx - 4.5) // 4) + 1)]
            light_ys = [-hy + 1.0 + 3.0 * k for k in range(int((2 * hy - 2.0) // 3) + 1)]
        parts.append(wv.ceiling_lights(looks, light_xs, light_ys))

    # Floor markings: aisle edges, AMR lanes, parking bays, dock safety zones, lanes.
    white = looks.color("PAINT_W", (0.92, 0.92, 0.9), rough=0.9)
    yellow = looks.color("PAINT_Y", (0.95, 0.78, 0.1), rough=0.9)
    green = looks.color("PAINT_G", (0.25, 0.65, 0.3), rough=0.9)
    red = looks.color("PAINT_R", (0.85, 0.2, 0.15), rough=0.9)
    for ry in ROW_YS:
        for side in (-1, 1):
            if deco:
                parts.append(wv.line(looks, white, RACK_X0 - 0.1, ry + side * 0.42, RACK_X1 + 0.1, ry + side * 0.42))
    for x in (-4.2 + WX, 8.1 + EX):
        if deco:
            parts.append(wv.dashed(looks, yellow, x, -hy + 0.4, x, hy - 0.4))
    for hxy in TRANSPORT_HOMES + PATROL_HOMES:
        if deco:
            parts.append(wv.outline(looks, white, hxy[0], hxy[1], 0.55, 0.55))
    if deco:
        parts.append(wv.hatch(looks, -hx + 0.02, -10.65 + WX, BELT_Y - 0.95, BELT_Y + 0.95))
    for yc in LANE_YC[:len(regions)]:
        if deco:
            parts.append(wv.hatch(looks, 10.45 + EX, hx - 0.02, yc - 0.8, yc + 0.8))
    if deco:
        parts.append(wv.outline(looks, red, -9.35 + WX, 2.15, 1.3, 0.75, w=0.06))

    # Racks: base and back panel keep their solids; uprights, beams, upper pallets and labels are looks.
    rack_cx, rack_len = (RACK_X0 + RACK_X1) / 2, RACK_X1 - RACK_X0
    for ri, ry in enumerate(ROW_YS):
        base = (rack_len, 2 * ROW_HALF_DEPTH, RACK_TOP)
        panel = (rack_len, 0.04, PANEL_HEIGHT)
        parts.append(f"Solid {{ name \"rack{ri}_base\" translation {wv.vec(rack_cx, ry, RACK_TOP / 2)} children [ "
                     f"{wv.shape(wv.rack_base_app(looks), wv.box(*base))} ] boundingObject {wv.box(*base)} }}\n")
        parts.append(f"Solid {{ name \"rack{ri}_panel\" translation {wv.vec(rack_cx, ry, PANEL_HEIGHT / 2)} children [ "
                     f"{wv.shape(wv.rack_panel_app(looks), wv.box(*panel))} ] boundingObject {wv.box(*panel)} }}\n")
        row_slots = [s for s in rack_slots if s["face"].startswith(f"r{ri}")]
        if deco:
            parts.append(wv.rack(looks, ri, ry, RACK_X0, RACK_X1, ROW_HALF_DEPTH, RACK_TOP, row_slots, deco_rng))

    # Inbound: roller conveyor, scanner gate, pushers, staging and exception tables, signs.
    stage_x0, stage_x1 = ZONE_XS[-1] - 0.25, ZONE_XS[0] + 0.25
    for name, cx, cy, sx in (("belt", (BELT_X0 + BELT_X1) / 2, BELT_Y, BELT_X1 - BELT_X0),
                             ("staging_table", (stage_x0 + stage_x1) / 2, ZONE_Y, stage_x1 - stage_x0)):
        parts.append(f"Solid {{ name \"{name}\" translation {wv.vec(cx, cy, BELT_TOP / 2)} "
                     f"boundingObject {wv.box(sx, 0.5, BELT_TOP)} }}\n")
    if deco:
        parts.append(wv.roller_bed(looks, BELT_X0, BELT_X1, BELT_Y, 0.5, BELT_TOP))
    if deco:
        parts.append(wv.roller_bed(looks, stage_x0, stage_x1, ZONE_Y, 0.44, BELT_TOP, pitch=0.1))
    for zx in ZONE_XS:
        if deco:
            parts.append(wv.outline(looks, green, zx, ZONE_Y, 0.44, 0.4, w=0.03, z=BELT_TOP + 0.003))
        if deco:
            parts.append(wv.pusher(looks, zx, BELT_Y + 0.34))
    if deco:
        parts.append(wv.scanner_gate(looks, GATE_X, BELT_Y))
    exc_app = looks.color("EXCTABLE", (0.75, 0.2, 0.18), rough=0.6)
    parts.append(f"Solid {{ name \"exception_table\" translation {wv.vec(-9.35 + WX, 2.15, BELT_TOP / 2)} children [ "
                 f"{wv.shape(exc_app, wv.box(1.0, 0.5, BELT_TOP))} ] boundingObject {wv.box(1.0, 0.5, BELT_TOP)} }}\n")
    if deco:
        parts.append(wv.sign(looks, "inbound", (-8.0 + WX, BELT_Y + 0.9, 2.5), size=(1.6, 0.8), facing="y"))
    if deco:
        parts.append(wv.sign(looks, "exception", (-9.35 + WX, 1.55, 1.6), size=(0.9, 0.45), facing="y"))
    if deco:
        parts.append(wv.sign(looks, "parking", (-7.7 + WX, -hy + 0.2, 1.6), size=(1.2, 0.6), facing="-y"))
    parts.append(wv.truck(looks, "TRUCK_IN", -hx - 0.05, BELT_Y, -1, (0.75, 0.15, 0.12)))

    # Outbound: lane marks, roll cages, region signs, trucks.
    cab_colors = [(0.15, 0.35, 0.7), (0.1, 0.5, 0.3), (0.55, 0.3, 0.6), (0.8, 0.45, 0.1)]
    for k, (region, lane) in enumerate(lanes.items()):
        yc = lane["y"]
        if deco:
            parts.append(wv.outline(looks, white, sum(LANE_XS) / 2, yc, 0.9, 1.7, w=0.05))
        for sid in lane["slots"]:
            sp = slot_of_id[sid]["pos"]
            if deco:
                parts.append(wv.outline(looks, white, sp[0], sp[1], 0.34, 0.34, w=0.02))
        for dy in (-0.45, 0.45):
            if deco:
                parts.append(wv.roll_cage(looks, 10.75 + EX, yc + dy))
        if deco:
            parts.append(wv.sign(looks, region, (sum(LANE_XS) / 2, yc, 2.6), size=(1.2, 0.6), facing="x"))
        parts.append(wv.truck(looks, f"TRUCK_{region}", hx + 0.05, yc, 1, cab_colors[k % len(cab_colors)]))

    # Workers who cause the unrecorded changes (shown by the supervisor where it happens).
    for k in range(2):
        parts.append(wv.worker(looks, f"WORKER_{k}", (0, -40 - 2 * k, -5)))

    # Cartons: shelf stock at its slot, inbound and return cartons parked out of sight.
    color = {s["sku"]: s["color"] for s in skus}
    ref_of = {s["sku"]: s["ref"] for s in skus}
    box_z = BOX_SIZE[2] / 2
    cartons = []

    def carton(lpn, sku, pos):
        parts.append(wv.carton(looks, lpn, pos, wv.carton_size(ref_of[sku], BOX_SIZE), sku, ref_of[sku], color[sku],
                               plain=args.plain))

    for k, st in enumerate(stock):
        lpn = f"ST{k + 1:04d}"
        s = slot_of_id[st["slot"]]
        cartons.append({"lpn": lpn, "sku": st["sku"]})
        carton(lpn, st["sku"], (s["pos"][0], s["pos"][1], s["z"] + box_z))
        st["lpn"] = lpn
    hidden = [c for tr in trucks for c in tr["cartons"]] + returns
    for k, c in enumerate(hidden):
        cartons.append(c)
        carton(c["lpn"], c["sku"], (100 + (k % 40) * 0.4, 100 + (k // 40) * 0.4, box_z))

    roster = ([("patrol", PATROL_HOMES[k], 0.0) for k in range(args.patrols)]
              + [("transport", TRANSPORT_HOMES[k], math.pi / 2) for k in range(args.transports)])
    faces = []
    for ri, ry in enumerate(ROW_YS):
        faces.append({"id": f"r{ri}s", "start": [RACK_X0 - 0.6, ry - PATROL_LANE], "end": [RACK_X1 + 0.6, ry - PATROL_LANE]})
        faces.append({"id": f"r{ri}n", "start": [RACK_X1 + 0.6, ry + PATROL_LANE], "end": [RACK_X0 - 0.6, ry + PATROL_LANE]})
    robots = []
    for i, (role, start, yaw) in enumerate(roster):
        robots.append({"id": i, "def": f"R{i}", "name": f"robot_{i}", "role": role, "home": list(start)})
        c = ROBOT_COLORS[role]
        parts.append(
            f"DEF R{i} SimpleBot {{\n  translation {start[0]} {start[1]} 0\n  rotation 0 0 1 {yaw:.4f}\n"
            f"  name \"robot_{i}\"\n  controller \"agent\"\n  controllerArgs [ \"--id\" \"{i}\" ]\n"
            f"  color {c[0]} {c[1]} {c[2]}\n  receiverChannel {100 + i}\n  cameraFov {CAMERA_FOV}\n"
            f"  detailed {'FALSE' if args.plain else 'TRUE'}\n}}\n"
        )
    # The supervisor also carries two cameras for snapshots (SNAPSHOT_AT): top-down (north up)
    # and a perspective view from the south-west.
    persp_eye, persp_target = (-6.0 * k_view, -12.0 * k_view, 17.0 * k_view), (1.0, 0.5, 0.0)
    parts.append(
        "Robot {\n  name \"supervisor\"\n  supervisor TRUE\n  controller \"supervisor\"\n"
        "  children [ Emitter { name \"emitter\" channel 100 range -1 } Receiver { name \"receiver\" channel 0 }\n"
        f"    Camera {{ name \"overview\" translation 0 0 {22 * k_view:g} rotation -0.57735 0.57735 0.57735 2.0944\n"
        "      width 1200 height 700 fieldOfView 1.0 }\n"
        f"    Camera {{ name \"overview_persp\" translation {wv.vec(*persp_eye)} "
        f"rotation {wv.vec(*look_at(persp_eye, persp_target))}\n"
        "      width 1200 height 700 fieldOfView 1.1 } ]\n}\n"
    )

    fs = args.fault_scale
    faults = {
        # S1 robot execution error: a carton lands in the next free slot of the face instead of
        # the target. Off by default (faults come from people only).
        "place_error_p": min(1.0, args.place_error_p * fs),
        # S1/S2 unobserved changes: a worker moves (nearby) or removes a carton without recording it.
        "worker_move_per_h": args.worker_move_per_h * fs,
        "worker_move_max_slots": args.worker_move_max_slots,
        "worker_remove_per_h": args.worker_remove_per_h * fs,
        # S3: while a robot carries a carton to a rack slot, a worker puts a returned carton there.
        "dest_taken_p": min(1.0, args.dest_taken_p * fs),
        # Inbound: the fixed scanner cannot read a label -> carton diverted to an exception slot.
        # Off by default (TODO).
        "scanner_fail_p": min(1.0, args.scanner_fail_p * fs),
    }
    if args.no_faults:
        faults = {k: 0 if not k.endswith("max_slots") else v for k, v in faults.items()}

    scenario = {
        "seed": args.seed,
        "run_name": args.run_name,
        # Every setting this file was made with: gen_world.py --config scenario.json repeats it.
        "generator": {k: v for k, v in vars(args).items() if k != "config"},
        "world": {"x_min": -hx, "x_max": hx, "y_min": -hy, "y_max": hy},
        "racks": [{"y": y, "x_min": RACK_X0, "x_max": RACK_X1, "half_depth": ROW_HALF_DEPTH} for y in ROW_YS],
        "obstacles": [
            {"name": "belt", "x_min": BELT_X0 - 0.1, "x_max": BELT_X1 + 0.05, "y_min": BELT_Y - 0.3, "y_max": BELT_Y + 0.3},
            {"name": "staging_table", "x_min": ZONE_XS[-1] - 0.3, "x_max": ZONE_XS[0] + 0.3,
             "y_min": ZONE_Y - 0.3, "y_max": BELT_Y},
            {"name": "exception_table", "x_min": -9.9 + WX, "x_max": -8.8 + WX, "y_min": 1.9, "y_max": 2.4},
            {"name": "inbound_dock", "x_min": -hx, "x_max": -10.6 + WX, "y_min": 2.0, "y_max": 4.0},
        ] + [{"name": f"lane_{r}", "x_min": 9.15 + EX, "x_max": 9.95 + EX, "y_min": l["y"] - 0.8, "y_max": l["y"] + 0.8}
             for r, l in lanes.items()]
          + [{"name": f"dock_{r}", "x_min": 10.4 + EX, "x_max": hx, "y_min": l["y"] - 0.8, "y_max": l["y"] + 0.8}
             for r, l in lanes.items()],
        "box_size": list(BOX_SIZE),
        "slots": slots,
        "skus": skus,
        "cartons": cartons,
        "initial_stock": stock,
        "initial_stock_layout": args.stock_layout,
        "robots": robots,
        "faces": faces,
        "belt": {"y": BELT_Y, "x_entry": BELT_X0 + 0.2, "x_end": ZONE_XS[0], "gate_x": GATE_X, "top": BELT_TOP,
                 "speed_mps": 0.25, "gap_m": 0.45, "zones": [f"B{k}" for k in range(len(ZONE_XS))],
                 "exception": [f"X{k}" for k in range(len(EXC_POS))], "unload_s": 3.0, "truck_pos": [-11.4 + WX, BELT_Y]},
        "inbound": {"source": "synthetic (covers the day's demand)", "trucks": trucks},
        "outbound": {
            "source": "real waves; regions synthetic",
            # Where outbound work is: the stand-in WES keeps fast movers near it.
            "center": [8.7 + EX, 0.0],
            **order_meta,
            "regions": regions,
            "lanes": lanes,
            # A region truck leaves when its lane holds dispatch_cartons, or at each cutoff with
            # whatever is staged; it is back after turnaround_s.
            "dispatch_cartons": 6,
            "cutoff_period_s": 900.0,
            "load_s": 15.0,
            "turnaround_s": 180.0,
            "waves": waves,
        },
        "returns": returns,
        "faults": faults,
        "episode": {"duration_s": args.duration},
        # Off by default: which slots to check is the WES's decision (VERIFY tasks).
        "patrol": {"idle_sweep": args.patrol_sweep},
        "perception": {"read_m": 1.2, "slot_match_m": 0.2, "carried_z": 0.35, "look_s": 1.0},
        "nav": {"speed_mps": 0.3, "wheel_radius": 0.033, "reach_m": 0.2, "goal_reach_m": 0.1,
                "avoid_dist_m": 0.6, "stop_dist_m": 0.3, "replan_after_s": 2.0, "act_s": 2.0, "reach_act_m": 1.3},
    }

    (ROOT / "worlds").mkdir(exist_ok=True)
    world = ROOT / "worlds" / "warehouse.wbt"
    # Kept read-only so closing Webots cannot save an older layout over it.
    if world.exists():
        world.chmod(stat.S_IWRITE | stat.S_IREAD)
    world.write_text("".join(parts), encoding="utf-8")
    world.chmod(stat.S_IREAD)
    (ROOT / "scenario.json").write_text(json.dumps(scenario, indent=1), encoding="utf-8")
    n_in = sum(len(t["cartons"]) for t in trucks)
    print(f"wrote warehouse.wbt: {args.patrols} patrol + {args.transports} transport robots; "
          f"{args.date}: {order_meta['waves']} waves, {order_meta['units']} units, time x{order_meta['time_scale']}; "
          f"stock {len(stock)}/{len(rack_slots)} slots, {len(trucks)} trucks / {n_in} cartons; "
          f"faults {'off' if args.no_faults else ('on' if fs == 1 else f'x{fs:g}')}, "
          f"patrol sweep {'on' if args.patrol_sweep else 'off'}, {'plain' if args.plain else 'decorated'}"
          + (f", run name {args.run_name}" if args.run_name else ""))


if __name__ == "__main__":
    main()
