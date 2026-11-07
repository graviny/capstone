"""Supervisor: warehouse ground truth, inbound belt, outbound docks, fault injection, WES hookup, logs.

Ground truth (which carton is where) lives only here. The WES (stand-in by
default, WES_MODULE env var for another module) sees nothing but events:

  environment  ORDER_RELEASED, TRUCK_ARRIVED, TRUCK_DEPARTED, INBOUND_STAGED,
               INBOUND_READ_FAIL, OUTBOUND_TRUCK_ARRIVED, OUTBOUND_DEPARTED
  robots       TASK_COMPLETED, TASK_FAILED, ITEM_MISMATCH, SHORT_PICK,
               DESTINATION_OCCUPIED, OBSERVATION_RESULT

and answers with Decisions (EXECUTE, REROUTE, WAIT, CANCEL, REASSIGN) that go to
the robots. Robot messages are delivered instantly (no channel model).

Inbound: a truck unloads cartons onto the belt (unload_s apart); the fixed
scanner at gate_x reads each label; each carton is pushed off the belt into the
east-most free buffer zone ahead of it (zones sit on a staging table beside the
belt, so a staged carton does not block the belt). With no free zone ahead it
waits at the belt end and holds up the cartons behind it. If the scanner fails,
the carton is diverted to a free exception slot (or staged unread when both are taken).

Outbound: each region's truck leaves when its lane holds dispatch_cartons or at
every cutoff with whatever is staged, and is back after turnaround_s.

Faults (scenario.json "faults"), the causes of S1-S3, logged to truth.jsonl only:
  S1_PLACE_ERROR          a carton lands in the next free slot of the face
  S1_UNRECORDED_MOVE      a worker moves a carton a few slots, unrecorded
  S2_UNRECORDED_REMOVAL   a worker takes a carton away, unrecorded
  S3_DEST_TAKEN           while a robot carries a carton to a rack slot, a worker
                          puts a returned carton there
Placement errors and S3 draws are keyed by carton / task, so they repeat across
WES policies; worker events come from a seeded process.

Diagnostics also go to truth.jsonl: rejected picks / places (ACT_REJECTED),
ground-truth consistency problems (INVARIANT_VIOLATION, checked every 10 s) and
robots that drive to a slot without progress for STUCK_S (ROBOT_STUCK).
SNAPSHOT_AT="60,1800" saves a top-down (view_<t>.png) and a perspective view
(persp_<t>.png) at those sim times; SNAPSHOT_EYE="x,y,z,tx,ty,tz" moves the
perspective camera (rendering must be on, i.e. no --no-rendering). Worker figures appear for a few seconds in the
aisle where an unrecorded change (S1 move, S2 removal, S3 return) happens; they
are looks only and do not block the robots.

Logs (logs/<tag>/): events.jsonl (events + decisions), truth.jsonl, poses.csv (every 2 s),
timeseries.csv (every 10 s), summary.json. QUIT_AT_END=1 closes Webots at the end.
"""
import csv
import importlib
import json
import math
import os
import random
import shutil
import sys
import tempfile
import zlib
from pathlib import Path

from controller import Supervisor

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "lib"))
POSE_EVERY_STEPS = 4
CARRY_Z = 0.5
STATS_PERIOD_S = 10.0
PRINT_PERIOD_S = 60.0
DIAGNOSTICS = ("ACT_REJECTED", "INVARIANT_VIOLATION", "ROBOT_STUCK")   # logged to truth.jsonl, not faults
POSE_LOG_S = 2.0
PUSH_TOL = 0.25                    # half the buffer zone pitch
WORKER_SHOW_S = 8.0                # how long a worker figure stays where it changed something
STUCK_S, STUCK_M = 60.0, 0.3       # driving to a slot but less than STUCK_M of progress in STUCK_S


def look_at(eye, target):
    """Axis-angle rotation of a camera at eye looking at target (x forward, z up)."""
    fx, fy, fz = (t - e for t, e in zip(target, eye))
    n = math.sqrt(fx * fx + fy * fy + fz * fz)
    fx, fy, fz = fx / n, fy / n, fz / n
    lx, ly = -fy, fx
    n = math.hypot(lx, ly)
    lx, ly = lx / n, ly / n
    ux, uy, uz = -fz * ly, fz * lx, fx * ly - fy * lx
    r = [[fx, lx, ux], [fy, ly, uy], [fz, 0.0, uz]]
    angle = math.acos(max(-1.0, min(1.0, (r[0][0] + r[1][1] + r[2][2] - 1) / 2)))
    s = 2 * math.sin(angle)
    return ((r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s, angle)


def unit(*parts):
    return zlib.crc32("-".join(str(p) for p in parts).encode()) / 2 ** 32


def main():
    cfg = json.loads((ROOT / "scenario.json").read_text(encoding="utf-8"))
    seed, faults, belt_cfg, out_cfg = cfg["seed"], cfg["faults"], cfg["belt"], cfg["outbound"]
    wes = importlib.import_module(os.environ.get("WES_MODULE", "wes_standin")).WES(cfg)
    worker_rng = random.Random(seed + 101)

    sup = Supervisor()
    dt = int(sup.getBasicTimeStep())
    step_s = dt / 1000.0
    emitter = sup.getDevice("emitter")
    receiver = sup.getDevice("receiver")
    receiver.enable(dt)
    # SNAPSHOT_AT="60,1800": save a top-down view (logs/<tag>/view_<t>.png) at these times.
    snapshots = sorted(float(v) for v in os.environ.get("SNAPSHOT_AT", "").split(",") if v.strip())
    cams = {}
    if snapshots and os.environ.get("SNAPSHOT_EYE"):
        # SNAPSHOT_EYE="x,y,z,tx,ty,tz": move the perspective camera to look from x,y,z at tx,ty,tz.
        ex, ey, ez, tx, ty, tz = (float(v) for v in os.environ["SNAPSHOT_EYE"].split(","))
        children = sup.getSelf().getField("children")
        for k in range(children.getCount()):
            node = children.getMFNode(k)
            if node.getTypeName() == "Camera" and node.getField("name").getSFString() == "overview_persp":
                node.getField("translation").setSFVec3f([ex, ey, ez])
                node.getField("rotation").setSFRotation(list(look_at((ex, ey, ez), (tx, ty, tz))))
    if snapshots:
        for prefix, name in (("view", "overview"), ("persp", "overview_persp")):
            cam = sup.getDevice(name)
            if cam:
                cam.enable(dt)
                cams[prefix] = cam
    robots = {r["id"]: r for r in cfg["robots"]}
    rnodes = {r["id"]: sup.getFromDef(r["def"]) for r in cfg["robots"]}
    slots = {s["id"]: s for s in cfg["slots"]}
    half_h = cfg["box_size"][2] / 2

    # ---- ground truth ----
    sku_of = {c["lpn"]: c["sku"] for c in cfg["cartons"]}
    bnode = {lpn: sup.getFromDef(f"BOX_{lpn}") for lpn in sku_of}
    occupant = {s: None for s in slots}
    slot_of = {}
    carried = {}                     # lpn -> robot id
    for st in cfg["initial_stock"]:
        occupant[st["slot"]], slot_of[st["lpn"]] = st["lpn"], st["slot"]
    returns = [c["lpn"] for c in cfg["returns"]]
    lane_tag = {}                    # lpn on an outbound lane -> what the robot placed it for
    hidden_k = [0]

    def put(lpn, sid):
        s = slots[sid]
        bnode[lpn].getField("translation").setSFVec3f([s["pos"][0], s["pos"][1], s["z"] + half_h])
        occupant[sid], slot_of[lpn] = lpn, sid

    def take(lpn):
        sid = slot_of.pop(lpn, None)
        if sid:
            occupant[sid] = None

    def hide(lpn):
        take(lpn)
        k = hidden_k[0] = hidden_k[0] + 1
        bnode[lpn].getField("translation").setSFVec3f([200 + (k % 40) * 0.4, 200 + (k // 40) * 0.4, half_h])

    def gt_sku(sid):
        return sku_of[occupant[sid]] if occupant[sid] else None

    # ---- logs ----
    mode = "batch" if os.environ.get("QUIT_AT_END") == "1" else "gui"
    faults_on = any(v for k, v in faults.items() if not k.endswith("max_slots"))
    sweep = "_sweep" if cfg.get("patrol", {}).get("idle_sweep") else ""
    run = f"_{cfg['run_name']}" if cfg.get("run_name") else ""      # gen_world.py --run-name
    tag = f"{out_cfg['date']}_{wes.name}_seed{seed}_{'faults' if faults_on else 'nofaults'}{sweep}{run}_{mode}"
    log_dir = ROOT / "logs" / tag
    log_dir.mkdir(parents=True, exist_ok=True)
    ev_file = open(log_dir / "events.jsonl", "w", encoding="utf-8")
    truth_file = open(log_dir / "truth.jsonl", "w", encoding="utf-8")
    ts_file = open(log_dir / "timeseries.csv", "w", newline="", encoding="utf-8")
    ts = csv.writer(ts_file)
    ts.writerow(["t", "rack_cartons", "belt", "buffer", "exception", "staged_out", "units_released",
                 "units_shipped", "robots_busy", "record_mismatch"])
    pose_file = open(log_dir / "poses.csv", "w", newline="", encoding="utf-8")
    pose_log = csv.writer(pose_file)
    pose_log.writerow(["t", "robot", "x", "y", "task", "phase", "holding"])

    n_events = [0]
    counts = {}
    truth_counts = {}
    t_now = [0.0]

    def emit(ev):
        n_events[0] += 1
        ev = {"event_id": f"E{n_events[0]:06d}", "t": round(t_now[0], 2), **ev}
        counts[ev["type"]] = counts.get(ev["type"], 0) + 1
        ev_file.write(json.dumps(ev) + "\n")
        wes.on_event(ev)

    def truth(kind, **kw):
        truth_counts[kind] = truth_counts.get(kind, 0) + 1
        truth_file.write(json.dumps({"t": round(t_now[0], 2), "type": kind, **kw}) + "\n")
        print(f"[truth t={t_now[0]:.0f}] {kind} {kw}")

    def to_robot(rid, msg):
        emitter.setChannel(100 + rid)
        emitter.send(json.dumps(msg))

    # ---- orders ----
    waves = sorted(out_cfg["waves"], key=lambda w: w["release_t"])
    next_wave = 0
    orders = {}
    for w in waves:
        for ln in w["lines"]:
            o = orders.setdefault(ln["order"], {"units": 0, "shipped": 0, "released_t": w["release_t"],
                                               "done_t": None, "region": ln["region"]})
            o["units"] += ln["qty"]
    units_released = units_shipped = mis_shipped = 0

    # ---- inbound ----
    trucks = sorted(cfg["inbound"]["trucks"], key=lambda tr: tr["arrive_t"])
    next_truck, dock, unload_at = 0, None, 0.0
    truck_in = sup.getFromDef("TRUCK_IN")
    truck_in_pos = truck_in.getField("translation").getSFVec3f()
    zones = belt_cfg["zones"]                          # B0 (end of belt) first
    zone_x = {z: slots[z]["pos"][0] for z in zones}
    belt = []                                          # cartons on the belt: {lpn, x, scanned, read}
    staged_in = 0

    def show(node, pos, visible):
        node.getField("translation").setSFVec3f([pos[0], pos[1], pos[2] if visible else -5.0])

    show(truck_in, truck_in_pos, False)

    # ---- outbound ----
    lanes = out_cfg["lanes"]
    lane_state = {}
    for region, lane in lanes.items():
        node = sup.getFromDef(f"TRUCK_{region}")
        lane_state[region] = {"node": node, "pos": node.getField("translation").getSFVec3f(), "present": True,
                              "loading_until": None, "back_at": None, "next_cutoff": out_cfg["cutoff_period_s"]}
    missed_cutoffs = 0
    shipments = 0

    # ---- robots / tasks ----
    robot_state = {rid: {"task": None, "holding": None, "phase": None} for rid in robots}
    task_dest = {}                                     # task id -> destination (for S3)
    place_count = {}
    s3_pending = []                                    # (time, slot, task)
    dist_m = {rid: 0.0 for rid in robots}
    anchor = {}                                        # robot -> (time, x, y) of its last real progress
    stuck_reported = set()
    busy_s = {rid: 0.0 for rid in robots}
    last_pos = {}

    def handle_act(msg):
        rid, op, sid = msg["src"], msg["op"], msg["slot"]
        rx, ry, _ = rnodes[rid].getPosition()
        s = slots[sid]
        near = math.hypot(s["pos"][0] - rx, s["pos"][1] - ry) <= cfg["nav"]["reach_act_m"]
        held = next((l for l, r in carried.items() if r == rid), None)
        res = {"kind": "ACT_RESULT", "op": op, "slot": sid, "ok": False, "task_id": msg.get("task_id")}
        if op == "pick":
            lpn = occupant[sid]
            if near and lpn and held is None:
                take(lpn)
                carried[lpn] = rid
                res.update(ok=True, item=sku_of[lpn], lpn=lpn)
                dest = task_dest.get(msg.get("task_id"))
                if (dest and slots[dest]["kind"] == "rack"
                        and unit(seed, msg["task_id"], dest, "dest") < faults["dest_taken_p"]):
                    s3_pending.append((t_now[0] + 4.0, dest, msg["task_id"]))
            else:
                res["reason"] = "too_far" if not near else ("empty" if not lpn else "holding")
        elif op == "place":
            if held is None:
                res["reason"] = "not_holding"
            elif not near:
                res["reason"] = "too_far"
            elif occupant[sid] is not None:
                res["reason"] = "occupied"
            else:
                actual = sid
                if s["kind"] == "rack":
                    k = place_count[held] = place_count.get(held, 0) + 1
                    u = unit(seed, held, k, "place")
                    if u < faults["place_error_p"]:
                        idx = int(sid[3:])
                        order = (-1, 1) if u < faults["place_error_p"] / 2 else (1, -1)
                        side = [f"{sid[:3]}{idx + d:02d}" for d in order]
                        nb = next((n for n in side if n in occupant and occupant[n] is None), None)
                        if nb:
                            actual = nb
                            truth("S1_PLACE_ERROR", lpn=held, item=sku_of[held], intended=sid, actual=nb,
                                  robot=rid, task_id=msg.get("task_id"))
                del carried[held]
                put(held, actual)
                if s["kind"] == "outbound":
                    lane_tag[held] = {"order": msg.get("order"), "item": msg.get("item"), "region": s["lane"],
                                      "task_id": msg.get("task_id")}
                res.update(ok=True, item=sku_of[held], lpn=held)
        if not res["ok"]:
            truth("ACT_REJECTED", robot=rid, op=op, slot=sid, reason=res["reason"], task_id=msg.get("task_id"),
                  dist=round(math.hypot(s["pos"][0] - rx, s["pos"][1] - ry), 2))
        to_robot(rid, res)

    # Worker figures: shown for WORKER_SHOW_S in the aisle where an unrecorded change happens.
    workers = [n for n in (sup.getFromDef(f"WORKER_{k}") for k in range(4)) if n]
    worker_home = [n.getField("translation").getSFVec3f() for n in workers]
    worker_until = [0.0] * len(workers)

    def show_worker(sid):
        if not workers:
            return
        k = min(range(len(workers)), key=lambda i: worker_until[i])
        s = slots[sid]
        dx, dy = s["approach"][0] - s["pos"][0], s["approach"][1] - s["pos"][1]
        n = math.hypot(dx, dy) or 1.0
        workers[k].getField("translation").setSFVec3f([s["approach"][0] + dx / n * 0.3,
                                                      s["approach"][1] + dy / n * 0.3, 0.0])
        workers[k].getField("rotation").setSFRotation([0, 0, 1, math.atan2(-dy, -dx)])
        worker_until[k] = t_now[0] + WORKER_SHOW_S

    def worker_events():
        rack_lpns = [l for l, s in slot_of.items() if slots[s]["kind"] == "rack"]
        if not rack_lpns:
            return
        if worker_rng.random() < faults["worker_move_per_h"] / 3600 * step_s:
            lpn = worker_rng.choice(rack_lpns)
            src = slot_of[lpn]
            idx, face = int(src[3:]), src[:3]
            free = [f"{face}{idx + d:02d}" for d in range(-faults["worker_move_max_slots"],
                                                           faults["worker_move_max_slots"] + 1)
                    if d and f"{face}{idx + d:02d}" in occupant and occupant[f"{face}{idx + d:02d}"] is None]
            if free:
                dst = worker_rng.choice(free)
                take(lpn)
                put(lpn, dst)
                truth("S1_UNRECORDED_MOVE", lpn=lpn, item=sku_of[lpn], src=src, dst=dst)
                show_worker(src)
        if worker_rng.random() < faults["worker_remove_per_h"] / 3600 * step_s:
            lpn = worker_rng.choice(rack_lpns)
            src = slot_of[lpn]
            hide(lpn)
            truth("S2_UNRECORDED_REMOVAL", lpn=lpn, item=sku_of[lpn], src=src)
            show_worker(src)

    step_count = 0
    next_stats, next_print = 0.0, PRINT_PERIOD_S
    duration = cfg["episode"]["duration_s"]
    finished = False

    while sup.step(dt) != -1:
        t = t_now[0] = sup.getTime()
        step_count += 1

        # Orders: one wave at a time.
        while next_wave < len(waves) and waves[next_wave]["release_t"] <= t:
            w = waves[next_wave]
            next_wave += 1
            units_released += sum(ln["qty"] for ln in w["lines"])
            emit({"type": "ORDER_RELEASED", "wave": w["wave"], "lines": w["lines"]})

        # Inbound truck at the dock.
        if dock is None and next_truck < len(trucks) and trucks[next_truck]["arrive_t"] <= t:
            dock = {"truck": trucks[next_truck]["truck"], "queue": list(trucks[next_truck]["cartons"])}
            next_truck += 1
            show(truck_in, truck_in_pos, True)
            unload_at = t
            emit({"type": "TRUCK_ARRIVED", "truck": dock["truck"],
                  "asn": [{"lpn": c["lpn"], "item": c["sku"]} for c in dock["queue"]]})
        if dock is not None:
            entry_clear = all(b["x"] > belt_cfg["x_entry"] + belt_cfg["gap_m"] for b in belt)
            if dock["queue"] and t >= unload_at and entry_clear:
                c = dock["queue"].pop(0)
                belt.append({"lpn": c["lpn"], "x": belt_cfg["x_entry"], "scanned": False, "read": True})
                unload_at = t + belt_cfg["unload_s"]
            if not dock["queue"]:
                show(truck_in, truck_in_pos, False)
                emit({"type": "TRUCK_DEPARTED", "truck": dock["truck"]})
                dock = None

        # Belt: cartons move east; each heads for the east-most free zone still ahead of it and
        # is pushed onto the staging table there. Only cartons on the belt hold each other up.
        belt.sort(key=lambda b: -b["x"])
        claimed, prev_x, done = set(), None, []
        for b in belt:
            prev_before = prev_x
            limit = belt_cfg["x_end"] if prev_x is None else prev_x - belt_cfg["gap_m"]
            # A zone's pusher reaches PUSH_TOL either side of it, so a carton queued next to a
            # free zone can still be pushed in.
            free = [z for z in zones if zone_x[z] >= b["x"] - PUSH_TOL and not occupant[z] and z not in claimed]
            target = max(free, key=lambda z: zone_x[z]) if free else None
            if target:
                claimed.add(target)
                goal = min(max(zone_x[target], b["x"]), limit)
            else:
                goal = limit
            if goal > b["x"]:
                b["x"] = min(goal, b["x"] + belt_cfg["speed_mps"] * step_s)
            bnode[b["lpn"]].getField("translation").setSFVec3f([b["x"], belt_cfg["y"], belt_cfg["top"] + half_h])
            prev_x = b["x"]
            if not b["scanned"] and b["x"] >= belt_cfg["gate_x"]:
                b["scanned"] = True
                if unit(seed, b["lpn"], "scan") < faults["scanner_fail_p"]:
                    free_x = next((x for x in belt_cfg["exception"] if occupant[x] is None), None)
                    truth("SCANNER_FAIL", lpn=b["lpn"], item=sku_of[b["lpn"]], diverted_to=free_x)
                    if free_x:
                        # Diverted off the belt: it no longer holds back the cartons behind it.
                        put(b["lpn"], free_x)
                        done.append(b)
                        prev_x = prev_before
                        claimed.discard(target)
                        emit({"type": "INBOUND_READ_FAIL", "slot": free_x})
                        continue
                    b["read"] = False
            if target and b["x"] >= zone_x[target] - 1e-6:
                put(b["lpn"], target)
                done.append(b)
                staged_in += 1
                emit({"type": "INBOUND_STAGED", "slot": target, "lpn": b["lpn"] if b["read"] else None,
                      "item": sku_of[b["lpn"]] if b["read"] else None})
        for b in done:
            belt.remove(b)

        # Outbound docks.
        for region, ls in lane_state.items():
            staged = [s for s in lanes[region]["slots"] if occupant[s]]
            if t >= ls["next_cutoff"]:
                if staged and not ls["present"]:
                    missed_cutoffs += 1
                cutoff_now = bool(staged) and ls["present"]
                ls["next_cutoff"] += out_cfg["cutoff_period_s"]
            else:
                cutoff_now = False
            if ls["present"] and ls["loading_until"] is None and (
                    len(staged) >= out_cfg["dispatch_cartons"] or cutoff_now):
                ls["loading_until"] = t + out_cfg["load_s"]
            if ls["loading_until"] is not None and t >= ls["loading_until"]:
                cartons = []
                for s in staged:
                    lpn = occupant[s]
                    tagd = lane_tag.pop(lpn, {})
                    ok = tagd.get("item") == sku_of[lpn] and tagd.get("region") == region
                    cartons.append({"slot": s, "lpn": lpn, "item": sku_of[lpn], "order": tagd.get("order")})
                    if ok and tagd.get("order") in orders:
                        o = orders[tagd["order"]]
                        o["shipped"] += 1
                        units_shipped += 1
                        if o["shipped"] >= o["units"]:
                            o["done_t"] = t
                    else:
                        mis_shipped += 1
                    hide(lpn)
                shipments += 1
                ls.update(present=False, loading_until=None, back_at=t + out_cfg["turnaround_s"])
                show(ls["node"], ls["pos"], False)
                emit({"type": "OUTBOUND_DEPARTED", "region": region, "cartons": cartons})
            if not ls["present"] and t >= ls["back_at"]:
                ls["present"] = True
                show(ls["node"], ls["pos"], True)
                emit({"type": "OUTBOUND_TRUCK_ARRIVED", "region": region})

        # Faults outside the robots' actions.
        worker_events()
        for item in [p for p in s3_pending if p[0] <= t]:
            s3_pending.remove(item)
            _, dest, tid = item
            if occupant[dest] is None and returns:
                lpn = returns.pop(0)
                put(lpn, dest)
                truth("S3_DEST_TAKEN", slot=dest, lpn=lpn, item=sku_of[lpn], task_id=tid)
                show_worker(dest)

        # Robot messages.
        while receiver.getQueueLength() > 0:
            msg = json.loads(receiver.getString())
            receiver.nextPacket()
            if msg["kind"] == "ACT":
                handle_act(msg)
            elif msg["kind"] == "STATE":
                robot_state[msg["src"]] = {"task": msg.get("task"), "holding": msg.get("holding"),
                                           "phase": msg.get("phase")}
            elif msg["kind"] == "EVENT":
                ev = {k: v for k, v in msg.items() if k not in ("kind", "src")}
                emit(ev)

        # WES decisions -> robots.
        info = []
        for rid, node in rnodes.items():
            x, y, _ = node.getPosition()
            info.append({"id": rid, "role": robots[rid]["role"], "pos": [x, y], **robot_state[rid]})
        for d in wes.decide(t, info):
            ev_file.write(json.dumps({"t": round(t, 2), "kind": "DECISION", **d}) + "\n")
            if d.get("robot_id") is None:
                continue
            task = d.get("task") or {}
            if d["action"] == "EXECUTE":
                task_dest[d["task_id"]] = task.get("destination")
            elif d["action"] == "REROUTE":
                task_dest[d["task_id"]] = d.get("destination")
            to_robot(d["robot_id"], {"kind": "DECISION", **d})

        # Carried cartons ride on their robot.
        for lpn, rid in carried.items():
            x, y, _ = rnodes[rid].getPosition()
            bnode[lpn].getField("translation").setSFVec3f([x, y, CARRY_Z])

        # Localization infrastructure (overhead camera on the real setup).
        if step_count % POSE_EVERY_STEPS == 0:
            poses = {}
            for rid, node in rnodes.items():
                x, y, _ = node.getPosition()
                rot = node.getOrientation()
                poses[rid] = [round(x, 3), round(y, 3), round(math.atan2(rot[3], rot[0]), 3)]
                if rid in last_pos:
                    dist_m[rid] += math.hypot(x - last_pos[rid][0], y - last_pos[rid][1])
                last_pos[rid] = (x, y)
                if robot_state[rid]["task"]:
                    busy_s[rid] += POSE_EVERY_STEPS * step_s
            packet = json.dumps({"kind": "POSES", "poses": poses})
            for rid in rnodes:
                emitter.setChannel(100 + rid)
                emitter.send(packet)

        while snapshots and t >= snapshots[0]:
            at = int(snapshots.pop(0))
            for prefix, cam in cams.items():
                # Webots cannot write to non-ASCII paths: save to the temp folder, then move.
                name = f"{prefix}_{at}.png"
                tmp = Path(tempfile.gettempdir()) / f"webots_{os.getpid()}_{name}"
                if cam.saveImage(str(tmp), 90) == 0 and tmp.exists():
                    shutil.move(str(tmp), str(log_dir / name))
                else:
                    print(f"[sup] snapshot {name} failed")
        for k, until in enumerate(worker_until):
            if until and t >= until:
                workers[k].getField("translation").setSFVec3f(worker_home[k])
                worker_until[k] = 0.0
        if step_count % int(POSE_LOG_S / step_s) == 0:
            for rid, node in rnodes.items():
                x, y, _ = node.getPosition()
                st = robot_state[rid]
                pose_log.writerow([f"{t:.1f}", rid, f"{x:.2f}", f"{y:.2f}", st["task"] or "", st["phase"] or "",
                                   st["holding"] or ""])
                # Driving to a slot but not getting anywhere: report once per task.
                a = anchor.get(rid)
                if a is None or math.hypot(x - a[1], y - a[2]) > STUCK_M or not str(st["phase"]).startswith("go_"):
                    anchor[rid] = (t, x, y)
                elif t - a[0] > STUCK_S and (rid, st["task"]) not in stuck_reported:
                    stuck_reported.add((rid, st["task"]))
                    truth("ROBOT_STUCK", robot=rid, task_id=st["task"], phase=st["phase"], x=round(x, 2), y=round(y, 2),
                          others=[j for j, n in rnodes.items() if j != rid
                                  and math.hypot(n.getPosition()[0] - x, n.getPosition()[1] - y) < 1.0])
        if t >= next_stats:
            # Ground truth must stay consistent: every carton in one place only.
            bad = [l for l, sid in slot_of.items() if occupant[sid] != l]
            bad += [sid for sid, l in occupant.items() if l and slot_of.get(l) != sid]
            bad += [l for l in carried if l in slot_of] + [b["lpn"] for b in belt if b["lpn"] in slot_of or b["lpn"] in carried]
            if bad:
                truth("INVARIANT_VIOLATION", items=bad[:10])
            mismatch = ""
            if hasattr(wes, "record_snapshot"):
                rec = wes.record_snapshot()
                mismatch = sum(1 for s, v in rec.items() if v != gt_sku(s))
            ts.writerow([f"{t:.0f}", sum(1 for s in slots if slots[s]["kind"] == "rack" and occupant[s]), len(belt),
                         sum(1 for z in zones if occupant[z]),
                         sum(1 for x in belt_cfg["exception"] if occupant[x]),
                         sum(1 for r in lanes for s in lanes[r]["slots"] if occupant[s]),
                         units_released, units_shipped, sum(1 for r in robot_state.values() if r["task"]), mismatch])
            next_stats = t + STATS_PERIOD_S
        if t >= next_print:
            for f in (ev_file, truth_file, ts_file, pose_file):
                f.flush()
            print(f"[sup t={t:.0f}] released {units_released} shipped {units_shipped} | belt {len(belt)} "
                  f"buffer {sum(1 for z in zones if occupant[z])} exc "
                  f"{sum(1 for x in belt_cfg['exception'] if occupant[x])} | busy "
                  f"{sum(1 for r in robot_state.values() if r['task'])}/{len(robots)} | faults {truth_counts}")
            next_print = t + PRINT_PERIOD_S

        if not finished and t >= duration:
            finished = True
            done_orders = [o for o in orders.values() if o["done_t"] is not None]
            released_orders = [o for o in orders.values() if o["released_t"] <= t]
            rec = wes.record_snapshot() if hasattr(wes, "record_snapshot") else {}
            summary = {
                "tag": tag, "duration_s": round(t, 1), "wes": wes.name,
                "orders_released": len(released_orders), "orders_completed": len(done_orders),
                "mean_order_lead_s": (sum(o["done_t"] - o["released_t"] for o in done_orders) / len(done_orders)
                                      if done_orders else None),
                "units_released": units_released, "units_shipped": units_shipped, "mis_shipped": mis_shipped,
                "outbound_trucks": shipments, "missed_cutoffs": missed_cutoffs,
                "inbound_trucks_arrived": next_truck, "inbound_staged": staged_in,
                "cartons_on_belt_at_end": len(belt),
                "events": counts,
                "faults_injected": {k: v for k, v in truth_counts.items() if k not in DIAGNOSTICS},
                "diagnostics": {k: v for k, v in truth_counts.items() if k in DIAGNOSTICS},
                "record_mismatch_at_end": sum(1 for s, v in rec.items() if v != gt_sku(s)) if rec else None,
                "robot_distance_m": {r: round(d, 1) for r, d in dist_m.items()},
                "robot_busy_share": {r: round(b / t, 3) for r, b in busy_s.items()},
                "wes_stats": wes.stats() if hasattr(wes, "stats") else None,
            }
            for f in (ev_file, truth_file, ts_file, pose_file):
                f.flush()
            (log_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
            print("[sup] SUMMARY " + json.dumps(summary))
            if os.environ.get("QUIT_AT_END") == "1":
                sup.simulationQuit(0)


if __name__ == "__main__":
    main()
