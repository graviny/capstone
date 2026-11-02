"""Robot controller for the planogram-keeping task.

Patrol robots (one side-looking camera) drive inspection passes along shelf
faces, read item labels and decide for each anomaly whether to report it, to
which recovery robot, and with which evidence:
  none  - header only (E+N+C)
  thumb - strip of the target and its neighbors: tells the target apart unless a
          lookalike nearby has the same neighbors
  crop  - target with a legible label: always tells it apart
Recovery robots see only an item's color. At the reported position C they pick
the target among lookalikes with the evidence they got, or with their own
memory of the scene; otherwise they take the candidate closest to C.

Coordination (all policies): a recovery robot's load (tasks held, 0 = idle)
reaches the scanners through its ACKs, DONEs and STATE messages (sent when it
starts and finishes a task). Scanners assign to idle robots first and queue work
only when every robot is busy. A scanner that reports an item sends CLAIM to the
other scanners so the item is not reported twice; the finishing robot's STATE
carries "done" so every scanner drops the claim.
Scanners split the inspection faces: each infers from the others' poses which
face they are sweeping, and FACE messages announce a face ("start") and its
completion ("done").

Policies (POLICY env var overrides scenario.json):
  full     - always crop, nearest idle recovery robot
  none     - never evidence
  pull     - no evidence; the recovery robot asks for a crop when ambiguous
  cache    - crop unless this recovery robot already got one for this item
  proposed - pick the (recipient, evidence) pair with the lowest expected cost:
             travel + queue + P(wrong pick) * cost + bytes * congestion.
             P(wrong pick) uses the recipient's estimated chance of having seen
             the scene (from its pose history) and the lookalikes around the
             target. CLAIM goes only to scanners within claim_range_m of the
             item; the task-start STATE is skipped when the channel is busy;
             FACE "start" only when the face is more than FACE_ANNOUNCE_M
             away and never "done" (the others see the pass from our pose).

Transport AMRs (no camera) shuttle between stations: fleet STATUS at status_hz
and TASK_REQ to the fleet server (dst -1).

Robot-to-robot and robot-server messages go through the supervisor's channel
model; POSES and ACT / ACT_RESULT are out of band (localization and actuators).
"""
import json
import math
import os
import random
import sys
from collections import deque
from pathlib import Path

from controller import Robot

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "lib"))
from nav import GridMap  # noqa: E402

CAM_Z = 0.24
CAM_YAW = math.pi / 2      # camera looks left
CARRIED_Z = 0.33           # anything higher is riding on a robot
LOOK_S = 1.0
SERVER = -1
ASSIGN_PENALTY_M = 4.0     # one unfinished report counts like 4 m of extra distance
MEMORY_S = 20.0            # how long a sighting is trusted for neighbor context
POSE_SAMPLE_S = 0.25
BUSY_CONGESTION = 5.0      # congestion above this (load ~80 %) counts as a busy channel
LEVELS = ("none", "thumb", "crop")
FACE_BUSY_S = 3.0          # a scanner seen in a face's lane keeps that face taken this long
FACE_CLAIM_S = 60.0        # an announced face stays taken this long (or until "done")
FACE_ANNOUNCE_M = 1.0      # proposed: announce a face only if its start is farther than this


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    rid = int(sys.argv[sys.argv.index("--id") + 1])
    cfg = json.loads((ROOT / "scenario.json").read_text(encoding="utf-8"))
    policy = os.environ.get("POLICY", cfg["traffic"]["policy"])
    me = cfg["robots"][rid]
    nav, traffic, per, prop, fleet = cfg["nav"], cfg["traffic"], cfg["perception"], cfg["proposed"], cfg["fleet"]
    ev_bytes = traffic["evidence_bytes"]
    items = {it["id"]: it for it in cfg["items"]}
    slots = {s["id"]: s for s in cfg["slots"]}
    racks = cfg["racks"]
    recovery_ids = [r["id"] for r in cfg["robots"] if r["role"] == "recovery"]
    patrol_ids = [r["id"] for r in cfg["robots"] if r["role"] == "patrol"]
    rng = random.Random(cfg["seed"] * 100 + rid)
    grid = GridMap(cfg)

    robot = Robot()
    dt = int(robot.getBasicTimeStep())
    gps = robot.getDevice("gps")
    imu = robot.getDevice("imu")
    emitter = robot.getDevice("emitter")
    receiver = robot.getDevice("receiver")
    for dev in (gps, imu, receiver):
        dev.enable(dt)
    cam = robot.getDevice("camera") if me["role"] != "transport" else None
    if cam:
        cam.enable(4 * dt)
        cam.recognitionEnable(4 * dt)
        fov = cam.getFov()
    left = robot.getDevice("left wheel motor")
    right = robot.getDevice("right wheel motor")
    for m in (left, right):
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
    v_max = nav["speed_mps"] / nav["wheel_radius"]

    def drive(fwd, turn):
        left.setVelocity(max(-v_max, min(v_max, v_max * (fwd - turn))))
        right.setVelocity(max(-v_max, min(v_max, v_max * (fwd + turn))))

    seq = 0

    def send(dst, kind, size, payload=None):
        nonlocal seq
        emitter.send(json.dumps({"src": rid, "dst": dst, "seq": seq, "kind": kind, "size": size,
                                 "t_sent": robot.getTime(), "payload": payload or {}}))
        seq += 1

    def act(op, **kw):
        emitter.send(json.dumps({"kind": "ACT", "src": rid, "op": op, **kw}))

    def perceive(x, y, yaw):
        out = []
        if cam is None:
            return out
        for obj in cam.getRecognitionObjects():
            model = obj.getModel()
            if model not in items:
                continue
            # Camera frame (x fwd, y left, z up) -> robot frame (camera yawed left) -> world.
            px, py, pz = obj.getPosition()
            rx, ry = -py, px
            wx = x + math.cos(yaw) * rx - math.sin(yaw) * ry
            wy = y + math.sin(yaw) * rx + math.cos(yaw) * ry
            ax, ay, az, ang = obj.getOrientation()
            up = math.cos(ang) + az * az * (1 - math.cos(ang))
            out.append({"id": model, "type": items[model]["type"], "pos": (wx, wy), "z": CAM_Z + pz,
                        "fallen": up < per["fallen_tilt_cos"], "dist": math.hypot(wx - x, wy - y)})
        return out

    pile_zone = cfg["pile"]["zone"]

    def facing_slot(p, x, y):
        """Slot on the rack side (or pile) facing the robot that is closest to p."""
        if pile_zone["x_min"] - 0.1 <= p[0] <= pile_zone["x_max"] + 0.1:
            # Pile labels are readable from the lanes on either side of it.
            if not (pile_zone["x_max"] < x < pile_zone["x_max"] + 1.2
                    or pile_zone["x_min"] - 1.2 < x < pile_zone["x_min"]):
                return None
            s = nearest_slot(p, per["slot_match_m"])
            return s if s is not None and s["kind"] == "pile" else None
        ri = min(range(len(racks)), key=lambda k: abs(racks[k]["y"] - p[1]))
        if abs(racks[ri]["y"] - p[1]) > 0.5:
            return None
        # Labels are only readable from inside the aisle in front of the shelf.
        if not (racks[ri]["x_min"] <= x <= racks[ri]["x_max"] and 0.4 < abs(y - racks[ri]["y"]) < 1.6):
            return None
        prefix = f"r{ri}{'s' if y < racks[ri]['y'] else 'n'}"
        s = min((s for s in slots.values() if s["id"].startswith(prefix)), key=lambda s: abs(s["pos"][0] - p[0]))
        return s if abs(s["pos"][0] - p[0]) <= per["slot_match_m"] else None

    def nearest_slot(p, max_d=float("inf")):
        s = min(slots.values(), key=lambda s: math.hypot(s["pos"][0] - p[0], s["pos"][1] - p[1]))
        return s if math.hypot(s["pos"][0] - p[0], s["pos"][1] - p[1]) <= max_d else None

    def context(p, t):
        """Colors in the two neighboring slots of the slot at p, from recent sightings."""
        s = nearest_slot(p, 0.25)
        if s is None:
            return None
        out = []
        for d in (-1, 1):
            k = int(s["id"][3:]) + d
            nb = slots.get(f"{s['id'][:3]}{k:02d}")
            seen_type = None
            if nb is not None:
                for i, (ts, pos) in memory.items():
                    if t - ts < MEMORY_S and math.hypot(pos[0] - nb["pos"][0], pos[1] - nb["pos"][1]) < 0.15:
                        seen_type = items[i]["type"]
                        break
            out.append(seen_type)
        return out

    poses = {}                 # robot id -> (x, y, yaw), from the supervisor
    pose_hist = {j: deque(maxlen=int(120 / POSE_SAMPLE_S)) for j in recovery_ids}
    last_hist_t = -1.0
    memory = {}                # item id -> (time, position) of the last sighting
    reported = {}              # patrol: item -> time of last report
    first_seen = {}            # patrol: item -> time the anomaly was first seen
    assigned = {}              # patrol: item -> recovery robot it was reported to (until DONE)
    got_crop = set()           # patrol (cache): (item, recovery) pairs that already got a crop
    claims = {}                # patrol: item -> (time, sorter) reported by any scanner, until done
    sorter_load = {j: 0 for j in recovery_ids}   # patrol: tasks a sorter holds (0 = idle), as last heard
    wait_ewma = 0.0            # how long messages to us waited for the channel
    face_seen = {f["id"]: -1.0 for f in cfg["faces"]}
    # Faces this scanner is responsible for, and the scanners sharing that same set.
    my_faces = [f for f in cfg["faces"] if f["id"] in me.get("faces", [g["id"] for g in cfg["faces"]])]
    peers = [r["id"] for r in cfg["robots"] if r["role"] == "patrol" and r.get("faces") == me.get("faces")]
    face_busy = {}             # patrol: face -> last time another scanner was seen sweeping it
    face_claim = {}            # patrol: face -> time another scanner announced it is heading there
    face, patrol_mode = None, None

    def face_of_pose(px, py, pyaw):
        """The face whose lane a robot at this pose is driving along, in pass direction, if any."""
        for f in cfg["faces"]:
            (sx, sy), (ex, ey) = f["start"], f["end"]
            L = math.hypot(ex - sx, ey - sy)
            ux, uy = (ex - sx) / L, (ey - sy) / L
            along = (px - sx) * ux + (py - sy) * uy
            across = abs(-(px - sx) * uy + (py - sy) * ux)
            if 0 <= along <= L and across < 0.3 and abs(wrap(pyaw - math.atan2(uy, ux))) < 0.5:
                return f["id"]
        return None

    path, goal = deque(), None
    blocked_since = None
    tasks = deque()            # recovery: pending REPORT payloads
    state, task = "idle", None
    timer, waiting_act, chosen, how = 0.0, False, None, ""
    face_point = None
    next_status = rng.random() / fleet["status_hz"]
    task_req_at = 0.0

    def set_goal(x, y, target):
        nonlocal goal, blocked_since
        goal, blocked_since = target, None
        p = grid.plan((x, y), target)
        path.clear()
        if p:
            path.extend(p)
        return bool(p)

    def replan_around_others(x, y):
        others = [(p[0], p[1]) for j, p in poses.items()
                  if j != rid and math.hypot(p[0] - x, p[1] - y) < 1.5]
        p = grid.plan((x, y), goal, obstacles=others)
        if p:
            path.clear()
            path.extend(p)

    def congestion():
        """Queueing slowdown 1 / (1 - load), estimated from the observed wait (M/M/1: W / S = load / (1 - load))."""
        return 1.0 + wait_ewma / prop["service_s"]

    def byte_cost(size):
        """Seconds of everyone's time our airtime costs: airtime x stations waiting x congestion."""
        airtime = size * 8 / cfg["comm"]["bandwidth_bps"] + cfg["comm"]["per_attempt_overhead_s"]
        return airtime * prop["externality"] * congestion()

    def tell_scanners(kind, payload, exclude=()):
        for k in patrol_ids:
            if k not in exclude:
                send(k, kind, traffic["ack_bytes"], payload)

    def finish(x, y, status, photo=None):
        nonlocal state, task, face_point
        send(task["src"], "DONE", traffic["ack_bytes"] + (ev_bytes["crop"] if photo else 0),
             {"target": task["target"], "status": status, "picked": chosen, "photo": photo,
              "queue": len(tasks)})
        # Everyone else learns the item is handled and that this sorter has room again.
        tell_scanners("STATE", {"queue": len(tasks), "done": task["target"]}, exclude=(task["src"],))
        print(f"[robot_{rid}] {task['kind']} {task['target']}: {status} (picked {chosen}, {how}"
              f"{', photo' if photo else ''})")
        state, task, face_point = "return", None, None
        set_goal(x, y, me["home"])

    # ---- patrol decision helpers ----
    def p_seen(j, target_pos, since):
        """Chance recovery robot j saw the target since `since`, from its pose history."""
        frames = 0
        per_sample = POSE_SAMPLE_S / (4 * dt / 1000.0)
        for ts, px, py, pyaw in pose_hist[j]:
            if ts < since:
                continue
            d = math.hypot(target_pos[0] - px, target_pos[1] - py)
            if d < prop["seen_range_m"] and abs(wrap(math.atan2(target_pos[1] - py, target_pos[0] - px)
                                                     - pyaw - CAM_YAW)) < fov / 2:
                frames += per_sample
        return prop["p_remember"] * (1 - (1 - prop["p_detect_per_frame"]) ** frames)

    def p_guess_wrong(o, t):
        """Chance that 'closest same-color item to a noisy C' is not the target."""
        p = 0.0
        for i, (ts, pos) in memory.items():
            if i == o["id"] or items[i]["type"] != o["type"] or t - ts > MEMORY_S:
                continue
            d = math.hypot(pos[0] - o["pos"][0], pos[1] - o["pos"][1])
            if d < per["candidate_radius_m"] + 0.3:
                p += 0.5 * math.erfc((d / 2) / (per["c_noise_m"] * math.sqrt(2)))
        return min(1.0, p)

    def thumb_unique(o, t):
        """True if no nearby lookalike shares the target's neighbor colors."""
        ctx = context(o["pos"], t)
        if ctx is None:
            return False
        for i, (ts, pos) in memory.items():
            if i == o["id"] or items[i]["type"] != o["type"] or t - ts > MEMORY_S:
                continue
            if math.hypot(pos[0] - o["pos"][0], pos[1] - o["pos"][1]) < per["candidate_radius_m"] + 0.3 \
                    and context(pos, t) == ctx:
                return False
        return True

    def candidates():
        """Idle sorters if there are any; only when all are busy does work get queued."""
        idle = [j for j in recovery_ids if sorter_load[j] == 0]
        return idle or recovery_ids

    def balanced_nearest(x, y):
        return min(candidates(), key=lambda j: math.hypot(
            poses.get(j, (1e9, 1e9))[0] - x, poses.get(j, (1e9, 1e9))[1] - y)
            + ASSIGN_PENALTY_M * sorter_load[j])

    def want_photo():
        """Sorter: send a confirmation photo of the placement?"""
        if policy in ("full", "cache"):
            return True
        if policy in ("none", "pull"):
            return False
        unseen_cost = cfg["episode"]["p_place_error"] * prop["undetected_delay_s"]
        return unseen_cost > byte_cost(ev_bytes["crop"])

    def report(o, anomaly, s, x, y, t):
        """Scanner: decide and send one anomaly report."""
        first_seen.setdefault(o["id"], t)
        decision = decide(o, anomaly, s, x, y, t)
        reported[o["id"]] = t      # also rate-limits a deliberate silence
        if decision is None:
            return
        dst, level = decision
        assigned[o["id"]] = dst
        claims[o["id"]] = (t, dst)
        sorter_load[dst] += 1      # until the sorter's own ACK / STATE corrects it
        if level == "crop":
            got_crop.add((o["id"], dst))
        # Position error runs along the face; which face it is on is known from where it was seen.
        if s["kind"] == "pile":
            c = [s["pos"][0], o["pos"][1] + rng.gauss(0, per["c_noise_m"])]
        else:
            c = [o["pos"][0] + rng.gauss(0, per["c_noise_m"]), s["pos"][1]]
        send(dst, "REPORT", traffic["header_bytes"] + ev_bytes[level],
             {"target": o["id"], "kind": anomaly, "type": o["type"], "c": c,
              "n_slot": items[o["id"]]["home"], "t_first": first_seen[o["id"]], "level": level,
              "ctx": context(o["pos"], t) if level == "thumb" else None})
        # Tell the other scanners so they do not send a second sorter. The proposed policy only
        # tells scanners close enough to see this item soon.
        for k in patrol_ids:
            if k == rid:
                continue
            kx, ky, _ = poses.get(k, (1e9, 1e9, 0))
            if policy != "proposed" or math.hypot(kx - o["pos"][0], ky - o["pos"][1]) < prop["claim_range_m"]:
                send(k, "CLAIM", traffic["ack_bytes"], {"target": o["id"], "dst": dst})

    def decide(o, anomaly, slot, x, y, t):
        """Return (dst, level), or None to stay silent."""
        if policy == "full":
            return balanced_nearest(x, y), "crop"
        if policy in ("none", "pull"):
            return balanced_nearest(x, y), "none"
        if policy == "cache":
            dst = balanced_nearest(x, y)
            return dst, "none" if (o["id"], dst) in got_crop else "crop"
        # proposed
        p_wrong_base = 0.0 if anomaly == "fallen" else p_guess_wrong(o, t)
        unique = p_wrong_base > 0 and thumb_unique(o, t)
        best = None
        for j in candidates():
            px, py, _ = poses.get(j, (1e9, 1e9, 0))
            base = (math.hypot(px - o["pos"][0], py - o["pos"][1]) / nav["speed_mps"]
                    + sorter_load[j] * prop["task_s"])
            miss = 1 - p_seen(j, o["pos"], first_seen.get(o["id"], t) - 5)
            for level in LEVELS:
                pw = {"none": p_wrong_base, "thumb": 0.0 if unique else p_wrong_base, "crop": 0.0}[level] * miss
                size = traffic["header_bytes"] + ev_bytes[level]
                cost = base + pw * prop["wrong_pick_cost_s"] + byte_cost(size)
                if best is None or cost < best[0]:
                    best = (cost, j, level)
        return best[1], best[2]

    while robot.step(dt) != -1:
        t = robot.getTime()
        x, y, _ = gps.getValues()
        if math.isnan(x):
            continue
        yaw = imu.getRollPitchYaw()[2]
        objs = perceive(x, y, yaw)
        for o in objs:
            memory[o["id"]] = (t, o["pos"])

        while receiver.getQueueLength() > 0:
            msg = json.loads(receiver.getString())
            receiver.nextPacket()
            kind = msg["kind"]
            if "t_delivered" in msg:
                # Delivery time minus airtime: how long the shared channel held this message.
                wait = msg["t_delivered"] - msg["t_sent"] - msg["size"] * 8 / cfg["comm"]["bandwidth_bps"]
                wait_ewma = 0.9 * wait_ewma + 0.1 * max(wait, 0.0)
            if kind == "POSES":
                poses = {int(k): v for k, v in msg["poses"].items()}
                if me["role"] == "patrol" and t - last_hist_t >= POSE_SAMPLE_S:
                    last_hist_t = t
                    for j in recovery_ids:
                        if j in poses:
                            pose_hist[j].append((t, *poses[j]))
            elif kind == "TASK" and me["role"] == "transport" and state != "moving":
                state = "moving"
                set_goal(x, y, msg["payload"]["station"])
            elif kind == "REPORT":
                p = msg["payload"]
                busy = {q["target"] for q in tasks} | ({task["target"]} if task else set())
                if p["target"] not in busy:
                    tasks.append({**p, "src": msg["src"]})
                elif task and task["target"] == p["target"] and LEVELS.index(p["level"]) > LEVELS.index(task["level"]):
                    task["level"] = p["level"]
                # The ACK also tells the reporter how loaded this sorter now is.
                send(msg["src"], "ACK", traffic["ack_bytes"],
                     {"ack_seq": msg["seq"], "queue": len(tasks) + (task is not None)})
            elif kind == "ACK" and msg["src"] in sorter_load:
                sorter_load[msg["src"]] = msg["payload"]["queue"]
            elif kind == "CLAIM":
                claims[msg["payload"]["target"]] = (t, msg["payload"]["dst"])
                sorter_load[msg["payload"]["dst"]] += 1
            elif kind == "FACE":
                f = msg["payload"]["face"]
                if msg["payload"]["phase"] == "start":
                    face_claim[f] = t
                else:
                    face_seen[f] = max(face_seen[f], t)
                    face_claim.pop(f, None)
            elif kind == "STATE":
                sorter_load[msg["src"]] = msg["payload"]["queue"]
                done = msg["payload"].get("done")
                if done:
                    for d in (claims, reported, assigned, first_seen):
                        d.pop(done, None)
            elif kind == "DONE":
                sorter_load[msg["src"]] = msg["payload"]["queue"]
                claims.pop(msg["payload"]["target"], None)
                reported.pop(msg["payload"]["target"], None)
                assigned.pop(msg["payload"]["target"], None)
                first_seen.pop(msg["payload"]["target"], None)
                photo = msg["payload"].get("photo")
                if photo and (photo["slot"] != items[photo["item"]]["home"] or photo["fallen"]):
                    # The confirmation photo shows a bad placement: report it right away.
                    s = slots[photo["slot"]]
                    o = {"id": photo["item"], "type": items[photo["item"]]["type"], "pos": tuple(s["pos"]),
                         "fallen": photo["fallen"]}
                    reported.pop(photo["item"], None)
                    report(o, "fallen" if photo["fallen"] else "misplaced", s, x, y, t)
            elif kind == "REQ":
                target = msg["payload"]["target"]
                send(msg["src"], "EVID", 40 + ev_bytes["crop"], {"target": target, "level": "crop"})
                got_crop.add((target, msg["src"]))
            elif kind == "EVID" and task is not None and task["target"] == msg["payload"]["target"]:
                task["level"] = "crop"
            elif kind == "ACT_RESULT" and task is not None:
                waiting_act = False
                op, ok = msg["op"], msg["ok"]
                if op == "upright" or (op == "pick" and not ok):
                    finish(x, y, "ok" if ok else "act_failed")
                elif op == "pick":
                    state = "go_n"
                    face_point = slots[task["n_slot"]]["pos"]
                    set_goal(x, y, slots[task["n_slot"]]["approach"])
                elif op == "place" and ok and msg["exchanged"] and "back_slot" not in task:
                    # Home slot was taken: we now hold its occupant; put it where ours came from.
                    back = slots[task["pick_slot"]]
                    task["back_slot"] = back["id"]
                    state, face_point = "go_back", back["pos"]
                    set_goal(x, y, back["approach"])
                elif op == "place" and ok and not msg["exchanged"]:
                    photo = {"item": msg["item"], "slot": msg["actual_slot"], "fallen": msg["fallen"]}
                    finish(x, y, "ok", photo if want_photo() else None)
                elif op == "place":
                    act("drop")
                    waiting_act = True
                elif op == "drop":
                    finish(x, y, "place_failed")

        # ---- behavior ----
        if me["role"] == "transport":
            if t >= next_status:
                send(SERVER, "STATUS", fleet["status_bytes"], {"pos": [round(x, 2), round(y, 2)]})
                next_status = t + 1 / fleet["status_hz"]
            if state == "moving" and not path:
                state, timer = "dwell", t + fleet["dwell_s"]
            if state in ("idle", "dwell") and t >= timer and t - task_req_at > fleet["task_timeout_s"]:
                # Ask for work; repeated until a TASK arrives (requests or replies can be lost).
                send(SERVER, "TASK_REQ", fleet["task_req_bytes"], {"done": state == "dwell"})
                task_req_at = t

        elif me["role"] == "patrol":
            if not path:
                if patrol_mode == "to_start":
                    # At the start of the face: drive along it with the shelf on the camera side.
                    goal = face["end"]
                    path.append(tuple(face["end"]))
                    patrol_mode = "pass"
                else:
                    if face is not None:
                        face_seen[face["id"]] = t
                        # Others infer a pass from our pose; proposed leaves "done" to that.
                        if policy != "proposed":
                            tell_scanners("FACE", {"face": face["id"], "phase": "done"}, exclude=(rid,))

                    # Most overdue face relative to how often it should be checked, skipping
                    # faces another scanner is driving along or has announced.
                    def taken(f):
                        return (t - face_busy.get(f["id"], -1e9) < FACE_BUSY_S
                                or t - face_claim.get(f["id"], -1e9) < FACE_CLAIM_S)

                    def overdue(f):
                        return (t - face_seen[f["id"]]) / f["interval_s"] - (10 if taken(f) else 0) \
                            + rng.random() * 0.1
                    if face is None:
                        # First pick: nobody has heard from anybody yet, so take turns by rank.
                        ranked = sorted(my_faces, key=overdue, reverse=True)
                        face = ranked[peers.index(rid) % len(ranked)]
                    else:
                        face = max(my_faces, key=overdue)
                    set_goal(x, y, face["start"])
                    patrol_mode = "to_start"
                    print(f"[robot_{rid}] t={t:.0f} sweep {face['id']}")
                    # Announce the face. Proposed skips it when we will be in its lane soon
                    # enough for the others to see it from our pose.
                    far = math.hypot(face["start"][0] - x, face["start"][1] - y) > FACE_ANNOUNCE_M
                    if policy != "proposed" or far:
                        tell_scanners("FACE", {"face": face["id"], "phase": "start"}, exclude=(rid,))

            # Infer which faces the other scanners are sweeping right now from their poses.
            for k in patrol_ids:
                if k != rid and k in poses:
                    f = face_of_pose(*poses[k])
                    if f is not None:
                        face_seen[f] = t
                        face_busy[f] = t

            for o in objs:
                if o["dist"] > per["patrol_read_label_m"] or o["z"] > CARRIED_Z:
                    continue
                s = facing_slot(o["pos"], x, y)
                if s is None:
                    continue
                anomaly = None
                if s["kind"] == "pile":
                    anomaly = "inbound"
                elif o["fallen"]:
                    anomaly = "fallen"
                elif s["id"] != items[o["id"]]["home"]:
                    anomaly = "misplaced"
                last = reported.get(o["id"])
                claim = claims.get(o["id"])
                if (anomaly and recovery_ids and (last is None or t - last >= traffic["report_timeout_s"])
                        and (claim is None or t - claim[0] >= traffic["report_timeout_s"])):
                    report(o, anomaly, s, x, y, t)

        else:
            if state in ("idle", "return") and tasks:
                task = tasks.popleft()
                # Starting work: let the scanners know this sorter is busy. The reporter already
                # heard it in the ACK; the proposed policy skips this when the channel is busy.
                if policy != "proposed" or congestion() < BUSY_CONGESTION:
                    tell_scanners("STATE", {"queue": len(tasks) + 1}, exclude=(task["src"],))
                seen = memory.get(task["target"])
                # Already saw the anomalous scene -> can tell the target from lookalikes.
                task["seen"] = bool(seen and seen[0] >= task["t_first"] - 5
                                    and math.hypot(seen[1][0] - task["c"][0], seen[1][1] - task["c"][1]) < 0.5)
                task["pulled"] = False
                face_point, chosen, how = task["c"], None, ""
                state = "go_c"
                set_goal(x, y, nearest_slot(task["c"])["approach"])
            elif state == "go_c" and not path:
                state, timer = "look", t + LOOK_S
            elif state == "wait_evid" and (task["level"] == "crop" or t >= timer):
                state, timer = "look", t
            elif state == "look" and t >= timer:
                c = task["c"]
                cands = [o for o in objs if o["type"] == task["type"] and o["z"] <= CARRIED_Z
                         and math.hypot(o["pos"][0] - c[0], o["pos"][1] - c[1]) < per["candidate_radius_m"]
                         and (o["fallen"] or task["kind"] != "fallen")]

                def closest(group):
                    return min(group, key=lambda o: math.hypot(o["pos"][0] - c[0], o["pos"][1] - c[1]))["id"]

                chosen = None
                if task["seen"] or task["level"] == "crop":
                    how = "seen" if task["seen"] else ("pulled" if task["pulled"] else "crop")
                    chosen = task["target"] if any(o["id"] == task["target"] for o in cands) else None
                elif policy == "pull" and len(cands) > 1 and not task["pulled"]:
                    task["pulled"] = True
                    send(task["src"], "REQ", traffic["req_bytes"], {"target": task["target"]})
                    state, timer = "wait_evid", t + traffic["evidence_timeout_s"]
                    continue
                elif cands and task["level"] == "thumb" and task.get("ctx"):
                    matches = [o for o in cands if context(o["pos"], t) == task["ctx"]]
                    chosen, how = closest(matches or cands), "thumb"
                elif cands:
                    chosen, how = closest(cands), "guess"
                if chosen is None:
                    how = how or "none"
                    finish(x, y, "not_found")
                else:
                    picked = next(o for o in cands if o["id"] == chosen)
                    task["pick_slot"] = nearest_slot(picked["pos"])["id"]
                    state, timer = "act", t + traffic["act_s"]
            elif state == "act" and t >= timer and not waiting_act:
                act("upright" if task["kind"] == "fallen" else "pick", item=chosen, intended=task["target"])
                waiting_act = True
                state = "acting"
            elif state == "go_n" and not path:
                state, timer = "place", t + traffic["act_s"]
            elif state == "place" and t >= timer and not waiting_act:
                act("place", slot=task.get("back_slot", task["n_slot"]))
                waiting_act = True
                state = "acting"
            elif state == "go_back" and not path:
                state, timer = "place", t + traffic["act_s"]
            elif state == "return" and not path:
                state = "idle"

        # ---- motion ----
        if state in ("look", "act", "acting", "place", "wait_evid"):
            # Turn so the work point is on the camera side (left).
            fx, fy = face_point
            err = wrap(math.atan2(fy - y, fx - x) - CAM_YAW - yaw)
            drive(0.0, 0.0 if abs(err) < 0.1 else math.copysign(0.3, err))
            if state == "look" and abs(err) >= 0.1:
                timer = t + LOOK_S      # look only once facing the spot
            continue
        if not path:
            drive(0.0, 0.0)
            continue

        tx, ty = path[0]
        reach = nav["goal_reach_m"] if len(path) == 1 else nav["reach_m"]
        if math.hypot(tx - x, ty - y) < reach:
            path.popleft()
            continue

        heading_to_wp = math.atan2(ty - y, tx - x)
        err = wrap(heading_to_wp - yaw)
        if abs(err) > 0.4:
            # Turning in place cannot hit anyone.
            drive(0.0, math.copysign(0.5, err))
            continue

        # Collision avoidance: the lower id has priority; yield to robots in the way.
        must_stop, back_off = False, False
        for j, (px, py, _) in poses.items():
            if j == rid:
                continue
            d = math.hypot(px - x, py - y)
            in_way = abs(wrap(math.atan2(py - y, px - x) - heading_to_wp)) < 0.8
            if in_way and (d < nav["stop_dist_m"] or (d < nav["avoid_dist_m"] and rid > j)):
                must_stop = True
                back_off |= d < nav["stop_dist_m"] and rid > j
        if must_stop:
            # Too close: the yielding robot backs up to open a gap.
            drive(-0.4 if back_off else 0.0, 0.0)
            blocked_since = blocked_since if blocked_since is not None else t
            if t - blocked_since > nav["replan_after_s"]:
                replan_around_others(x, y)
                blocked_since = None
            continue
        blocked_since = None
        drive(1.0, 1.5 * err)


if __name__ == "__main__":
    main()
