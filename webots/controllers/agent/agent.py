"""Robot controller: executes WES decisions and reports what its camera sees.

Roles
  transport  PUTAWAY / PICK / RELOCATE: drive to the source, check the carton's label
             with the camera, pick, drive to the destination, check that it is free, place.
  patrol     VERIFY: drive to a slot and report what is there. With "patrol.idle_sweep"
             (gen_world --patrol-sweep) an idle patrol also sweeps the rack faces and
             reports every slot it passes (OBSERVATION_RESULT); otherwise it waits.

A robot only knows what its camera shows (labels within read_m); the supervisor
checks that a pick / place is physically possible. A robot does not notice a
placement error: it reports TASK_COMPLETED for the slot it was told.

Decisions (from the WES via the supervisor): EXECUTE, REROUTE, WAIT, CANCEL, REASSIGN.
Events (to the WES): TASK_COMPLETED, TASK_FAILED, ITEM_MISMATCH, SHORT_PICK,
DESTINATION_OCCUPIED, OBSERVATION_RESULT. Messages are delivered instantly.
"""
import json
import math
import sys
from pathlib import Path

from controller import Robot

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "lib"))
from nav import GridMap  # noqa: E402

CAM_Z = 0.24
CAM_YAW = math.pi / 2      # camera looks left
NEAR_GOAL_M = 0.7          # this close to the goal only the hard stop distance applies
ALONGSIDE_M = 0.4          # a yielding robot waits for a moving robot this close beside / ahead of it
ALONGSIDE_RAD = 0.8        # ... going in about the same direction
PASS_M = 0.4               # robots this close beside the way are passed slowly, steering away
PASS_TURN = 1.5
MOVING_M = 0.01            # a robot that moved this far between two pose broadcasts is moving
SWEEP_ANGLE = 0.3          # patrol reports a slot when it is this close to the camera axis
IDLE_PARK_S = 20.0
ARRIVE_M = 0.35            # a path that ends farther than this from the goal is replanned
MAX_RETRIES = 3


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    rid = int(sys.argv[sys.argv.index("--id") + 1])
    cfg = json.loads((ROOT / "scenario.json").read_text(encoding="utf-8"))
    me = cfg["robots"][rid]
    nav, per = cfg["nav"], cfg["perception"]
    slots = {s["id"]: s for s in cfg["slots"]}
    skus = {s["sku"] for s in cfg["skus"]}
    racks = cfg["racks"]
    grid = GridMap(cfg)
    patrol_ids = [r["id"] for r in cfg["robots"] if r["role"] == "patrol"]

    robot = Robot()
    dt = int(robot.getBasicTimeStep())
    gps = robot.getDevice("gps")
    imu = robot.getDevice("imu")
    emitter = robot.getDevice("emitter")
    receiver = robot.getDevice("receiver")
    for dev in (gps, imu, receiver):
        dev.enable(dt)
    cam = robot.getDevice("camera")
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

    def send(kind, **kw):
        emitter.send(json.dumps({"kind": kind, "src": rid, **kw}))

    def event(etype, **kw):
        send("EVENT", type=etype, robot_id=rid, **kw)

    def perceive(x, y, yaw):
        out = []
        for obj in cam.getRecognitionObjects():
            model = obj.getModel()
            if model not in skus:
                continue
            # Camera frame (x fwd, y left, z up) -> robot frame (camera yawed left) -> world.
            px, py, pz = obj.getPosition()
            rx, ry = -py, px
            wx = x + math.cos(yaw) * rx - math.sin(yaw) * ry
            wy = y + math.sin(yaw) * rx + math.cos(yaw) * ry
            out.append({"item": model, "pos": (wx, wy), "z": CAM_Z + pz, "dist": math.hypot(wx - x, wy - y)})
        return out

    def view_angle(s, x, y, yaw):
        return wrap(math.atan2(s["pos"][1] - y, s["pos"][0] - x) - yaw - CAM_YAW)

    def visible(s, x, y, yaw, max_angle):
        if math.hypot(s["pos"][0] - x, s["pos"][1] - y) > per["read_m"] or abs(view_angle(s, x, y, yaw)) > max_angle:
            return False
        if s["kind"] == "rack":
            # Labels face the aisle: readable only from the same side of the rack.
            ry = min(racks, key=lambda r: abs(r["y"] - s["pos"][1]))["y"]
            return (y - ry) * (s["pos"][1] - ry) > 0 and abs(y - ry) < 1.6
        return True

    def observe(s, objs):
        hits = [o for o in objs if o["z"] < per["carried_z"]
                and math.hypot(o["pos"][0] - s["pos"][0], o["pos"][1] - s["pos"][1]) <= per["slot_match_m"]]
        return min(hits, key=lambda o: o["dist"])["item"] if hits else "EMPTY"

    poses, moving = {}, set()
    path, goal = [], None
    blocked_since = None
    task, phase, holding = None, None, None
    face_point, timer, waiting_act = None, 0.0, False
    # Patrols start spread out over the faces.
    n_faces = len(cfg["faces"])
    k0 = patrol_ids.index(rid) * n_faces // len(patrol_ids) if rid in patrol_ids else 0
    sweep = {"face": None, "mode": None, "seen": set(), "k": k0 - 1}
    parked, idle_since = False, None
    plan_retries, act_retries = 0, 0

    def set_goal(x, y, target):
        nonlocal goal, blocked_since
        goal, blocked_since = target, None
        p = grid.plan((x, y), target)
        path.clear()
        if p:
            path.extend(p)
        return bool(p)

    def replan_around_others(x, y):
        others = [(p[0], p[1]) for j, p in poses.items() if j != rid and math.hypot(p[0] - x, p[1] - y) < 1.5]
        p = grid.plan((x, y), goal, obstacles=others)
        if p:
            path.clear()
            path.extend(p)

    def state():
        send("STATE", task=task["task_id"] if task else None, holding=holding, phase=phase)

    def go_slot(x, y, sid, new_phase):
        nonlocal phase, face_point, plan_retries
        phase, face_point, plan_retries = new_phase, slots[sid]["pos"], 0
        set_goal(x, y, slots[sid]["approach"])
        state()

    def finish():
        nonlocal task, phase, parked
        task, phase, parked = None, None, False
        sweep["face"] = None
        state()

    def start(d, x, y):
        nonlocal task, act_retries
        task, act_retries = dict(d["task"]), 0
        if task["task_type"] == "VERIFY":
            go_slot(x, y, task["destination"], "go_verify")
        else:
            go_slot(x, y, task["source"], "go_src")
        print(f"[robot_{rid}] {task['task_id']} {task['task_type']} {task['item_id']} "
              f"{task['source']} -> {task['destination']}")

    while robot.step(dt) != -1:
        t = robot.getTime()
        x, y, _ = gps.getValues()
        if math.isnan(x):
            continue
        yaw = imu.getRollPitchYaw()[2]
        objs = perceive(x, y, yaw)

        while receiver.getQueueLength() > 0:
            msg = json.loads(receiver.getString())
            receiver.nextPacket()
            kind = msg["kind"]
            if kind == "POSES":
                new_poses = {int(k): v for k, v in msg["poses"].items()}
                moving = {j for j, p in new_poses.items()
                          if j in poses and math.hypot(p[0] - poses[j][0], p[1] - poses[j][1]) > MOVING_M}
                poses = new_poses
            elif kind == "DECISION":
                action = msg["action"]
                if action == "EXECUTE":
                    if task is None:
                        start(msg, x, y)
                    else:
                        event("TASK_FAILED", task_id=msg["task_id"], reason="BUSY")
                elif task is None or msg["task_id"] != task["task_id"]:
                    continue
                elif action == "REROUTE":
                    task["source"] = msg.get("source", task["source"])
                    task["destination"] = msg.get("destination", task["destination"])
                    task["state_version"] = task.get("state_version", 1) + 1
                    if holding:
                        go_slot(x, y, task["destination"], "go_dst")
                    else:
                        go_slot(x, y, task["source"], "go_src")
                elif action == "WAIT":
                    phase = "wait"
                    path.clear()
                    state()
                elif action in ("CANCEL", "REASSIGN"):
                    if holding:
                        event("TASK_FAILED", task_id=task["task_id"], reason="HOLDING_CARTON", item_id=holding)
                    else:
                        finish()
            elif kind == "ACT_RESULT" and task is not None:
                waiting_act = False
                reason = msg.get("reason")
                if not msg["ok"] and reason == "too_far" and act_retries < MAX_RETRIES:
                    # Stopped short of the slot (e.g. after yielding): drive up again.
                    act_retries += 1
                    go_slot(x, y, msg["slot"], "go_src" if msg["op"] == "pick" else "go_dst")
                elif msg["op"] == "pick":
                    if msg["ok"]:
                        holding = msg["item"]
                        go_slot(x, y, task["destination"], "go_dst")
                    elif reason == "empty":
                        # Taken in the moment between looking and picking.
                        event("SHORT_PICK", task_id=task["task_id"], slot=msg["slot"], item_id=task["item_id"],
                              observed="EMPTY")
                        phase = "wait"
                        state()
                    else:
                        event("TASK_FAILED", task_id=task["task_id"], reason=f"PICK_{reason}",
                              slot=msg["slot"], item_id=task["item_id"])
                        finish()
                elif msg["op"] == "place":
                    if msg["ok"]:
                        event("TASK_COMPLETED", task_id=task["task_id"], task_type=task["task_type"],
                              item_id=holding, slot=task["destination"])
                        holding = None
                        finish()
                    else:
                        # Still holding the carton, so the WES has to send it somewhere else.
                        event("DESTINATION_OCCUPIED", task_id=task["task_id"], slot=msg["slot"], item_id=holding,
                              observed="UNKNOWN" if reason == "occupied" else "UNREACHABLE")
                        phase = "wait"
                        state()

        # ---- task phases ----
        if phase in ("go_src", "go_dst", "go_verify") and not path:
            if goal is not None and math.hypot(goal[0] - x, goal[1] - y) > ARRIVE_M and plan_retries < MAX_RETRIES:
                # No path (or it ran out short of the goal): plan again around the others.
                plan_retries += 1
                replan_around_others(x, y)
                continue
            phase = {"go_src": "look_src", "go_dst": "look_dst", "go_verify": "look_verify"}[phase]
            timer = t + per["look_s"]
        elif phase in ("look_src", "look_dst", "look_verify") and t >= timer:
            sid = {"look_src": task["source"], "look_dst": task["destination"],
                   "look_verify": task["destination"]}[phase]
            s = slots[sid]
            seen = observe(s, objs) if visible(s, x, y, yaw, fov / 2) else "UNREADABLE"
            if phase == "look_verify":
                event("OBSERVATION_RESULT", task_id=task["task_id"], slot=sid, observed=seen, source="verify")
                event("TASK_COMPLETED", task_id=task["task_id"], task_type="VERIFY", slot=sid, item_id=seen)
                finish()
            elif phase == "look_src":
                if seen == task["item_id"]:
                    phase, timer = "pick", t + nav["act_s"]
                else:
                    event("SHORT_PICK" if seen == "EMPTY" else "ITEM_MISMATCH", task_id=task["task_id"],
                          slot=sid, item_id=task["item_id"], observed=seen)
                    phase = "wait"
                    state()
            else:
                # Cannot see the slot: try anyway, the place itself fails if it is taken.
                if seen in ("EMPTY", "UNREADABLE"):
                    phase, timer = "place", t + nav["act_s"]
                else:
                    event("DESTINATION_OCCUPIED", task_id=task["task_id"], slot=sid, item_id=holding,
                          observed=seen)
                    phase = "wait"
                    state()
        elif phase in ("pick", "place") and t >= timer and not waiting_act:
            sid = task["source"] if phase == "pick" else task["destination"]
            send("ACT", op=phase, slot=sid, task_id=task["task_id"], item=task["item_id"], order=task.get("order"))
            waiting_act = True

        # ---- idle behavior ----
        # Transports wait where they finished (the next task is often close by) and park
        # only after IDLE_PARK_S without work.
        if task is not None:
            idle_since = None
        elif idle_since is None:
            idle_since = t
        sweeping = me["role"] == "patrol" and cfg.get("patrol", {}).get("idle_sweep", False)
        if task is None and not sweeping and not parked and t - idle_since >= IDLE_PARK_S:
            set_goal(x, y, me["home"])
            parked = True
        if task is None and sweeping:
            faces = cfg["faces"]
            if sweep["face"] is None or (sweep["mode"] == "pass" and not path):
                sweep["k"] = (sweep["k"] + 1) % len(faces)
                sweep.update(face=faces[sweep["k"]], mode="to_start", seen=set())
                set_goal(x, y, sweep["face"]["start"])
            elif sweep["mode"] == "to_start" and not path:
                goal = sweep["face"]["end"]
                path.append(tuple(sweep["face"]["end"]))
                sweep["mode"] = "pass"
            if sweep["mode"] == "pass":
                fid = sweep["face"]["id"]
                for s in slots.values():
                    if s.get("face") == fid and s["id"] not in sweep["seen"] and visible(s, x, y, yaw, SWEEP_ANGLE):
                        sweep["seen"].add(s["id"])
                        event("OBSERVATION_RESULT", slot=s["id"], observed=observe(s, objs), source="patrol")

        # ---- motion ----
        if phase in ("look_src", "look_dst", "look_verify", "pick", "place"):
            # Turn so the slot is on the camera side (left).
            fx, fy = face_point
            err = wrap(math.atan2(fy - y, fx - x) - CAM_YAW - yaw)
            drive(0.0, 0.0 if abs(err) < 0.08 else math.copysign(0.3, err))
            if phase.startswith("look") and abs(err) >= 0.08:
                timer = t + per["look_s"]      # look only once facing the slot
            continue
        if phase == "wait" or not path:
            drive(0.0, 0.0)
            continue

        tx, ty = path[0]
        reach = nav["goal_reach_m"] if len(path) == 1 else nav["reach_m"]
        if math.hypot(tx - x, ty - y) < reach:
            path.pop(0)
            continue

        heading_to_wp = math.atan2(ty - y, tx - x)
        err = wrap(heading_to_wp - yaw)
        if abs(err) > 0.4:
            # Turning in place cannot hit anyone.
            drive(0.0, math.copysign(0.5, err))
            continue

        # Collision avoidance: the lower id has priority; yield to robots in the way.
        # Side by side in the same direction the yielding robot also waits, so the one
        # with priority pulls ahead instead of the two rubbing along together.
        near_goal = goal is not None and math.hypot(goal[0] - x, goal[1] - y) < NEAR_GOAL_M
        must_stop, back_off, steer, slow = False, False, 0.0, False
        for j, (px, py, pyaw) in poses.items():
            if j == rid:
                continue
            d = math.hypot(px - x, py - y)
            bearing = abs(wrap(math.atan2(py - y, px - x) - heading_to_wp))
            in_way = bearing < 0.8
            if in_way and (d < nav["stop_dist_m"] or (d < nav["avoid_dist_m"] and rid > j and not near_goal)):
                must_stop = True
                back_off |= d < nav["stop_dist_m"] and rid > j
            elif (rid > j and j in moving and d < ALONGSIDE_M and bearing < math.pi / 2
                  and abs(wrap(pyaw - heading_to_wp)) < ALONGSIDE_RAD):
                must_stop = True
            elif d < PASS_M and bearing < math.pi / 2:
                # Passing close by (the path ignores other robots): slow down and keep away.
                rel = wrap(math.atan2(py - y, px - x) - heading_to_wp)
                steer -= math.copysign(PASS_TURN * (PASS_M - d) / PASS_M, rel)
                slow = True
        if must_stop:
            # Too close: the yielding robot backs up to open a gap.
            drive(-0.4 if back_off else 0.0, 0.0)
            blocked_since = blocked_since if blocked_since is not None else t
            if t - blocked_since > nav["replan_after_s"]:
                replan_around_others(x, y)
                blocked_since = None
            continue
        blocked_since = None
        drive(0.5 if slow else 1.0, 1.5 * err + steer)


if __name__ == "__main__":
    main()
