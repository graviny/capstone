"""Supervisor: shared wireless channel, fleet server, inbound trucks, anomalies, physical actions, metrics.

Work: every inbound_period_s a truck drops parcels on the inbound pile (they
must be put away to their home slots); shelf anomalies arrive as a Poisson
process. A sorter's placement goes wrong with p_place_error, drawn from the
parcel id and attempt number so every policy sees the same execution errors.
Wrong picks (information errors) and place errors (execution) are counted apart.

Channel: robot messages arrive on channel 0. Each one occupies a single shared
FIFO channel (bandwidth_bps) for one or more attempts; an attempt is corrupted
with packet error rate 1-(1-BER)^bits (Gilbert-Elliott BER, worse through
racks) and retried up to max_retries. Survivors are forwarded to 100 + dst.
dst / src = -1 is the fleet server at the access point (ap_pos).

Out of band (not part of the modeled channel):
  - POSES: every robot's pose (overhead localization on the real setup)
  - ACT / ACT_RESULT: pick, place, upright, drop (the robot's own actuators)

Logs (logs/):
  comm_<tag>.csv       every message
  anomalies_<tag>.csv  every anomaly with creation / resolution time
  accuracy_<tag>.csv   share of items at their home slot, upright, every second
  summary_<tag>.json   episode summary (written at duration_s)
Set QUIT_AT_END=1 to close Webots after the episode (batch experiments).
"""
import csv
import heapq
import json
import math
import os
import random
import zlib
from pathlib import Path

from controller import Supervisor

ROOT = Path(__file__).resolve().parents[2]
STATS_PERIOD_S = 30.0
POSE_EVERY_STEPS = 4
REACH_M = 1.3                      # max robot-item distance for an action
SERVER = -1
UPRIGHT = [0, 0, 1, 0]
TIPPED = [0, 1, 0, 1.5708]         # lies along the shelf (x), so it reads as a long horizontal bar


def racks_crossed(p, q, racks):
    n = 0
    for r in racks:
        dy0, dy1 = p[1] - r["y"], q[1] - r["y"]
        if dy0 * dy1 < 0:
            x = p[0] + (q[0] - p[0]) * dy0 / (dy0 - dy1)
            if r["x_min"] <= x <= r["x_max"]:
                n += 1
    return n


class Shelves:
    """Ground-truth item state, moved through the supervisor API."""

    def __init__(self, sup, cfg):
        self.slots = {s["id"]: s for s in cfg["slots"]}
        self.items = {it["id"]: it for it in cfg["items"]}
        self.nodes = {i: sup.getFromDef(f"ITEM_{i}") for i in self.items}
        self.rack_top = cfg["rack_top"]
        self.z_up = cfg["item_size"][2] / 2
        self.z_down = cfg["item_size"][0] / 2
        self.slot_of = {i: it["home"] for i, it in self.items.items()}
        self.occupant = {s: None for s in self.slots}
        for i, s in self.slot_of.items():
            self.occupant[s] = i
        self.fallen = set()
        self.carried_by = {}               # item -> robot id

    def correct(self, i):
        return self.slot_of[i] == self.items[i]["home"] and i not in self.fallen and i not in self.carried_by

    def accuracy(self):
        return sum(self.correct(i) for i in self.items) / len(self.items)

    def _put(self, i, slot, fallen=False):
        x, y = self.slots[slot]["pos"]
        base = 0.0 if self.slots[slot]["kind"] == "pile" else self.rack_top
        self.nodes[i].getField("translation").setSFVec3f([x, y, base + (self.z_down if fallen else self.z_up)])
        self.nodes[i].getField("rotation").setSFRotation(TIPPED if fallen else UPRIGHT)
        self.slot_of[i], self.occupant[slot] = slot, i
        (self.fallen.add if fallen else self.fallen.discard)(i)

    def move(self, i, slot):
        self.occupant[self.slot_of[i]] = None
        self._put(i, slot)

    def swap(self, i, j):
        si, sj = self.slot_of[i], self.slot_of[j]
        self._put(i, sj)
        self._put(j, si)

    def tip(self, i):
        self._put(i, self.slot_of[i], fallen=True)

    def upright(self, i):
        self._put(i, self.slot_of[i])

    def pick(self, i, rid):
        self.occupant[self.slot_of[i]] = None
        self.slot_of[i] = None
        self.fallen.discard(i)
        self.carried_by[i] = rid
        self.nodes[i].getField("rotation").setSFRotation(UPRIGHT)

    def place(self, i, slot):
        del self.carried_by[i]
        self._put(i, slot)

    def carry_update(self, robot_nodes):
        for i, rid in self.carried_by.items():
            x, y, _ = robot_nodes[rid].getPosition()
            self.nodes[i].getField("translation").setSFVec3f([x, y, 0.45])

    def item_xy(self, i):
        p = self.nodes[i].getPosition()
        return p[0], p[1]

    def free_slots(self, kind="rack"):
        return [s for s, occ in self.occupant.items() if occ is None and self.slots[s]["kind"] == kind]

    def in_pile(self):
        return sum(1 for s in self.slot_of.values() if s and self.slots[s]["kind"] == "pile")


class Channel:
    """One shared FIFO wireless channel with BER-based loss and MAC retries.

    Other stations on the same Wi-Fi (robots elsewhere, cameras, handhelds) are
    background traffic: Poisson arrivals of bg_packet_bytes packets sized so they
    alone keep the channel busy a bg_util share of the time. Above 1.0 the channel
    is overloaded and the transmit buffer (max_queue_s) overflows.
    """

    def __init__(self, comm, rng, bg_util):
        self.c, self.rng = comm, rng
        self.free_at = 0.0
        self.ge_bad = False
        self.busy_s = 0.0              # airtime used in the current stats window
        self.bg_airtime = comm["bg_packet_bytes"] * 8 / comm["bandwidth_bps"] + comm["per_attempt_overhead_s"]
        self.bg_rate = bg_util / self.bg_airtime     # packets per second

    def poisson(self, mean):
        if mean > 30:
            return max(0, round(self.rng.gauss(mean, math.sqrt(mean))))
        k, p, limit = 0, 1.0, math.exp(-mean)
        while True:
            p *= self.rng.random()
            if p <= limit:
                return k
            k += 1

    def update_state(self, t, step_s):
        p = self.c["p_bad_to_good_per_s"] if self.ge_bad else self.c["p_good_to_bad_per_s"]
        if self.rng.random() < p * step_s:
            self.ge_bad = not self.ge_bad
        n = self.poisson(self.bg_rate * step_s)
        if n:
            busy = n * self.bg_airtime
            self.free_at = max(t, self.free_at) + busy
            self.busy_s += busy
            # Background senders have finite buffers too: the backlog cannot grow forever.
            self.free_at = min(self.free_at, t + self.c["max_queue_s"])

    def transmit(self, t, size, dist, n_racks):
        """Return (status, queue_delay, deliver_at, attempts)."""
        c = self.c
        ber = (c["ber_bad"] if self.ge_bad else c["ber_good"]) * c["ber_factor_per_rack"] ** n_racks
        per = 1 - (1 - min(ber, 1.0)) ** (8 * size)
        airtime = size * 8 / c["bandwidth_bps"] + c["per_attempt_overhead_s"]
        start = max(t, self.free_at)
        if start - t > c["max_queue_s"]:
            return "drop_queue", start - t, start, 0
        if dist > c["range_m"]:
            ok, attempts = False, 1 + c["max_retries"]
        else:
            attempts, ok = 0, False
            while attempts < 1 + c["max_retries"] and not ok:
                attempts += 1
                ok = self.rng.random() >= per
        self.free_at = start + attempts * airtime
        self.busy_s += attempts * airtime
        status = "ok" if ok else ("drop_range" if dist > c["range_m"] else "drop_loss")
        return status, start - t, self.free_at + c["base_latency_s"], attempts


def main():
    cfg = json.loads((ROOT / "scenario.json").read_text(encoding="utf-8"))
    cfg["traffic"]["policy"] = os.environ.get("POLICY", cfg["traffic"]["policy"])
    comm, ep, fleet = cfg["comm"], cfg["episode"], cfg["fleet"]
    rng = random.Random(cfg["seed"])
    anomaly_rng = random.Random(cfg["seed"] + 7)   # same anomalies for every policy
    server_rng = random.Random(cfg["seed"] + 13)
    ap = tuple(comm["ap_pos"])

    sup = Supervisor()
    dt = int(sup.getBasicTimeStep())
    step_s = dt / 1000.0
    emitter = sup.getDevice("emitter")
    receiver = sup.getDevice("receiver")
    receiver.enable(dt)
    nodes = {r["id"]: sup.getFromDef(r["def"]) for r in cfg["robots"]}
    roles = {r["id"]: r["role"] for r in cfg["robots"]}
    shelves = Shelves(sup, cfg)
    bg_util = float(os.environ.get("BG_UTIL", comm["bg_util"]))
    channel = Channel(comm, rng, bg_util)

    n_team = sum(r != "transport" for r in roles.values())
    # Batch runs and the interactive window write separate logs, so both can run at once.
    mode = "batch" if os.environ.get("QUIT_AT_END") == "1" else "gui"
    tag = (f"{n_team}team_{len(nodes) - n_team}amr_bg{round(100 * bg_util)}_"
           f"{cfg['traffic']['policy']}_seed{cfg['seed']}_{mode}")
    (ROOT / "logs").mkdir(exist_ok=True)
    comm_file = open(ROOT / "logs" / f"comm_{tag}.csv", "w", newline="", encoding="utf-8")
    comm_log = csv.writer(comm_file)
    comm_log.writerow(["t_sent", "src", "dst", "seq", "kind", "size", "dist", "racks",
                       "ge_bad", "attempts", "status", "queue_delay", "total_delay"])
    acc_file = open(ROOT / "logs" / f"accuracy_{tag}.csv", "w", newline="", encoding="utf-8")
    acc_log = csv.writer(acc_file)
    acc_log.writerow(["t", "accuracy", "open_anomalies", "in_pile"])

    anomalies = []
    open_by_item = {}
    totals = {"msgs": 0, "bytes": 0, "dropped": 0, "team_msgs": 0, "team_bytes": 0,
              "team_delay_sum": 0.0, "team_delivered": 0, "wrong_picks": 0, "picks": 0,
              "place_errors": 0, "putaways": 0, "arrived": 0, "transport_trips": 0, "busy_s": 0.0}
    from_pile = set()              # items picked off the pile and not yet shelved at home
    place_count = {}               # item -> placements so far (keys the deterministic error draw)

    def place_error(item):
        """Same parcel, same placement attempt -> same outcome under every policy."""
        k = place_count[item] = place_count.get(item, 0) + 1
        u = zlib.crc32(f"{cfg['seed']}-{item}-{k}".encode()) / 2 ** 32
        return u < ep["p_place_error"], u

    def inbound(t, n):
        """A truck drops n parcels (taken off the shelves: sold, now restocked) on the pile."""
        cands = [i for i in shelves.items if shelves.correct(i)]
        free = shelves.free_slots("pile")
        for i in anomaly_rng.sample(cands, min(n, len(cands), len(free))):
            slot = anomaly_rng.choice(free)
            free.remove(slot)
            shelves.move(i, slot)
            new_anomaly(t, i, "inbound")
            totals["arrived"] += 1

    def new_anomaly(t, item, kind):
        if item in open_by_item:
            return
        a = {"id": len(anomalies), "item": item, "kind": kind, "t_created": t, "t_resolved": None}
        anomalies.append(a)
        open_by_item[item] = a

    def spawn_anomaly(t):
        cands = [i for i in shelves.items if shelves.correct(i)]
        if not cands:
            return
        i = anomaly_rng.choice(cands)
        u = anomaly_rng.random()
        if u < ep["p_fallen"]:
            shelves.tip(i)
            new_anomaly(t, i, "fallen")
            return
        if u < ep["p_fallen"] + ep["p_swap"]:
            block = shelves.slot_of[i][:3], int(shelves.slot_of[i][3:]) // 4
            partners = [j for j in cands if j != i and shelves.slot_of[j][:3] == block[0]
                        and int(shelves.slot_of[j][3:]) // 4 == block[1]]
            if partners:
                j = anomaly_rng.choice(partners)
                shelves.swap(i, j)
                new_anomaly(t, i, "swapped")
                new_anomaly(t, j, "swapped")
                return
        free = shelves.free_slots()
        same = [s for s in free if shelves.slots[s]["type"] == shelves.items[i]["type"]]
        pool = same if same and anomaly_rng.random() < ep["p_next_to_lookalike"] else free
        if pool:
            shelves.move(i, anomaly_rng.choice(pool))
            new_anomaly(t, i, "misplaced")

    def reply(rid, payload):
        emitter.setChannel(100 + rid)
        emitter.send(json.dumps({"kind": "ACT_RESULT", **payload}))

    def handle_act(t, msg):
        rid, op = msg["src"], msg["op"]
        rx, ry, _ = nodes[rid].getPosition()
        item, ok, exchanged = msg.get("item"), False, False
        held = next((i for i, r in shelves.carried_by.items() if r == rid), None)
        result = {}
        if op in ("pick", "upright"):
            ix, iy = shelves.item_xy(item)
            near = math.hypot(ix - rx, iy - ry) < REACH_M and item not in shelves.carried_by
            if op == "pick" and near and held is None:
                if shelves.slots[shelves.slot_of[item]]["kind"] == "pile":
                    from_pile.add(item)
                shelves.pick(item, rid)
                ok = True
            elif op == "upright" and near and item in shelves.fallen:
                shelves.upright(item)
                ok = True
            if ok:
                totals["picks"] += 1
                if item != msg.get("intended"):
                    totals["wrong_picks"] += 1
                    print(f"[sup t={t:.0f}] WRONG {op}: robot_{rid} took {item}, wanted {msg.get('intended')}")
        elif op == "place" and held is not None:
            slot = msg["slot"]
            sx, sy = shelves.slots[slot]["pos"]
            if math.hypot(sx - rx, sy - ry) < REACH_M:
                occupant = shelves.occupant[slot]
                if occupant is not None and occupant not in shelves.fallen:
                    # Slot taken: put ours in, take the occupant along.
                    shelves.pick(occupant, rid)
                    shelves.carried_by.pop(occupant)
                    shelves.place(held, slot)
                    shelves.carried_by[occupant] = rid
                    exchanged = True
                    item = occupant
                    ok = True
                elif occupant is None:
                    err, u = place_error(held)
                    if err:
                        # Execution error: next free slot on the same face, else tipped in place.
                        k = int(slot[3:])
                        side = [f"{slot[:3]}{k + d:02d}" for d in ((-1, 1) if u < ep["p_place_error"] / 2 else (1, -1))]
                        nb = next((s for s in side if s in shelves.occupant and shelves.occupant[s] is None), None)
                        totals["place_errors"] += 1
                        print(f"[sup t={t:.0f}] PLACE ERROR: robot_{rid} put {held} "
                              f"{'into ' + nb if nb else 'tipped into ' + slot}, meant {slot}")
                        shelves.place(held, nb or slot)
                        if nb is None:
                            shelves.tip(held)
                    else:
                        shelves.place(held, slot)
                    item, ok = held, True
                if ok and not exchanged:
                    result = {"actual_slot": shelves.slot_of[held], "fallen": held in shelves.fallen}
                if ok and shelves.correct(held) and held in from_pile:
                    from_pile.discard(held)
                    totals["putaways"] += 1
                if ok and not shelves.correct(held):
                    new_anomaly(t, held, "fallen" if held in shelves.fallen else "misplaced")
        elif op == "drop" and held is not None and shelves.free_slots():
            slot = min(shelves.free_slots(), key=lambda s: math.hypot(
                shelves.slots[s]["pos"][0] - rx, shelves.slots[s]["pos"][1] - ry))
            shelves.place(held, slot)
            item, ok = held, True
            if slot != shelves.items[held]["home"]:
                new_anomaly(t, held, "misplaced")
        # actual_slot / fallen describe what a confirmation photo would show; the sorter
        # itself does not use them.
        reply(rid, {"op": op, "ok": ok, "item": item, "slot": msg.get("slot"), "exchanged": exchanged, **result})

    pending = []                   # heap of (deliver_time, order, msg)
    order = 0

    def submit(t, msg):
        nonlocal order
        p = ap if msg["src"] == SERVER else nodes[msg["src"]].getPosition()
        q = ap if msg["dst"] == SERVER else nodes[msg["dst"]].getPosition()
        dist = math.hypot(p[0] - q[0], p[1] - q[1])
        # The access point hangs from the ceiling, so only robot-to-robot links pass through racks.
        n_racks = 0 if SERVER in (msg["src"], msg["dst"]) else racks_crossed(p, q, cfg["racks"])
        status, qdelay, deliver_at, attempts = channel.transmit(t, msg["size"], dist, n_racks)
        if status == "ok":
            heapq.heappush(pending, (deliver_at, order, msg))
            order += 1
        team = msg["src"] != SERVER and roles[msg["src"]] != "transport"
        totals["msgs"] += 1
        totals["bytes"] += msg["size"]
        totals["dropped"] += status != "ok"
        window["sent"] += 1
        window["dropped"] += status != "ok"
        if team:
            totals["team_msgs"] += 1
            totals["team_bytes"] += msg["size"]
            if status == "ok":
                totals["team_delay_sum"] += deliver_at - msg["t_sent"]
                totals["team_delivered"] += 1
                window["team_delays"].append(deliver_at - msg["t_sent"])
        comm_log.writerow([f"{msg['t_sent']:.3f}", msg["src"], msg["dst"], msg["seq"], msg["kind"],
                           msg["size"], f"{dist:.2f}", n_racks, int(channel.ge_bad), attempts, status,
                           f"{qdelay:.3f}", f"{deliver_at - msg['t_sent']:.3f}"])

    def server_receive(t, msg):
        # Fleet server: hand the next station to a transport that asks for work.
        if msg["kind"] == "TASK_REQ":
            if msg["payload"].get("done"):
                totals["transport_trips"] += 1
            submit(t, {"src": SERVER, "dst": msg["src"], "seq": msg["seq"], "kind": "TASK",
                       "size": fleet["task_bytes"], "t_sent": t,
                       "payload": {"station": server_rng.choice(cfg["stations"])}})

    window = {"sent": 0, "dropped": 0, "team_delays": []}
    next_stats, next_acc = STATS_PERIOD_S, 0.0
    step_count = 0
    finished = False

    while sup.step(dt) != -1:
        t = sup.getTime()
        step_count += 1
        if step_count == 1:
            # Field changes before the first step are not applied reliably.
            inbound(t, ep["initial_inbound"])
            next_truck = t + ep["inbound_period_s"]
        channel.update_state(t, step_s)
        if not finished and t >= next_truck:
            inbound(t, ep["inbound_batch"])
            next_truck = t + ep["inbound_period_s"]
        if not finished and anomaly_rng.random() < ep["anomaly_rate_per_s"] * step_s:
            spawn_anomaly(t)

        while receiver.getQueueLength() > 0:
            msg = json.loads(receiver.getString())
            receiver.nextPacket()
            if msg["kind"] == "ACT":
                handle_act(t, msg)
            else:
                submit(t, msg)

        while pending and pending[0][0] <= t:
            _, _, msg = heapq.heappop(pending)
            msg["t_delivered"] = t
            if msg["dst"] == SERVER:
                server_receive(t, msg)
            else:
                emitter.setChannel(100 + msg["dst"])
                emitter.send(json.dumps(msg))

        shelves.carry_update(nodes)

        # Localization infrastructure (overhead camera on the real setup).
        if step_count % POSE_EVERY_STEPS == 0:
            poses = {}
            for rid, node in nodes.items():
                x, y, _ = node.getPosition()
                rot = node.getOrientation()
                poses[rid] = [round(x, 3), round(y, 3), round(math.atan2(rot[3], rot[0]), 3)]
            packet = json.dumps({"kind": "POSES", "t": t, "poses": poses})
            for rid in nodes:
                emitter.setChannel(100 + rid)
                emitter.send(packet)

        if t >= next_acc:
            for item, a in list(open_by_item.items()):
                if shelves.correct(item):
                    a["t_resolved"] = t
                    del open_by_item[item]
            acc_log.writerow([f"{t:.1f}", f"{shelves.accuracy():.4f}", len(open_by_item), shelves.in_pile()])
            next_acc = t + 1.0

        if t >= next_stats:
            d = window["team_delays"]
            print(f"[sup t={t:.0f}s] accuracy={shelves.accuracy():.3f} pile={shelves.in_pile()} "
                  f"putaways={totals['putaways']} open={len(open_by_item)} wrong_pick={totals['wrong_picks']} "
                  f"place_err={totals['place_errors']} trips={totals['transport_trips']} | "
                  f"util={channel.busy_s / STATS_PERIOD_S:.0%} msgs={window['sent']} drop={window['dropped']} "
                  f"team_delay={sum(d) / len(d) if d else 0:.2f}s backlog={max(0.0, channel.free_at - t):.1f}s")
            totals["busy_s"] += channel.busy_s
            channel.busy_s = 0.0
            comm_file.flush()
            acc_file.flush()
            window = {"sent": 0, "dropped": 0, "team_delays": []}
            next_stats = t + STATS_PERIOD_S

        if not finished and t >= ep["duration_s"]:
            finished = True
            comm_file.flush()
            acc_file.flush()
            with open(ROOT / "logs" / f"anomalies_{tag}.csv", "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=["id", "item", "kind", "t_created", "t_resolved"])
                w.writeheader()
                w.writerows(anomalies)
            resolved = [a["t_resolved"] - a["t_created"] for a in anomalies if a["t_resolved"] is not None]
            with open(ROOT / "logs" / f"accuracy_{tag}.csv", encoding="utf-8") as f:
                acc_rows = list(csv.reader(f))[1:]
            summary = {
                "tag": tag, "duration_s": t,
                "mean_accuracy": sum(float(r[1]) for r in acc_rows) / max(1, len(acc_rows)),
                "mean_pile": sum(int(r[3]) for r in acc_rows) / max(1, len(acc_rows)),
                "anomalies": len(anomalies), "resolved": len(resolved),
                "mean_repair_s": sum(resolved) / len(resolved) if resolved else None,
                "channel_util": (totals["busy_s"] + channel.busy_s) / t,
                "team_mean_delay_s": totals["team_delay_sum"] / max(1, totals["team_delivered"]),
                **{k: v for k, v in totals.items() if k not in ("busy_s", "team_delay_sum")},
            }
            (ROOT / "logs" / f"summary_{tag}.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
            print("[sup] SUMMARY " + json.dumps(summary))
            if os.environ.get("QUIT_AT_END") == "1":
                sup.simulationQuit(0)


if __name__ == "__main__":
    main()
