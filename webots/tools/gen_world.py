"""Generate the warehouse world (worlds/warehouse.wbt) and scenario.json.

Task: inbound putaway plus shelf upkeep. Trucks drop batches of parcels on an
inbound pile; scanner robots (patrol role) read their labels and tell sorter
robots (recovery role) which parcel to take to which shelf slot. Parcels of one
color look identical; only the label (ID) tells them apart, so in a dense pile
the sorter needs evidence to pick the right one. Shelf anomalies (misplaced,
tipped, swapped) still happen at a lower rate, and sorters occasionally put a
parcel into the wrong slot themselves.

Transport AMRs share the wireless channel (fleet status / task messages).

Usage: python tools/gen_world.py --patrols 3 --recoveries 8 --transports 6 --seed 0 [--policy proposed]
"""
import argparse
import json
import math
import stat
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

FLOOR_X, FLOOR_Y = 20.0, 14.0
ROW_YS = [-4.5, -1.5, 1.5, 4.5]    # rack row centers
RACK_X0, RACK_X1 = -3.0, 7.0
ROW_HALF_DEPTH = 0.3
RACK_TOP = 0.12
PANEL_HEIGHT = 0.7                 # center panel blocks view into the next aisle
SLOT_PITCH = 0.5
SLOTS_PER_SIDE = 20                # 5 blocks of 4 slots, one color per block
BLOCK = 4
ITEMS_PER_BLOCK = 3                # last slot of each block stays free
ITEM_SIZE = (0.10, 0.10, 0.30)         # tall and thin: a tipped item lies clearly sideways
APPROACH_OFFSET = 0.6              # where a sorter stops to handle a rack slot
PATROL_LANE = 0.95                 # inspection passes run this far from the rack center
AISLE_YS = [-5.9, -3.0, 0.0, 3.0, 5.9]

# Inbound pile: two columns of floor cells west of the racks, scanned northbound from
# PILE_LANE_X (east side) and southbound from PILE_WEST_LANE_X.
PILE_XS = [-7.1, -7.35]
PILE_Y0, PILE_Y1, PILE_PITCH = -6.0, 6.0, 0.25
PILE_ZONE = {"x_min": -7.5, "x_max": -6.95, "y_min": -6.15, "y_max": 6.15}
PILE_APPROACH_X = -6.45
PILE_LANE_X = -6.2
PILE_WEST_LANE_X = -7.95

PATROL_HOMES = [(-4.85, -6.2), (-4.85, 6.2), (8.4, 0.0), (8.4, -3.0), (8.4, 3.0)]
RECOVERY_HOMES = [(x, y) for x in (-4.3, -5.4) for y in (-0.6, 0.6, -1.8, 1.8, -3.0, 3.0, -4.2, 4.2, -5.4, 5.4)]
TRANSPORT_STARTS = [(9.3, y) for y in (-6.0, -4.8, -3.6, -2.4, -1.2, 0.0, 1.2, 2.4, 3.6, 4.8, 6.0)]
STATIONS = ([[8.4, y] for y in (-6.0, -3.0, 0.0, 3.0, 6.0)]
            + [[-4.85, y] for y in (-6.3, 6.3)] + [[2.0, y] for y in AISLE_YS])
CAMERA_FOV = 1.08                  # OV5647 Pi camera, 62 deg

# Items of one type look identical; only the label (ID) differs.
ITEM_TYPES = {
    "blue":  (0.15, 0.35, 0.8),
    "red":   (0.8, 0.2, 0.15),
    "brown": (0.55, 0.38, 0.2),
    "green": (0.2, 0.6, 0.25),
}

ROBOT_COLORS = {"patrol": (0.9, 0.6, 0.1), "recovery": (0.2, 0.4, 0.9), "transport": (0.55, 0.55, 0.55)}


def box_solid(name, pos, size, color, model="", recog=False, bounding=True, def_name=""):
    x, y, z = pos
    sx, sy, sz = size
    r, g, b = color
    geom = f"Box {{ size {sx} {sy} {sz} }}"
    s = f"DEF {def_name} " if def_name else ""
    s += (
        f"Solid {{\n  translation {x:.3f} {y:.3f} {z:.3f}\n  name \"{name}\"\n"
        f"  children [ Shape {{ appearance PBRAppearance {{ baseColor {r} {g} {b} roughness 0.8 metalness 0 }} "
        f"geometry {geom} }} ]\n"
    )
    if model:
        s += f"  model \"{model}\"\n"
    if recog:
        s += f"  recognitionColors [ {r} {g} {b} ]\n"
    if bounding:
        s += f"  boundingObject {geom}\n"
    return s + "}\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--patrols", type=int, default=3)
    ap.add_argument("--recoveries", type=int, default=8)
    ap.add_argument("--transports", type=int, default=6)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--policy", default="proposed", choices=["full", "none", "pull", "cache", "proposed"])
    args = ap.parse_args()
    for name, n, cap in (("patrols", args.patrols, PATROL_HOMES), ("recoveries", args.recoveries, RECOVERY_HOMES),
                         ("transports", args.transports, TRANSPORT_STARTS)):
        if not 0 <= n <= len(cap):
            raise SystemExit(f"--{name} must be between 0 and {len(cap)}")

    parts = [
        "#VRML_SIM R2025a utf8\n",
        'EXTERNPROTO "../protos/SimpleBot.proto"\n\n',
        "WorldInfo {\n  basicTimeStep 32\n"
        '  contactProperties [ ContactProperties { material2 "slippery" coulombFriction [ 0 ] } ]\n}\n',
        "Viewpoint {\n  orientation -0.45 0.45 0.77 1.82\n  position 1 -15 17\n}\n",
        "Background { skyColor [ 0.75 0.8 0.85 ] }\n",
        "DirectionalLight { direction -0.3 0.4 -1 intensity 2.5 castShadows TRUE }\n",
        "DirectionalLight { direction 0.4 -0.3 -1 intensity 1 }\n",
        f"Solid {{\n  name \"floor\"\n  children [ Shape {{ appearance PBRAppearance {{ baseColor 0.55 0.55 0.52 roughness 1 metalness 0 }} "
        f"geometry Plane {{ size {FLOOR_X} {FLOOR_Y} }} }} ]\n  boundingObject Plane {{ size {FLOOR_X} {FLOOR_Y} }}\n}}\n",
        # Inbound area marking (visual only).
        f"Solid {{\n  name \"inbound_mark\"\n  translation -7.22 0 0.001\n  children [ Shape {{ appearance PBRAppearance "
        f"{{ baseColor 0.85 0.75 0.3 roughness 1 metalness 0 }} geometry Plane {{ size 0.7 12.4 }} }} ]\n}}\n",
    ]

    hx, hy, wall_c = FLOOR_X / 2, FLOOR_Y / 2, (0.75, 0.75, 0.7)
    parts += [
        box_solid("wall_n", (0, hy, 0.25), (FLOOR_X, 0.1, 0.5), wall_c),
        box_solid("wall_s", (0, -hy, 0.25), (FLOOR_X, 0.1, 0.5), wall_c),
        box_solid("wall_e", (hx, 0, 0.25), (0.1, FLOOR_Y, 0.5), wall_c),
        box_solid("wall_w", (-hx, 0, 0.25), (0.1, FLOOR_Y, 0.5), wall_c),
    ]

    slots, items = [], []
    type_names = list(ITEM_TYPES)
    item_z = RACK_TOP + ITEM_SIZE[2] / 2
    rack_cx, rack_len = (RACK_X0 + RACK_X1) / 2, RACK_X1 - RACK_X0
    for ri, ry in enumerate(ROW_YS):
        parts.append(box_solid(f"rack{ri}_base", (rack_cx, ry, RACK_TOP / 2),
                               (rack_len, 2 * ROW_HALF_DEPTH, RACK_TOP), (0.45, 0.45, 0.5)))
        parts.append(box_solid(f"rack{ri}_panel", (rack_cx, ry, PANEL_HEIGHT / 2),
                               (rack_len, 0.04, PANEL_HEIGHT), (0.35, 0.4, 0.5)))
        for side, sy, ay in (("s", ry - 0.17, ry - APPROACH_OFFSET), ("n", ry + 0.17, ry + APPROACH_OFFSET)):
            for si in range(SLOTS_PER_SIDE):
                sx = RACK_X0 + SLOT_PITCH * (si + 0.5)
                block = si // BLOCK
                block_type = type_names[(ri * 2 + (side == "n") + block) % len(type_names)]
                slot_id = f"r{ri}{side}{si:02d}"
                home_of = None
                if si % BLOCK < ITEMS_PER_BLOCK:
                    home_of = f"{block_type}_{len([i for i in items if i['type'] == block_type]):02d}"
                    items.append({"id": home_of, "type": block_type, "home": slot_id})
                    parts.append(box_solid(home_of, (sx, sy, item_z), ITEM_SIZE, ITEM_TYPES[block_type],
                                           model=home_of, recog=True, def_name=f"ITEM_{home_of}"))
                slots.append({"id": slot_id, "kind": "rack", "pos": [round(sx, 3), round(sy, 3)],
                              "approach": [round(sx, 3), round(ay, 3)],
                              "type": block_type, "home_of": home_of})

    n_pile_rows = int(round((PILE_Y1 - PILE_Y0) / PILE_PITCH)) + 1
    for ci, px in enumerate(PILE_XS):
        for k in range(n_pile_rows):
            py = PILE_Y0 + k * PILE_PITCH
            # Id layout "p<col>x<row>" keeps neighbors at row +-1, like rack slots.
            slots.append({"id": f"p{ci}x{k:02d}", "kind": "pile", "pos": [px, round(py, 3)],
                          "approach": [PILE_APPROACH_X, round(py, 3)], "type": None, "home_of": None})

    roster = ([("patrol", PATROL_HOMES[k], 0.0) for k in range(args.patrols)]
              + [("recovery", RECOVERY_HOMES[k], 0.0) for k in range(args.recoveries)]
              + [("transport", TRANSPORT_STARTS[k], math.pi) for k in range(args.transports)])
    # Inspection passes: drive along a face with it on the camera (left) side.
    # interval_s: how often a face should be looked at.
    faces = [{"id": "pile", "start": [PILE_LANE_X, PILE_Y0 - 0.3], "end": [PILE_LANE_X, PILE_Y1 + 0.3],
              "interval_s": 40.0},
             {"id": "pile_w", "start": [PILE_WEST_LANE_X, PILE_Y1 + 0.3], "end": [PILE_WEST_LANE_X, PILE_Y0 - 0.3],
              "interval_s": 40.0}]
    for ri, ry in enumerate(ROW_YS):
        faces.append({"id": f"r{ri}s", "start": [RACK_X0 - 0.6, ry - PATROL_LANE],
                      "end": [RACK_X1 + 0.6, ry - PATROL_LANE], "interval_s": 240.0})
        faces.append({"id": f"r{ri}n", "start": [RACK_X1 + 0.6, ry + PATROL_LANE],
                      "end": [RACK_X0 - 0.6, ry + PATROL_LANE], "interval_s": 240.0})
    pile_faces = [f["id"] for f in faces if f["id"].startswith("pile")]
    rack_faces = [f["id"] for f in faces if f["id"] not in pile_faces]

    robots = []
    for i, (role, start, yaw) in enumerate(roster):
        robots.append({"id": i, "def": f"R{i}", "name": f"robot_{i}", "role": role,
                       "start": list(start), "home": list(start)})
        if role == "patrol":
            # The first scanner is dedicated to the inbound pile; the rest share the racks.
            alone = args.patrols == 1
            robots[-1]["faces"] = pile_faces + rack_faces if alone else (pile_faces if i == 0 else rack_faces)
        c = ROBOT_COLORS[role]
        parts.append(
            f"DEF R{i} SimpleBot {{\n  translation {start[0]} {start[1]} 0\n  rotation 0 0 1 {yaw:.4f}\n"
            f"  name \"robot_{i}\"\n  controller \"agent\"\n  controllerArgs [ \"--id\" \"{i}\" ]\n"
            f"  color {c[0]} {c[1]} {c[2]}\n  receiverChannel {100 + i}\n  cameraFov {CAMERA_FOV}\n}}\n"
        )

    parts.append(
        "Robot {\n  name \"supervisor\"\n  supervisor TRUE\n  controller \"supervisor\"\n"
        "  children [ Emitter { name \"emitter\" channel 100 range -1 } Receiver { name \"receiver\" channel 0 } ]\n}\n"
    )

    scenario = {
        "seed": args.seed,
        "world": {"x_min": -FLOOR_X / 2, "x_max": FLOOR_X / 2, "y_min": -FLOOR_Y / 2, "y_max": FLOOR_Y / 2},
        "racks": [{"y": y, "x_min": RACK_X0, "x_max": RACK_X1, "half_depth": ROW_HALF_DEPTH} for y in ROW_YS],
        "obstacles": [PILE_ZONE],
        "pile": {"zone": PILE_ZONE, "lane_x": PILE_LANE_X},
        "item_size": list(ITEM_SIZE),
        "rack_top": RACK_TOP,
        "slots": slots,
        "items": items,
        "robots": robots,
        "faces": faces,
        "stations": STATIONS,
        "episode": {
            "duration_s": 600,
            "initial_inbound": 12,
            "inbound_period_s": 50.0,       # a truck drops a batch this often
            "inbound_batch": 4,
            "anomaly_rate_per_s": 1 / 90,   # shelf anomalies (Poisson)
            "p_fallen": 0.3,
            "p_swap": 0.3,                  # two lookalikes in one block trade places
            "p_next_to_lookalike": 0.6,     # misplaced: into a free slot of a same-color block
            "p_place_error": 0.03,          # sorter puts a parcel next to / tipped in the target slot
        },
        "perception": {
            # Labels are ~1 cm tags: 20 px needed (7 cm tag measured at 2.6 m). Scanner 5 MP stills
            # (2592 px, 62 deg) -> ~1.0 m; sorter 480p video -> ~0.33 m, i.e. it cannot read them.
            "patrol_read_label_m": 1.0,
            "slot_match_m": 0.2,
            "fallen_tilt_cos": 0.7,
            "candidate_radius_m": 0.6,      # sorter: same-color items this close to C are candidates
            "c_noise_m": 0.25,              # along-face error of the reported position
        },
        "comm": {
            # Measured on the Pi <-> laptop link (phone hotspot, 2.4 GHz): 22-28 Mbit/s,
            # 0-2 % loss, 14-18 ms ping through the access point.
            "bandwidth_bps": 22_000_000,    # shared by every robot and the fleet server
            "per_attempt_overhead_s": 0.0003,   # preamble, backoff, MAC ACK
            "bg_util": 0.6,                 # other stations' airtime share (BG_UTIL env var overrides)
            "bg_packet_bytes": 1500,
            "max_retries": 3,               # MAC retransmissions after a corrupted attempt
            "base_latency_s": 0.007,         # half the measured ping
            "range_m": 14.0,
            "ber_good": 2e-6,               # bit error rate; packet error = 1-(1-ber)^bits
            "ber_bad": 3e-5,
            "ber_factor_per_rack": 3.0,     # each rack panel on the link multiplies the BER
            "p_good_to_bad_per_s": 0.02,
            "p_bad_to_good_per_s": 0.2,
            "ap_pos": [1.0, 0.0],           # fleet server access point, ceiling-mounted (racks don't block it)
            "max_queue_s": 2.0,             # transmit buffer: drop if it would wait longer
        },
        "fleet": {                          # transport AMR traffic, VDA 5050-like sizes
            "status_hz": 2.0,
            "status_bytes": 1500,
            "task_req_bytes": 200,
            "task_bytes": 800,
            "task_timeout_s": 3.0,
            "dwell_s": 2.0,
        },
        "traffic": {
            "policy": args.policy,
            # Evidence sizes: Pi camera (OV5647) 1280x720 frames taken 0.78 m from the shelf
            # (~1370 px/m), JPEG q80 at ~1.2 bit/pixel.
            "header_bytes": 160,            # E + N + C
            "evidence_bytes": {
                "none": 0,
                "thumb": 1_800,             # 1.0 x 0.3 m strip (target + neighbors), 200x60 px
                "crop": 4_200,              # target only, label legible, 137x205 px
                "frame": 138_000,           # full 1280x720 frame (reference only)
            },
            "ack_bytes": 40,
            "req_bytes": 60,                # sorter asks for evidence (pull)
            "report_timeout_s": 90.0,       # re-report if no DONE by then
            "evidence_timeout_s": 4.0,      # pull: give up waiting and guess
            "act_s": 2.0,                   # pick / place / upright duration
        },
        "proposed": {
            "wrong_pick_cost_s": 60.0,      # redo + the anomaly a wrong pick creates
            "task_s": 40.0,                 # typical time one sorter task takes
            "externality": 50,              # stations that wait behind our airtime
            "service_s": 0.00085,           # airtime of a typical 1500 B packet (congestion = 1 + wait / this)
            "p_detect_per_frame": 0.15,     # chance a sorter notices the target in one frame
            "p_remember": 0.97,
            "seen_range_m": 2.0,
            "claim_range_m": 6.0,           # tell scanners this close that an item is taken
            "undetected_delay_s": 120.0,    # how long a misplacement stays unnoticed without a photo
        },
        "nav": {"speed_mps": 0.3, "wheel_radius": 0.033, "reach_m": 0.2, "goal_reach_m": 0.1,
                "avoid_dist_m": 0.6, "stop_dist_m": 0.3, "replan_after_s": 2.0},
    }

    (ROOT / "worlds").mkdir(exist_ok=True)
    world = ROOT / "worlds" / "warehouse.wbt"
    # Kept read-only so closing Webots cannot save an older layout over it.
    if world.exists():
        world.chmod(stat.S_IWRITE | stat.S_IREAD)
    world.write_text("".join(parts), encoding="utf-8")
    world.chmod(stat.S_IREAD)
    (ROOT / "scenario.json").write_text(json.dumps(scenario, indent=2), encoding="utf-8")
    print(f"wrote warehouse.wbt: {args.patrols} patrol + {args.recoveries} recovery + {args.transports} transport, "
          f"{len(items)} items, {len(slots)} slots, policy={args.policy}")


if __name__ == "__main__":
    main()
