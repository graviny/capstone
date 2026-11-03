"""Visual parts of the warehouse world (used by gen_world.py).

Everything here is looks only: no bounding objects and no recognition, so the robots'
navigation and perception are the same as with plain boxes. Exceptions are the floor,
walls and the carton solids, whose bounding objects keep the sizes gen_world.py uses.
Photo textures: textures/webots/ (Webots project, Apache 2.0); text textures:
textures/gen/ (make_textures.py).
"""
import math
import random
import zlib

TEX = "../textures"


def f(v):
    return f"{v:.4f}".rstrip("0").rstrip(".") if isinstance(v, float) else str(v)


def vec(*v):
    return " ".join(f(float(x)) for x in v)


class Looks:
    """PBR appearances: DEF the first time, USE afterwards (the world is written in order)."""

    def __init__(self):
        self.done = set()

    def get(self, name, body):
        if name in self.done:
            return f"USE {name}"
        self.done.add(name)
        return f"DEF {name} PBRAppearance {{ {body} }}"

    def color(self, name, c, rough=0.7, metal=0.0, emissive=None, transparency=0.0):
        body = f"baseColor {vec(*c)} roughness {f(rough)} metalness {f(metal)}"
        if emissive:
            body += f" emissiveColor {vec(*emissive)} emissiveIntensity 2"
        if transparency:
            body += f" transparency {f(transparency)}"
        return self.get(name, body)

    def texture(self, name, url, scale=(1, 1), rough=0.8, metal=0.0, tint=(1, 1, 1), normal=None):
        body = (f"baseColor {vec(*tint)} baseColorMap ImageTexture {{ url [ \"{TEX}/{url}\" ] }} "
                f"roughness {f(rough)} metalness {f(metal)}")
        if normal:
            body += f" normalMap ImageTexture {{ url [ \"{TEX}/{normal}\" ] }}"
        if scale != (1, 1):
            body += f" textureTransform TextureTransform {{ scale {vec(*scale)} }}"
        return self.get(name, body)


def shape(app, geom):
    return f"Shape {{ appearance {app} geometry {geom} }}"


def box(sx, sy, sz):
    return f"Box {{ size {vec(sx, sy, sz)} }}"


def cyl(r, h):
    return f"Cylinder {{ radius {f(float(r))} height {f(float(h))} }}"


def pose(t, children, rot=None, def_name=None):
    s = f"DEF {def_name} " if def_name else ""
    s += f"Pose {{ translation {vec(*t)} "
    if rot:
        s += f"rotation {vec(*rot)} "
    return s + "children [ " + " ".join(children) + " ] }\n"


def block(looks, app, center, size, rot=None):
    return pose(center, [shape(app, box(*size))], rot)


# ---------------------------------------------------------------- building

def floor(looks, fx, fy):
    app = looks.texture("CONCRETE", "webots/formed_concrete_base_color.jpg", scale=(fx / 4, fy / 4), rough=0.95,
                        normal="webots/formed_concrete_normal.jpg")
    geom = f"Plane {{ size {vec(fx, fy)} }}"
    return f"Solid {{\n  name \"floor\"\n  children [ {shape(app, geom)} ]\n  boundingObject {geom}\n}}\n"


def yard(looks, depth=1.1):
    """Asphalt outside, one dock height below the warehouse floor."""
    app = looks.texture("ASPHALT", "webots/asphalt_base_color.jpg", scale=(12, 8), rough=1.0)
    return pose((0, 0, -depth), [shape(app, "Plane { size 60 40 }")])


def walls(looks, hx, hy, west_doors, east_doors, height=4.0, door_h=3.0, depth=1.1):
    """Corrugated-metal walls from the yard up, with dock openings (y ranges) on the west / east."""
    metal = looks.texture("WALL", "webots/corrugated_metal_base_color.jpg", scale=(6, 2), rough=0.6, metal=0.4,
                          tint=(0.85, 0.88, 0.9))
    concrete = looks.texture("DOCKFACE", "webots/formed_concrete_base_color.jpg", scale=(8, 1), rough=0.95)
    t, z0 = 0.15, -depth
    out, n = [], [0]

    def wall(cx, cy, sx, sy, z_lo, z_hi, app=metal, solid=True):
        n[0] += 1
        size = (sx, sy, z_hi - z_lo)
        c = (cx, cy, (z_lo + z_hi) / 2)
        if solid:
            out.append(f"Solid {{ name \"wall_{n[0]}\" translation {vec(*c)} children [ {shape(app, box(*size))} ] "
                       f"boundingObject {box(*size)} }}\n")
        else:
            out.append(block(looks, app, c, size))

    wall(0, hy + t / 2, 2 * hx + 2 * t, t, z0, height)
    wall(0, -hy - t / 2, 2 * hx + 2 * t, t, z0, height)
    for side, doors in ((-1, west_doors), (1, east_doors)):
        x = side * (hx + t / 2)
        edges = [-hy] + [v for d in sorted(doors) for v in d] + [hy]
        for y_lo, y_hi in zip(edges[0::2], edges[1::2]):
            if y_hi > y_lo:
                wall(x, (y_lo + y_hi) / 2, t, y_hi - y_lo, z0, height)
        for y_lo, y_hi in doors:
            w = y_hi - y_lo
            wall(x, (y_lo + y_hi) / 2, t, w, door_h, height)                # header over the door
            wall(x, (y_lo + y_hi) / 2, t, w, z0, 0.0, concrete, solid=False)  # dock face below the floor
            out.append(dock_door(looks, side, hx, (y_lo + y_hi) / 2, w, door_h))
    return "".join(out)


def dock_door(looks, side, hx, yc, w, door_h):
    rubber = looks.color("RUBBER", (0.08, 0.08, 0.08), rough=0.9)
    steel = looks.color("STEEL", (0.55, 0.57, 0.6), rough=0.4, metal=0.8)
    shutter = looks.texture("SHUTTER", "webots/corrugated_metal_base_color.jpg", scale=(4, 1), rough=0.5, metal=0.5,
                            tint=(0.7, 0.72, 0.75))
    xi, xo = side * (hx - 0.12), side * (hx + 0.2)
    parts = [
        # Rolled-up shutter and its guide rails, inside.
        pose((xi, yc, door_h + 0.18), [shape(shutter, cyl(0.2, w + 0.1))], rot=(1, 0, 0, math.pi / 2)),
        block(looks, steel, (xi, yc - w / 2 - 0.05, door_h / 2), (0.08, 0.06, door_h)),
        block(looks, steel, (xi, yc + w / 2 + 0.05, door_h / 2), (0.08, 0.06, door_h)),
        # Dock leveler plate and rubber bumpers / seals outside.
        block(looks, steel, (side * (hx - 0.4), yc, 0.006), (0.7, w - 0.2, 0.012)),
        block(looks, rubber, (xo, yc - w / 2 + 0.15, -0.25), (0.25, 0.25, 0.35)),
        block(looks, rubber, (xo, yc + w / 2 - 0.15, -0.25), (0.25, 0.25, 0.35)),
        block(looks, rubber, (side * (hx + 0.25), yc - w / 2 - 0.1, door_h / 2), (0.3, 0.2, door_h + 0.3)),
        block(looks, rubber, (side * (hx + 0.25), yc + w / 2 + 0.1, door_h / 2), (0.3, 0.2, door_h + 0.3)),
        block(looks, rubber, (side * (hx + 0.25), yc, door_h + 0.15), (0.3, w + 0.4, 0.3)),
    ]
    return "".join(parts)


def ceiling_lights(looks, xs, ys, z=3.8):
    lamp = looks.color("LAMP", (1, 1, 0.95), emissive=(1, 0.98, 0.9))
    housing = looks.color("LAMPHOUSING", (0.88, 0.88, 0.86), rough=0.5, metal=0.3)
    out = []
    for x in xs:
        for y in ys:
            out.append(pose((x, y, z), [shape(housing, box(1.4, 0.22, 0.06)),
                                        pose((0, 0, -0.035), [shape(lamp, box(1.3, 0.14, 0.01))])]))
    return "".join(out)


# ---------------------------------------------------------------- floor markings

def line(looks, app, x0, y0, x1, y1, w=0.05, z=0.004):
    length = math.hypot(x1 - x0, y1 - y0)
    yaw = math.atan2(y1 - y0, x1 - x0)
    return pose(((x0 + x1) / 2, (y0 + y1) / 2, z), [shape(app, f"Plane {{ size {vec(length, w)} }}")],
                rot=(0, 0, 1, yaw))


def dashed(looks, app, x0, y0, x1, y1, dash=0.6, gap=0.4, w=0.08):
    length = math.hypot(x1 - x0, y1 - y0)
    ux, uy = (x1 - x0) / length, (y1 - y0) / length
    out, s = [], 0.0
    while s < length:
        e = min(length, s + dash)
        out.append(line(looks, app, x0 + ux * s, y0 + uy * s, x0 + ux * e, y0 + uy * e, w))
        s = e + gap
    return "".join(out)


def hatch(looks, x0, x1, y0, y1, z=0.005):
    """Yellow / black safety stripes over a rectangle (stripes at 45 deg, clipped by drawing short ones)."""
    yellow = looks.color("HATCH_Y", (0.95, 0.78, 0.1), rough=0.9)
    dark = looks.color("HATCH_K", (0.1, 0.1, 0.1), rough=0.9)
    out = [pose(((x0 + x1) / 2, (y0 + y1) / 2, z - 0.0005), [shape(dark, f"Plane {{ size {vec(x1 - x0, y1 - y0)} }}")])]
    step, w = 0.3, 0.12
    k = x0 - (y1 - y0)
    while k < x1:
        # Segment of the line x - y = k - y0 inside the rectangle.
        a = (max(x0, k), max(y0, y0 + (x0 - k)))
        b = (min(x1, k + (y1 - y0)), min(y1, y0 + (x1 - k)))
        if b[0] > a[0] + 0.02:
            out.append(line(looks, yellow, a[0], a[1], b[0], b[1], w, z))
        k += step
    return "".join(out)


def outline(looks, app, cx, cy, sx, sy, w=0.04, z=0.004):
    x0, x1, y0, y1 = cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2
    return "".join(line(looks, app, *seg, w=w, z=z) for seg in
                   ((x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)))


# ---------------------------------------------------------------- racks

def rack(looks, ri, ry, x0, x1, half_depth, top, slots, rng, bay=2.5, levels=(0.95, 1.75), height=2.4):
    """Pallet racking around one row: uprights, beams, upper decks with stored pallets, slot labels.

    The robots only work the bottom level (rack top at `top`); the upper levels are decoration.
    """
    upright = looks.color("UPRIGHT", (0.1, 0.28, 0.6), rough=0.5, metal=0.5)
    beam = looks.color("BEAM", (0.95, 0.45, 0.08), rough=0.5, metal=0.4)
    deck = looks.color("DECK", (0.6, 0.62, 0.65), rough=0.5, metal=0.7, transparency=0.25)
    pine = looks.texture("PINE", "webots/rough_pine_base_color.jpg", rough=0.9)
    cardboard = looks.texture("CARDBOARD", "webots/cardboard_top_base_color.jpg", rough=0.85)
    wrap = looks.color("STRETCHWRAP", (0.9, 0.92, 0.95), rough=0.2, transparency=0.85)
    out = []
    xs = [x0 + k * bay for k in range(int(round((x1 - x0) / bay)) + 1)]
    for x in xs:
        for side in (-1, 1):
            out.append(block(looks, upright, (x, ry + side * half_depth, height / 2), (0.08, 0.08, height)))
    for xa, xb in zip(xs, xs[1:]):
        cx = (xa + xb) / 2
        for z in levels:
            for side in (-1, 1):
                out.append(block(looks, beam, (cx, ry + side * half_depth, z - 0.05), (xb - xa - 0.08, 0.05, 0.1)))
            out.append(block(looks, deck, (cx, ry, z + 0.01), (xb - xa - 0.08, 2 * half_depth, 0.02)))
            for px in (cx - 0.6, cx + 0.6):
                if rng.random() < 0.25:
                    continue
                out.append(block(looks, pine, (px, ry, z + 0.08), (1.0, 2 * half_depth - 0.06, 0.12)))
                h = rng.choice((0.35, 0.45, 0.55))
                out.append(block(looks, cardboard, (px, ry, z + 0.14 + h / 2), (0.92, 2 * half_depth - 0.12, h)))
                out.append(block(looks, wrap, (px, ry, z + 0.14 + h / 2), (0.95, 2 * half_depth - 0.09, h + 0.02)))
    # Location labels on the front edge of the bottom shelf.
    for s in slots:
        side = 1 if s["pos"][1] > ry else -1
        app = looks.texture(f"LBL_{s['id']}", f"gen/slot_{s['id']}.png", rough=0.6)
        rot = (0, 0, 1, math.pi) if side > 0 else None
        out.append(block(looks, app, (s["pos"][0], ry + side * (half_depth + 0.004), top / 2), (0.18, 0.006, 0.06),
                         rot=rot))
    return "".join(out)


def rack_base_app(looks):
    return looks.color("RACKBASE", (0.42, 0.44, 0.48), rough=0.5, metal=0.6)


def rack_panel_app(looks):
    return looks.color("RACKMESH", (0.5, 0.52, 0.55), rough=0.5, metal=0.6, transparency=0.35)


# ---------------------------------------------------------------- inbound equipment

def roller_bed(looks, x0, x1, y, width, top, pitch=0.12):
    roller = looks.color("ROLLER", (0.7, 0.72, 0.75), rough=0.3, metal=0.9)
    frame = looks.color("CONVFRAME", (0.25, 0.27, 0.3), rough=0.5, metal=0.6)
    out = [block(looks, frame, ((x0 + x1) / 2, y + s * (width / 2 + 0.03), top / 2), (x1 - x0, 0.05, top + 0.02))
           for s in (-1, 1)]
    out.append(block(looks, frame, ((x0 + x1) / 2, y, 0.02), (x1 - x0, width, 0.04)))
    x = x0 + pitch / 2
    while x < x1:
        out.append(pose((x, y, top - 0.025), [shape(roller, cyl(0.025, width))], rot=(1, 0, 0, math.pi / 2)))
        x += pitch
    return "".join(out)


def scanner_gate(looks, x, y, half_w=0.32, h=0.75):
    post = looks.color("GATEPOST", (0.95, 0.8, 0.1), rough=0.5, metal=0.3)
    head = looks.color("SCANHEAD", (0.12, 0.12, 0.14), rough=0.4)
    laser = looks.color("LASER", (1, 0.1, 0.05), emissive=(1, 0.05, 0.02))
    return "".join([
        block(looks, post, (x, y - half_w, h / 2), (0.06, 0.06, h)),
        block(looks, post, (x, y + half_w, h / 2), (0.06, 0.06, h)),
        block(looks, post, (x, y, h), (0.08, 2 * half_w + 0.06, 0.06)),
        block(looks, head, (x, y, h - 0.07), (0.14, 0.22, 0.08)),
        block(looks, laser, (x, y, 0.121), (0.01, 2 * half_w - 0.08, 0.002)),
        block(looks, head, (x, y + half_w - 0.04, 0.3), (0.06, 0.08, 0.1)),
    ])


def pusher(looks, x, y, toward=-1):
    """Pneumatic pusher on the far side of the belt, pushing toward the staging table (-y)."""
    body = looks.color("PUSHER", (0.2, 0.45, 0.75), rough=0.4, metal=0.5)
    rod = looks.color("STEEL", (0.55, 0.57, 0.6), rough=0.4, metal=0.8)
    pad = looks.color("RUBBER", (0.08, 0.08, 0.08), rough=0.9)
    return "".join([
        block(looks, body, (x, y, 0.2), (0.16, 0.14, 0.14)),
        pose((x, y + toward * 0.1, 0.2), [shape(rod, cyl(0.012, 0.08))], rot=(1, 0, 0, math.pi / 2)),
        block(looks, pad, (x, y + toward * 0.15, 0.2), (0.22, 0.02, 0.1)),
    ])


def sign(looks, name, center, size=(1.2, 0.6), facing="y", post_to=None):
    """Hanging sign. facing "y": readable from -y; "x": readable from -x."""
    app = looks.texture(f"SIGN_{name}", f"gen/sign_{name}.png", rough=0.6)
    chain = looks.color("STEEL", (0.55, 0.57, 0.6), rough=0.4, metal=0.8)
    w, h = size
    # A thin panel along x, turned so that its textured broad face looks the right way.
    rot = (0, 0, 1, math.pi) if facing == "y" else (0, 0, 1, math.pi / 2)
    out = [block(looks, app, center, (w, 0.03, h), rot=rot)]
    top = center[2] + h / 2
    z1 = post_to if post_to is not None else 3.8
    for dx in (-w / 2 + 0.1, w / 2 - 0.1):
        off = (dx, 0) if facing == "y" else (0, dx)
        out.append(pose((center[0] + off[0], center[1] + off[1], (top + z1) / 2),
                        [shape(chain, cyl(0.008, z1 - top))]))
    return "".join(out)


# ---------------------------------------------------------------- outbound equipment

def roll_cage(looks, x, y, w=0.7, d=0.8, h=1.6):
    frame = looks.color("CAGE", (0.7, 0.72, 0.74), rough=0.3, metal=0.9)
    mesh = looks.color("CAGEMESH", (0.75, 0.77, 0.8), rough=0.3, metal=0.9, transparency=0.7)
    wheel = looks.color("RUBBER", (0.08, 0.08, 0.08), rough=0.9)
    out = [block(looks, frame, (x, y, 0.12), (d, w, 0.04))]
    for sx in (-1, 1):
        for sy in (-1, 1):
            out.append(pose((x + sx * (d / 2 - 0.06), y + sy * (w / 2 - 0.06), 0.05),
                            [shape(wheel, cyl(0.05, 0.04))], rot=(1, 0, 0, math.pi / 2)))
            out.append(block(looks, frame, (x + sx * d / 2, y + sy * w / 2, 0.14 + h / 2), (0.03, 0.03, h)))
    for z in (0.6, 1.1, 0.14 + h):
        out.append(block(looks, frame, (x + d / 2, y, z), (0.03, w, 0.03)))
        for sy in (-1, 1):
            out.append(block(looks, frame, (x, y + sy * w / 2, z), (d, 0.03, 0.03)))
    out.append(block(looks, mesh, (x + d / 2, y, 0.14 + h / 2), (0.01, w, h)))
    for sy in (-1, 1):
        out.append(block(looks, mesh, (x, y + sy * w / 2, 0.14 + h / 2), (d, 0.01, h)))
    return "".join(out)


def truck(looks, def_name, x_rear, y, facing, cab_color, depth=1.1):
    """Box truck backed up to a dock door; its bed is level with the warehouse floor (z = 0).

    facing -1: the truck extends to -x (west dock), +1: to +x (east dock).
    """
    s = facing
    body = looks.color("TRUCKBOX", (0.93, 0.93, 0.95), rough=0.5, metal=0.2)
    inside = looks.color("TRUCKIN", (0.6, 0.62, 0.65), rough=0.7, metal=0.3)
    cab = looks.color(f"CAB_{cab_color[0]}_{cab_color[1]}_{cab_color[2]}".replace(".", ""), cab_color, rough=0.3,
                      metal=0.5)
    glass = looks.color("GLASS", (0.15, 0.2, 0.25), rough=0.05, metal=0.5, transparency=0.2)
    tire = looks.color("TIRE", (0.06, 0.06, 0.06), rough=0.9)
    chassis = looks.color("CHASSIS", (0.15, 0.15, 0.17), rough=0.6, metal=0.5)
    stripe = looks.color(f"STRIPE_{def_name}", cab_color, rough=0.5)
    L, W, H = 4.2, 2.1, 2.3
    cx = s * L / 2
    parts = [
        block(looks, inside, (cx, 0, -0.04), (L, W, 0.08)),                       # bed, level with the dock
        block(looks, body, (cx, -W / 2, H / 2), (L, 0.04, H)),
        block(looks, body, (cx, W / 2, H / 2), (L, 0.04, H)),
        block(looks, body, (cx, 0, H), (L, W, 0.04)),
        block(looks, body, (s * L, 0, H / 2), (0.04, W, H)),
        block(looks, stripe, (cx, -W / 2 - 0.022, 1.2), (L - 0.2, 0.004, 0.25)),
        block(looks, stripe, (cx, W / 2 + 0.022, 1.2), (L - 0.2, 0.004, 0.25)),
        # Rear doors swung open against the sides.
        block(looks, body, (s * 0.52, -W / 2 - 0.05, H / 2), (1.0, 0.03, H - 0.1)),
        block(looks, body, (s * 0.52, W / 2 + 0.05, H / 2), (1.0, 0.03, H - 0.1)),
        block(looks, chassis, (s * (L + 0.9) / 2, 0, -0.35), (L + 0.9, 1.0, 0.25)),
        # Cab.
        block(looks, cab, (s * (L + 0.95), 0, 0.35 - 0.1), (1.7, W - 0.05, 1.9)),
        block(looks, glass, (s * (L + 1.81), 0, 0.75), (0.02, W - 0.3, 0.7)),
        block(looks, glass, (s * (L + 1.0), -W / 2 + 0.01, 0.75), (0.9, 0.02, 0.6)),
        block(looks, glass, (s * (L + 1.0), W / 2 - 0.01, 0.75), (0.9, 0.02, 0.6)),
    ]
    for wx in (s * 0.9, s * 2.1, s * (L + 1.2)):
        for wy in (-W / 2 + 0.25, W / 2 - 0.25):
            parts.append(pose((wx, wy, -depth + 0.45), [shape(tire, cyl(0.45, 0.3))], rot=(1, 0, 0, math.pi / 2)))
    return pose((x_rear, y, 0), parts, def_name=def_name)


# ---------------------------------------------------------------- people, cartons

def worker(looks, def_name, hidden_at):
    vest = looks.color("VEST", (1, 0.45, 0.05), rough=0.6)
    reflect = looks.color("REFLECT", (0.85, 0.9, 0.85), rough=0.2, metal=0.6)
    pants = looks.color("PANTS", (0.12, 0.16, 0.3), rough=0.9)
    skin = looks.color("SKIN", (0.85, 0.68, 0.55), rough=0.7)
    helmet = looks.color("HELMET", (0.98, 0.85, 0.1), rough=0.3)
    parts = [
        pose((0, -0.09, 0.42), [shape(pants, cyl(0.07, 0.84))]),
        pose((0, 0.09, 0.42), [shape(pants, cyl(0.07, 0.84))]),
        block(looks, vest, (0, 0, 1.12), (0.24, 0.42, 0.6)),
        block(looks, reflect, (0, 0, 1.0), (0.25, 0.43, 0.04)),
        pose((0, -0.26, 1.1), [shape(vest, cyl(0.05, 0.55))]),
        pose((0, 0.26, 1.1), [shape(vest, cyl(0.05, 0.55))]),
        pose((0, 0, 1.55), [shape(skin, "Sphere { radius 0.11 }")]),
        pose((0, 0, 1.63), [shape(helmet, "Sphere { radius 0.12 }")]),
        block(looks, helmet, (0.08, 0, 1.6), (0.12, 0.2, 0.02)),
    ]
    return pose(hidden_at, parts, def_name=def_name)


def carton_size(ref, base):
    """Shoe cartons vary a little by model (footprint only; height stays the same)."""
    h = zlib.crc32(ref.encode())
    return (base[0] + (h % 3 - 1) * 0.02, base[1] + ((h >> 3) % 3 - 1) * 0.015, base[2])


def carton(looks, lpn, pos, size, sku, ref, label_color, plain=False):
    sx, sy, sz = size
    card = looks.texture("CARDBOARD", "webots/cardboard_top_base_color.jpg", rough=0.85)
    tape = looks.color("TAPE", (0.78, 0.62, 0.38), rough=0.4)
    label = looks.texture(f"STK_{ref}", "gen/sticker.png", rough=0.5, tint=label_color)
    r, g, b = label_color
    parts = [shape(card, box(sx, sy, sz)),
             pose((0, 0, sz / 2 + 0.001), [shape(tape, box(sx + 0.002, 0.05, 0.002))]),
             pose((0, -sy / 2 - 0.001, 0), [shape(label, box(0.11, 0.002, 0.07))]),
             pose((0, sy / 2 + 0.001, 0), [shape(label, box(0.11, 0.002, 0.07))], rot=(0, 0, 1, math.pi))]
    if plain:
        # One colored box per carton: much faster in batch runs.
        parts = [shape(looks.color(f"BOXC_{ref}", label_color, rough=0.8), box(sx, sy, sz))]
    return (f"DEF BOX_{lpn} Solid {{\n  translation {vec(*pos)}\n  name \"{lpn}\"\n  model \"{sku}\"\n"
            f"  recognitionColors [ {vec(r, g, b)} ]\n  children [ {' '.join(parts)} ]\n"
            f"  boundingObject {box(sx, sy, sz)}\n}}\n")


def rng_for(seed):
    return random.Random(seed)
