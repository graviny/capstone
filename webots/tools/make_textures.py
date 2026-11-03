"""Text textures for the world: slot labels, area / region signs, the carton barcode sticker.

Written to textures/gen/ by gen_world.py. Photo textures (concrete, cardboard, ...) are in
textures/webots/ (from the Webots project, Apache 2.0).
"""
import random
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

FONT_FILES = ["C:/Windows/Fonts/arialbd.ttf", "C:/Windows/Fonts/arial.ttf",
              "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"]


def font(size):
    for f in FONT_FILES:
        if Path(f).exists():
            return ImageFont.truetype(f, size)
    return ImageFont.load_default(size=size)


def centered(draw, box, text, fnt, fill):
    x0, y0, x1, y1 = box
    l, t, r, b = draw.textbbox((0, 0), text, font=fnt)
    draw.text((x0 + (x1 - x0 - (r - l)) / 2 - l, y0 + (y1 - y0 - (b - t)) / 2 - t), text, font=fnt, fill=fill)


def barcode(draw, box, seed):
    rng = random.Random(seed)
    x0, y0, x1, y1 = box
    x = x0
    while x < x1:
        w = rng.choice((1, 1, 2, 3))
        if rng.random() < 0.55:
            draw.rectangle((x, y0, min(x + w, x1), y1), fill=(20, 20, 20))
        x += w


def slot_label(text, path):
    im = Image.new("RGB", (256, 64), (250, 250, 245))
    d = ImageDraw.Draw(im)
    d.rectangle((0, 0, 255, 63), outline=(30, 30, 30), width=3)
    centered(d, (6, 4, 160, 60), text.upper(), font(32), (15, 15, 15))
    barcode(d, (168, 10, 246, 54), text)
    im.save(path)


def sign(title, subtitle, path, color):
    im = Image.new("RGB", (512, 256), color)
    d = ImageDraw.Draw(im)
    d.rectangle((0, 0, 511, 255), outline=(255, 255, 255), width=10)
    centered(d, (0, 20, 512, 170), title, font(120), (255, 255, 255))
    centered(d, (0, 170, 512, 240), subtitle, font(44), (255, 255, 255))
    im.save(path)


def sticker(path):
    """White shipping label with a barcode; tinted per product model in the world."""
    im = Image.new("RGB", (256, 128), (255, 255, 255))
    d = ImageDraw.Draw(im)
    d.rectangle((0, 0, 255, 127), outline=(40, 40, 40), width=4)
    d.rectangle((12, 10, 243, 32), fill=(60, 60, 60))
    barcode(d, (20, 42, 236, 100), "sticker")
    centered(d, (0, 102, 256, 124), "SHOE CARTON", font(18), (30, 30, 30))
    im.save(path)


def make_all(out_dir, slot_ids, regions):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    for sid in slot_ids:
        slot_label(sid, out / f"slot_{sid}.png")
    palette = [(200, 60, 40), (40, 110, 190), (40, 150, 80), (150, 80, 170)]
    for k, r in enumerate(regions):
        sign(r, f"OUTBOUND LANE {r}", out / f"sign_{r}.png", palette[k % len(palette)])
    sign("INBOUND", "RECEIVING DOCK", out / "sign_inbound.png", (210, 150, 20))
    sign("EXCEPTION", "NO-READ CARTONS", out / "sign_exception.png", (190, 40, 40))
    sign("CHARGING", "AMR PARKING", out / "sign_parking.png", (60, 60, 70))
    sticker(out / "sticker.png")
