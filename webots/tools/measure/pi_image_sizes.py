"""Measure real JPEG sizes of the evidence images on the Raspberry Pi camera.

Run on the Pi with the camera pointed at a shelf (or a few boxes) from the
distance the scanner drives at (0.78 m in the simulator):

    python3 pi_image_sizes.py --distance 0.78 --shots 10

For each shot it crops the same regions the simulator assumes and encodes them
as JPEG at several qualities:
  frame - the whole 1280x720 frame
  crop  - the center target, 0.20 x 0.30 m, at half resolution (label legible)
  thumb - a 1.0 x 0.3 m strip around the center (target + neighbors), 200x60 px
Results go to pi_image_sizes.csv plus a summary on screen. Compare the mean
"crop" / "thumb" bytes with traffic.evidence_bytes in scenario.json.
"""
import argparse
import csv
import io
import math
import statistics
import time

from PIL import Image
from picamera2 import Picamera2

QUALITIES = (60, 70, 80, 90)
CROP_M = (0.20, 0.30)       # target only (w, h) in meters
THUMB_M = (1.00, 0.30)      # target + neighbors
THUMB_PX = (200, 60)


def jpeg_bytes(img, quality):
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=quality)
    return buf.tell()


def center_box(img, w_px, h_px):
    W, H = img.size
    w_px, h_px = min(w_px, W), min(h_px, H)
    left, top = (W - w_px) // 2, (H - h_px) // 2
    return img.crop((left, top, left + w_px, top + h_px))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--hfov-deg", type=float, default=62.0, help="OV5647 module: 62; check your lens")
    ap.add_argument("--distance", type=float, default=0.78, help="camera to shelf face, meters")
    ap.add_argument("--shots", type=int, default=10)
    ap.add_argument("--out", default="pi_image_sizes.csv")
    args = ap.parse_args()

    px_per_m = args.width / (2 * args.distance * math.tan(math.radians(args.hfov_deg) / 2))
    print(f"{args.width}x{args.height}, HFOV {args.hfov_deg} deg at {args.distance} m -> {px_per_m:.0f} px/m")

    cam = Picamera2()
    cam.configure(cam.create_still_configuration(main={"size": (args.width, args.height), "format": "RGB888"}))
    cam.start()
    time.sleep(2)  # auto exposure settles

    rows = []
    for shot in range(args.shots):
        frame = Image.fromarray(cam.capture_array()[:, :, ::-1])  # BGR -> RGB
        crop = center_box(frame, int(CROP_M[0] * px_per_m), int(CROP_M[1] * px_per_m))
        crop = crop.resize((max(1, crop.width // 2), max(1, crop.height // 2)))
        thumb = center_box(frame, int(THUMB_M[0] * px_per_m), int(THUMB_M[1] * px_per_m)).resize(THUMB_PX)
        for name, img in (("frame", frame), ("crop", crop), ("thumb", thumb)):
            for q in QUALITIES:
                t0 = time.perf_counter()
                n = jpeg_bytes(img, q)
                ms = (time.perf_counter() - t0) * 1000
                rows.append({"shot": shot, "kind": name, "w": img.width, "h": img.height, "quality": q,
                             "bytes": n, "bits_per_px": round(8 * n / (img.width * img.height), 3),
                             "encode_ms": round(ms, 2)})
        print(f"shot {shot + 1}/{args.shots}")
        time.sleep(0.5)
    cam.stop()

    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    print(f"\n{'kind':6} {'size':>9} {'q':>3} {'mean B':>8} {'min':>7} {'max':>7} {'bit/px':>7} {'enc ms':>7}")
    for name in ("thumb", "crop", "frame"):
        for q in QUALITIES:
            sel = [r for r in rows if r["kind"] == name and r["quality"] == q]
            b = [r["bytes"] for r in sel]
            print(f"{name:6} {sel[0]['w']:>4}x{sel[0]['h']:<4} {q:>3} {statistics.mean(b):>8.0f} {min(b):>7} "
                  f"{max(b):>7} {statistics.mean(r['bits_per_px'] for r in sel):>7.2f} "
                  f"{statistics.mean(r['encode_ms'] for r in sel):>7.1f}")
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
