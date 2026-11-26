"""ArUco preview only: image pixels are not warehouse coordinates.

No robot driver is imported and no robot command is sent. A video frame received
now can still be delayed by a phone/virtual camera; timestamps do not prove its
capture time. Metric calibration and height compensation are later steps.
"""

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
DICTIONARY_NAME = "DICT_4X4_50"


def dictionary():
    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "ArucoDetector"):
        raise RuntimeError("OpenCV 4.7 이상과 ArUco 기능이 필요합니다.")
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def make_marker(out_dir, marker_id=0):
    """120 mm black marker in an SVG; PNG is a matching preview."""
    if not 0 <= marker_id < 50:
        raise ValueError("이 사전의 마커 ID 범위는 0~49입니다.")
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    bits = cv2.aruco.generateImageMarker(dictionary(), marker_id, 6)
    image = np.full((1050, 900), 255, dtype=np.uint8)
    image[250:850, 150:750] = cv2.aruco.generateImageMarker(dictionary(), marker_id, 600)
    cv2.arrowedLine(image, (450, 150), (450, 50), 0, 8, tipLength=0.3)
    cv2.putText(image, "ROBOT FRONT", (260, 220), cv2.FONT_HERSHEY_SIMPLEX, 1, 0, 2)
    cv2.putText(image, "4x4_50 / ID {}".format(marker_id), (250, 970),
                cv2.FONT_HERSHEY_SIMPLEX, 1, 0, 2)
    png = out_dir / "robot_{}.png".format(marker_id)
    if not cv2.imwrite(str(png), image):
        raise OSError("마커 PNG를 저장하지 못했습니다.")
    rectangles = []
    for row in range(6):
        for col in range(6):
            if bits[row, col] == 0:
                rectangles.append('<rect x="{}" y="{}" width="20" height="20"/>'.format(
                    30 + col * 20, 50 + row * 20))
    svg = out_dir / "robot_{}.svg".format(marker_id)
    svg.write_text(
        '<svg xmlns="http://www.w3.org/2000/svg" width="180mm" height="210mm" '
        'viewBox="0 0 180 210">\n<rect width="180" height="210" fill="white"/>\n'
        '<g fill="black">' + "\n".join(rectangles) + '</g>\n'
        '<path d="M90 28 L90 8 M82 17 L90 8 L98 17" fill="none" stroke="black" stroke-width="2"/>\n'
        '<g font-family="Arial,sans-serif" font-size="5" text-anchor="middle">'
        '<text x="90" y="39">ROBOT FRONT</text>'
        '<text x="90" y="190">DICT_4X4_50 / ID {}</text>'
        '<text x="90" y="201">Black marker: 120 mm / Print at 100%</text></g>\n</svg>'.format(marker_id),
        encoding="utf-8",
    )
    return png, svg


def pixel_pose(corners):
    """Project the canonical marker centre and front edge into image pixels."""
    corners = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    canonical = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(canonical, corners)
    points = cv2.perspectiveTransform(
        np.array([[[0.5, 0.5], [0.5, 0.0]]], dtype=np.float32), matrix
    )[0]
    centre, front = points
    delta = front - centre
    if not np.isfinite(points).all() or np.linalg.norm(delta) < 1:
        raise ValueError("마커의 방향을 계산할 수 없습니다.")
    return {
        "u_px": round(float(centre[0]), 2),
        "v_px": round(float(centre[1]), 2),
        "front_u_px": round(float(front[0]), 2),
        "front_v_px": round(float(front[1]), 2),
        # Image right = 0 degrees; image up = +90. Not calibrated world yaw.
        "heading_image_deg": round(math.degrees(math.atan2(-delta[1], delta[0])), 2),
    }


class MarkerTracker:
    def __init__(self, marker_id=0):
        self.marker_id = marker_id
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(dictionary(), parameters)

    def detect(self, frame, frame_seq):
        corners, ids, _ = self.detector.detectMarkers(frame)
        result = {
            "valid": False, "reason": "marker_not_found", "marker_id": self.marker_id,
            "coordinate_system": "image_pixels_only", "frame_seq": frame_seq,
            "received_at_unix_s": time.time(),
            "image_size": [int(frame.shape[1]), int(frame.shape[0])],
        }
        if ids is not None:
            matches = np.flatnonzero(ids.flatten() == self.marker_id)
            if len(matches) > 1:
                result["reason"] = "duplicate_marker_id"
            elif len(matches) == 1:
                try:
                    result.update(pixel_pose(corners[int(matches[0])]))
                    result.update(valid=True, reason="detected")
                except ValueError:
                    result["reason"] = "invalid_marker_geometry"
        return result, corners, ids


def write_result(path, result):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".{}.tmp".format(os.getpid()))
    temporary.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(str(temporary), str(path))


def open_capture(args):
    source = int(args.source) if args.source.isdigit() else args.source
    if source == "realsense":
        from realsense_camera import RealSenseCapture
        return RealSenseCapture(serial=args.serial)
    elif isinstance(source, int):
        backends = {"dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF, "auto": cv2.CAP_ANY}
        return cv2.VideoCapture(source, backends[args.backend])
    else:
        return cv2.VideoCapture(source, cv2.CAP_FFMPEG, [
            cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000,
            cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000,
        ])


def track(args):
    tracker = MarkerTracker(args.marker_id)
    source = args.source
    capture = None
    reason = "stopped"
    sequence = 0
    last_print = 0.0
    try:
        capture = open_capture(args)
        if not capture.isOpened():
            raise RuntimeError("카메라를 열지 못했습니다. Camo 연결과 카메라 번호를 확인하세요.")
        print("카메라 연결됨. 마커 인식만 수행합니다. 로봇 이동 명령은 보내지 않습니다.")
        print("영상 좌표 u/v 단위는 픽셀입니다. 창에서 Q 또는 Esc로 종료하세요.")
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                reason = "frame_read_failed"
                raise RuntimeError("영상 프레임을 받지 못했습니다.")
            sequence += 1
            result, corners, ids = tracker.detect(frame, sequence)
            if source == "realsense":
                result["source"] = "D455"
                result["marker_depth_z_m"] = capture.depth_at(result["u_px"], result["v_px"]) if result["valid"] else None
                result["depth_valid"] = result["marker_depth_z_m"] is not None
            write_result(args.output, result)
            if time.monotonic() - last_print > 1:
                if result["valid"]:
                    print("ID {}: u={} px, v={} px, image_heading={} deg".format(
                        args.marker_id, result["u_px"], result["v_px"], result["heading_image_deg"]))
                else:
                    print("마커 확인 안 됨:", result["reason"])
                last_print = time.monotonic()
            if not args.headless:
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                label = "MARKER LOST"
                if result["valid"]:
                    centre = (round(result["u_px"]), round(result["v_px"]))
                    front = (round(result["front_u_px"]), round(result["front_v_px"]))
                    cv2.arrowedLine(frame, centre, front, (0, 180, 255), 3)
                    label = "ID {}  u={:.0f}  v={:.0f}  image angle={:.1f}".format(
                        args.marker_id, result["u_px"], result["v_px"], result["heading_image_deg"])
                cv2.putText(frame, label, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 180, 255), 2)
                cv2.putText(frame, "PIXELS ONLY / Q: quit / No robot control", (15, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 180, 255), 1)
                display = frame
                if source == "realsense":
                    depth_view = capture.depth_preview()
                    if depth_view is not None:
                        cv2.putText(depth_view, "ALIGNED DEPTH / Black: invalid", (15, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        display = np.hstack((frame, depth_view))
                cv2.imshow("Robot marker preview", display)
                if cv2.waitKey(1) & 0xff in (ord("q"), 27):
                    break
                if cv2.getWindowProperty("Robot marker preview", cv2.WND_PROP_VISIBLE) < 1:
                    break
            if args.max_frames and sequence >= args.max_frames:
                break
    except BaseException:
        if reason == "stopped":
            reason = "tracker_unavailable"
        raise
    finally:
        if capture is not None:
            capture.release()
        if not args.headless:
            cv2.destroyAllWindows()
        # A consumer must never mistake the last visible marker for live data.
        write_result(args.output, {
            "valid": False, "reason": reason, "marker_id": args.marker_id,
            "coordinate_system": "image_pixels_only", "frame_seq": sequence,
            "received_at_unix_s": time.time(),
        })


def main():
    parser = argparse.ArgumentParser(description="로봇 마커 인쇄 및 카메라 인식 확인")
    sub = parser.add_subparsers(dest="command", required=True)
    marker = sub.add_parser("marker", help="인쇄용 SVG와 미리보기 PNG 생성")
    marker.add_argument("--out", type=Path, default=ROOT / "markers")
    marker.add_argument("--marker-id", type=int, default=0, choices=range(50))
    preview = sub.add_parser("track", help="카메라 영상에서 마커 확인")
    preview.add_argument("--source", default="0", help="realsense, 카메라 번호, 영상 파일 또는 영상 URL")
    preview.add_argument("--serial", help="D455가 여러 대일 때 사용할 기기의 시리얼")
    preview.add_argument("--backend", default="dshow" if os.name == "nt" else "auto",
                         choices=["dshow", "msmf", "auto"])
    preview.add_argument("--marker-id", type=int, default=0, choices=range(50))
    preview.add_argument("--output", type=Path, default=ROOT / "runtime" / "latest_detection.json")
    preview.add_argument("--headless", action="store_true")
    preview.add_argument("--max-frames", type=int, default=0)
    args = parser.parse_args()
    try:
        if args.command == "marker":
            for path in make_marker(args.out, args.marker_id):
                print(path)
        else:
            track(args)
        return 0
    except KeyboardInterrupt:
        print("\n인식을 종료했습니다.")
        return 130
    except Exception as error:
        print("실패:", error, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
