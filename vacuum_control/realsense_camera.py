"""D455 colour/depth input; importing this module never opens a device.

Factory intrinsics describe the camera, not its pose relative to the floor.
This file does not import the vacuum driver or send any robot commands.
"""

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np


def load_sdk():
    try:
        import pyrealsense2 as rs
    except ImportError as error:
        raise RuntimeError(
            "RealSense SDK를 불러오지 못했습니다. .venv-camera의 Python으로 실행하세요. "
            "설치 명령은 D455.md에 있습니다. 원인: " + str(error)
        ) from error
    return rs


def info(device, rs, field):
    key = getattr(rs.camera_info, field)
    return device.get_info(key) if device.supports(key) else "unknown"


def list_devices(rs_module=None):
    rs = rs_module if rs_module is not None else load_sdk()
    return [{"name": info(device, rs, "name"),
             "serial": info(device, rs, "serial_number"),
             "firmware": info(device, rs, "firmware_version"),
             "usb": info(device, rs, "usb_type_descriptor")}
            for device in rs.context().query_devices()]


def median_depth_m(depth_image, scale, u, v, radius=2):
    """Optical Z depth, in metres; never a calibrated floor position."""
    if depth_image is None or not np.isfinite([u, v]).all():
        return None
    height, width = depth_image.shape
    u, v = int(round(u)), int(round(v))
    if not (0 <= u < width and 0 <= v < height):
        return None
    patch = depth_image[max(0, v-radius):min(height, v+radius+1),
                        max(0, u-radius):min(width, u+radius+1)]
    valid = patch[patch > 0]
    return round(float(np.median(valid)) * scale, 4) if valid.size else None


class RealSenseCapture:
    """Small VideoCapture-compatible adapter for the existing marker preview."""

    def __init__(self, serial=None, width=640, height=480, fps=30, rs_module=None):
        self.rs = rs_module if rs_module is not None else load_sdk()
        self._started = False
        self._depth_image = None
        self._profile = None
        devices = list_devices(self.rs)
        candidates = [d for d in devices if "D455" in d["name"]]
        if serial:
            candidates = [d for d in candidates if d["serial"] == serial]
        if not candidates:
            raise RuntimeError(
                "D455가 보이지 않습니다. USB 3 데이터 케이블과 연결 포트를 확인하세요."
            )
        if len(candidates) > 1:
            raise RuntimeError("D455가 여러 대입니다. --serial로 사용할 기기를 선택하세요.")
        self.device = candidates[0]
        self.pipeline = self.rs.pipeline()
        config = self.rs.config()
        config.enable_device(self.device["serial"])
        config.enable_stream(self.rs.stream.depth, width, height, self.rs.format.z16, fps)
        config.enable_stream(self.rs.stream.color, width, height, self.rs.format.bgr8, fps)
        try:
            self._profile = self.pipeline.start(config)
            self._started = True
            self.depth_scale = self._profile.get_device().first_depth_sensor().get_depth_scale()
            self.align = self.rs.align(self.rs.stream.color)
        except BaseException:
            self.release()
            raise

    def isOpened(self):
        return self._started

    def read(self):
        self._depth_image = None
        frames = self.pipeline.wait_for_frames(5000)
        aligned = self.align.process(frames)
        color, depth = aligned.get_color_frame(), aligned.get_depth_frame()
        if not color or not depth:
            return False, None
        self._depth_image = np.asanyarray(depth.get_data()).copy()
        self.frame_number = int(color.get_frame_number())
        return True, np.asanyarray(color.get_data()).copy()

    def depth_at(self, u, v):
        return median_depth_m(self._depth_image, self.depth_scale, u, v)

    def depth_preview(self):
        if self._depth_image is None:
            return None
        grey = cv2.convertScaleAbs(self._depth_image, alpha=0.03)
        view = cv2.applyColorMap(grey, cv2.COLORMAP_JET)
        view[self._depth_image == 0] = 0
        return view

    def metadata(self):
        intrinsics = self._profile.get_stream(self.rs.stream.color).as_video_stream_profile().get_intrinsics()
        return {
            "device": self.device,
            "color_intrinsics": {
                "width": intrinsics.width, "height": intrinsics.height,
                "fx": intrinsics.fx, "fy": intrinsics.fy,
                "ppx": intrinsics.ppx, "ppy": intrinsics.ppy,
                "model": str(intrinsics.model), "coeffs": list(intrinsics.coeffs),
            },
            "depth_scale_m": self.depth_scale,
            "depth_aligned_to": "color",
            "world_calibrated": False,
        }

    def release(self):
        if self._started:
            try:
                self.pipeline.stop()
            finally:
                self._started = False
                self._depth_image = None


def main():
    parser = argparse.ArgumentParser(description="D455 연결 및 컬러/깊이 수신 확인")
    parser.add_argument("command", choices=["list", "probe"])
    parser.add_argument("--serial")
    args = parser.parse_args()
    capture = None
    try:
        devices = list_devices()
        if not devices:
            print("RealSense 기기가 보이지 않습니다. USB 연결을 확인하세요.")
            return 1
        for device in devices:
            print("{name} / Serial {serial} / USB {usb} / Firmware {firmware}".format(**device))
        if args.command == "list":
            return 0
        capture = RealSenseCapture(serial=args.serial)
        # Brief warmup; no pictures/video are written to disk.
        for _ in range(30):
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError("컬러/깊이 프레임을 받지 못했습니다.")
        height, width = frame.shape[:2]
        depth = capture.depth_at(width / 2, height / 2)
        valid_depth = capture._depth_image[capture._depth_image > 0]
        coverage = 100.0 * valid_depth.size / capture._depth_image.size
        print("PASS: 컬러 {}x{}, 정렬된 깊이 {}x{} 수신".format(
            width, height, capture._depth_image.shape[1], capture._depth_image.shape[0]))
        print("깊이 유효 픽셀: {:.1f}%".format(coverage))
        if valid_depth.size:
            print("유효 영역 깊이 중앙값: {:.3f}m".format(
                float(np.median(valid_depth)) * capture.depth_scale))
        else:
            print("유효한 깊이가 없습니다. 렌즈 앞을 비우고 약 1m 떨어진 물체를 비춰 보세요.")
        print("중앙 부근 깊이 Z:", depth, "m (None이면 해당 위치의 깊이가 유효하지 않음)")
        output = Path(__file__).resolve().parent / "runtime" / "d455_camera.json"
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(capture.metadata(), ensure_ascii=False, indent=2), encoding="utf-8")
        print("카메라 내부 파라미터:", output)
        return 0
    except Exception as error:
        print("실패:", error, file=sys.stderr)
        print("Viewer/Camo 등 다른 카메라 프로그램이 스트리밍 중이면 종료하세요.", file=sys.stderr)
        return 1
    finally:
        if capture:
            capture.release()


if __name__ == "__main__":
    raise SystemExit(main())
