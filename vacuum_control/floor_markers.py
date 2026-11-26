"""Fixed layout shared by the printable marker set and the calibration UI."""
import cv2
import numpy as np

from camera_demo import pixel_pose
from navigation_core import FloorCalibration
from realsense_camera import median_depth_m

ROBOT_IDS = (0, 1, 2)
ROBOT_SIDE_MM = 120
FLOOR_IDS = (10, 11, 12, 13)  # clockwise from top left
FLOOR_SIDE_MM = 160


def floor_centres(width, height):
    return np.array([[0,height,0],[width,height,0],[width,0,0],[0,0,0]],dtype=float)


def floor_corners(width, height, side_mm):
    # Canonical ArUco corners are TL, TR, BR, BL when the arrow points to +Y.
    offsets = (side_mm/2000)*np.array([[-1,1,0],[1,1,0],[1,-1,0],[-1,-1,0]])
    return floor_centres(width,height)[:,None,:]+offsets


def calibrate_floor_markers(detector, packet, width, height, marker_height, side_mm):
    corners, ids, _ = detector.detectMarkers(packet["frame"])
    detected = ids.flatten() if ids is not None else np.empty(0,dtype=int)
    missing = [str(marker_id) for marker_id in FLOOR_IDS if marker_id not in detected]
    if missing:
        raise ValueError("안 보이는 바닥 마커 ID {}. 검은 테두리와 흰 여백 전체가 보이게 배치하세요.".format(
            ", ".join(missing)))
    squares, centres = [], []
    for marker_id in FLOOR_IDS:
        matches = np.flatnonzero(detected==marker_id)
        if len(matches)!=1:
            raise ValueError("바닥 마커 ID {}는 한 장만 보여야 합니다.".format(marker_id))
        square = corners[matches[0]].reshape(4,2)
        if np.min(np.linalg.norm(square-np.roll(square,1,axis=0),axis=1)) < 24:
            raise ValueError("바닥 마커 ID {}가 영상에서 너무 작습니다. 해상도를 높이거나 더 크게 출력하세요.".format(marker_id))
        squares.append(square)
        pose = pixel_pose(square)
        centres.append((pose["u_px"],pose["v_px"]))
    contour = np.asarray(centres,np.float32)
    if not cv2.isContourConvex(contour) or cv2.contourArea(contour,oriented=True)<3000:
        raise ValueError("바닥 마커를 왼쪽 위 10 → 오른쪽 위 11 → 오른쪽 아래 12 → 왼쪽 아래 13 순서로 배치하세요.")
    if not np.isfinite(side_mm) or not 50 <= side_mm <= 400 or side_mm >= min(width,height)*1000:
        raise ValueError("바닥 마커 검은 정사각형의 한 변을 mm로 입력하세요(50~400mm).")
    image = np.asarray(squares).reshape(-1,2)
    world = floor_corners(width,height,side_mm).reshape(-1,3)
    calibration = FloorCalibration.fit_points(image,world,width,height,marker_height,
        packet["ray"],packet["metadata"],marker_ids=FLOOR_IDS)
    if packet.get("depth") is None:
        return calibration,centres

    # Compare depth only at the four known floor points. Require agreement before
    # applying a scale so a covered marker cannot set the floor reference.
    measured = [median_depth_m(packet["depth"],packet["scale"],u,v,radius=5) for u,v in centres]
    if any(value is None or not 0.3 < value < 6.0 for value in measured):
        return calibration,centres
    measured = np.asarray(measured)
    rays = np.array([packet["ray"](centre) for centre in centres])
    expected = -calibration.camera[2] / (rays @ calibration.R)[:,2]
    scale = float(np.median(expected / measured))
    if 0.8 <= scale <= 1.2 and np.max(np.abs(expected - scale * measured)) <= 0.06:
        calibration = FloorCalibration({**calibration.data,"floor_depth_scale":scale})
    return calibration,centres
