"""Temporary floor reference and fixed target for a two-tag drive trial."""
import cv2
import numpy as np

from camera_demo import pixel_pose
from floor_markers import FLOOR_SIDE_MM
from navigation_core import FloorCalibration

REFERENCE_IDS = (10,11)


def tag_squares(corners, ids):
    squares = {}
    for marker_id in set(ids):
        indices = np.flatnonzero(ids==marker_id)
        if len(indices)==1:
            squares[int(marker_id)] = np.asarray(corners[indices[0]],float).reshape(4,2)
    return squares


def tag_centre(square):
    pose = pixel_pose(square)
    return np.array([pose["u_px"],pose["v_px"]])


def tag_depth_points(packet, square):
    # Inside the paper, away from the segmentation boundary. Use the white
    # cells as well, since dark ink can leave holes in the depth image.
    depth = packet["depth"]
    centre = tag_centre(square)
    polygon = np.rint(centre + .85*(square-centre)).astype(np.int32)
    mask = np.zeros(depth.shape,np.uint8)
    cv2.fillConvexPoly(mask,polygon,1)
    v,u = np.nonzero(mask[::2,::2] & (depth[::2,::2]>0))
    v,u = 2*v,2*u
    z = depth[v,u]*packet["scale"]
    valid = (z>.3)&(z<6)
    if np.count_nonzero(valid)<15:
        raise ValueError("태그 주변 깊이 데이터가 부족합니다. 종이를 평평하게 놓고 조명을 확인하세요.")
    rays = np.array([packet["ray"]((x,y)) for x,y in zip(u[valid],v[valid])])
    return rays*z[valid,None]


def project_to_plane(ray, normal, anchor):
    divisor = float(ray@normal)
    if divisor>=-.05:
        raise ValueError("D455를 바닥 쪽으로 더 기울여 네 방향의 바닥이 보이게 하세요.")
    return ray*(float(anchor@normal)/divisor)


def robot_tag_height(packet, square, normal, anchor):
    try:
        points = tag_depth_points(packet,square)
    except ValueError as error:
        raise ValueError("로봇 태그 깊이가 부족합니다. 실측값을 --robot-marker-height로 입력하세요.") from error
    heights = (points-anchor)@normal
    height = float(np.median(heights))
    if not .02<=height<=.35 or np.ptp(np.percentile(heights,[10,90]))>.05:
        raise ValueError("로봇 태그 높이 추정이 불안정합니다. 실측값을 --robot-marker-height로 입력하세요.")
    return height


def floor_map_calibration(packet, normal, anchor, origin, right, robot_height):
    forward = np.cross(normal,right)
    rotation = np.column_stack([right,forward,normal])
    h,w = packet["frame"].shape[:2]
    corners = [(4,4),(w-5,4),(w-5,h-5),(4,h-5)]
    footprint = np.array([project_to_plane(packet["ray"](p),normal,anchor) for p in corners])
    bounds = (footprint-origin)@rotation[:,:2]
    lower,upper = bounds.min(axis=0),bounds.max(axis=0)
    width,height = upper-lower
    if min(width,height)<.25 or max(width,height)>8:
        raise ValueError("바닥이 화면 안에 들어오도록 D455 각도와 높이를 조정하세요.")
    translation = origin+rotation[:,:2]@lower
    calibration = FloorCalibration({
        "schema":1,"width_m":float(width),"height_m":float(height),"marker_height_m":robot_height,
        "world_to_camera_R":rotation.tolist(),"world_to_camera_t":translation.tolist(),
        "camera_metadata":packet["metadata"],
    })
    return calibration,tuple(float(x) for x in -lower)


def fit_tag_trial(packet, detector, robot_id, robot_height=None):
    """Use depth on two flat floor tags for scale and lift the robot centre above that plane."""
    corners,ids,_ = detector.detectMarkers(packet["frame"])
    ids = ids.ravel() if ids is not None else np.empty(0,dtype=int)
    squares = tag_squares(corners,ids)
    for marker_id in (*REFERENCE_IDS,robot_id):
        if marker_id not in squares:
            raise ValueError(f"태그 테스트 대기 · 화면에 ID {marker_id}가 한 장만 보여야 합니다.")
        square = squares[marker_id]
        if np.min(np.linalg.norm(square-np.roll(square,1,axis=0),axis=1))<24:
            raise ValueError(f"태그 테스트 대기 · ID {marker_id}가 너무 작습니다.")

    floor_samples = [tag_depth_points(packet,squares[i]) for i in REFERENCE_IDS]
    points = np.concatenate(floor_samples)
    for _ in range(3):
        anchor = points.mean(axis=0)
        _,_,vectors = np.linalg.svd(points-anchor,full_matrices=False)
        normal = vectors[-1]
        points = points[np.abs((points-anchor)@normal)<.025]
        if len(points)<30:
            raise ValueError("ID 10·11에서 같은 바닥 평면을 찾지 못했습니다.")
    if normal@anchor>0:
        normal = -normal
    if not .45 <= -normal@anchor <= 5 or normal[2]>-.65:
        raise ValueError("카메라 높이 또는 기울기를 확인하세요. 두 태그를 바닥에 평평하게 놓으세요.")
    for samples in floor_samples:
        if np.mean(np.abs((samples-anchor)@normal)<.025)<.75:
            raise ValueError("ID 10·11의 깊이가 한 평면에 맞지 않습니다. 두 종이를 바닥에 평평하게 놓으세요.")

    ray = packet["ray"]
    centre10,centre11 = [project_to_plane(ray(tag_centre(squares[i])),normal,anchor) for i in REFERENCE_IDS]
    separation = float(np.linalg.norm(centre11-centre10))
    if not .25<=separation<=6:
        raise ValueError("ID 10과 11을 바닥 위쪽에 간격을 두고 놓으세요.")
    for marker_id in REFERENCE_IDS:
        square3 = np.array([project_to_plane(ray(p),normal,anchor) for p in squares[marker_id]])
        edges = np.linalg.norm(square3-np.roll(square3,1,axis=0),axis=1)
        if np.any(np.abs(edges-FLOOR_SIDE_MM/1000)>.035):
            raise ValueError(f"ID {marker_id}의 깊이와 160mm 인쇄 크기가 일치하지 않습니다.")

    if robot_height is None:
        robot_height = robot_tag_height(packet,squares[robot_id],normal,anchor)
    calibration,goal = floor_map_calibration(packet,normal,anchor,centre10,
        (centre11-centre10)/separation,robot_height)
    return calibration,goal,{marker_id:squares[marker_id] for marker_id in REFERENCE_IDS}
