"""Depth-only floor reference and a bounded goal along the starting heading."""
import math

import cv2
import numpy as np

from tag_trial import floor_map_calibration,robot_tag_height,tag_squares


def fit_floor(packet, robot_square):
    step = 8
    depth = packet["depth"]
    mask = np.zeros(depth.shape,np.uint8)
    centre = robot_square.mean(axis=0)
    cv2.fillConvexPoly(mask,np.rint(centre+2.0*(robot_square-centre)).astype(np.int32),1)
    rows,cols = np.mgrid[0:depth.shape[0]:step,0:depth.shape[1]:step]
    optical_depths = depth[rows,cols]*packet["scale"]
    valid = (optical_depths>.3)&(optical_depths<6)&(mask[rows,cols]==0)
    if np.count_nonzero(valid)<max(100,.35*valid.size):
        raise ValueError("깊이가 잡히는 바닥이 부족합니다. 빈 바닥을 더 많이 비추세요.")
    pixels = np.column_stack((cols[valid],rows[valid]))
    points = np.array([packet["ray"](pixel) for pixel in pixels])*optical_depths[valid,None]

    # Fit a dominant plane looking toward the camera. Keep sizeable obstacles
    # out of the floor fit and reject fits supported by only a small patch.
    rng = np.random.default_rng(0)
    triples = points[rng.integers(len(points),size=(128,3))]
    normals = np.cross(triples[:,1]-triples[:,0],triples[:,2]-triples[:,0])
    lengths = np.linalg.norm(normals,axis=1)
    available = lengths>1e-6
    normals = normals[available]/lengths[available,None]
    anchors = triples[available,0]
    normals[normals[:,2]>0] *= -1
    offsets = -np.sum(normals*anchors,axis=1)
    available = (offsets>=.45)&(offsets<=5.0)&(normals[:,2]<-.70)
    if not np.any(available):
        raise ValueError("바닥 평면을 찾지 못했습니다. D455를 바닥 쪽으로 향하게 하세요.")
    normals,offsets = normals[available],offsets[available]
    distances = np.abs(points@normals.T+offsets)
    inside = distances<.025
    best = int(np.argmax(inside.sum(axis=0)))
    inliers = inside[:,best]
    if np.count_nonzero(inliers)<.55*len(points) or np.ptp(pixels[inliers],axis=0).min()<.45*min(depth.shape):
        raise ValueError("평평한 바닥으로 확인된 범위가 좁습니다. D455가 더 넓은 빈 바닥을 보도록 하세요.")
    good = points[inliers]
    anchor = good.mean(axis=0)
    _,_,axes = np.linalg.svd(good-anchor,full_matrices=False)
    normal = axes[-1]
    if normal[2]>0:
        normal = -normal
    if not .45<=-float(normal@anchor)<=5 or normal[2]>-.70:
        raise ValueError("추정 바닥 위치가 불안정합니다. 카메라 높이와 기울기를 확인하세요.")
    return normal,anchor


def fit_forward_floor(packet, detector, robot_id, robot_height=None):
    corners,ids,_ = detector.detectMarkers(packet["frame"])
    squares = tag_squares(corners,ids.ravel()) if ids is not None else {}
    square = squares.get(robot_id)
    if square is None or np.min(np.linalg.norm(square-np.roll(square,1,axis=0),axis=1))<24:
        raise ValueError(f"로봇 위 ID {robot_id}가 한 장만 보이고 검은 변이 충분히 커야 합니다.")
    normal,anchor = fit_floor(packet,square)
    if robot_height is None:
        robot_height = robot_tag_height(packet,square,normal,anchor)
    right = np.array([1.,0.,0.])-normal[0]*normal
    right /= np.linalg.norm(right)
    calibration,_ = floor_map_calibration(packet,normal,anchor,anchor,right,robot_height)
    return calibration


def forward_target(pose, grid, max_distance):
    """Select a reachable point on the starting heading and never exceed the test limit."""
    start = (pose.x,pose.y)
    if not grid.free(start):
        return None
    for distance in np.arange(max_distance,.24,-.05):
        goal = (pose.x+distance*math.cos(pose.yaw),pose.y+distance*math.sin(pose.yaw))
        if grid.free(goal) and grid.plan(start,goal):
            return goal
    return None
