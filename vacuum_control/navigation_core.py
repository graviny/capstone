"""Metric geometry, conservative depth occupancy, A*, and feedback control.

No camera is opened and no robot connection is made by this module.
"""
from collections import deque
from dataclasses import dataclass, replace
import heapq
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Pose:
    x: float
    y: float
    yaw: float
    stamp: float


class FloorCalibration:
    """World XY is the floor; positive Z points upwards."""
    def __init__(self, data):
        self.data = data
        self.width = float(data["width_m"])
        self.height = float(data["height_m"])
        self.marker_height = float(data["marker_height_m"])
        self.floor_depth_scale = float(data.get("floor_depth_scale", 1.0))
        self.offset_forward = float(data.get("offset_forward_m", 0))
        self.offset_left = float(data.get("offset_left_m", 0))
        self.R = np.asarray(data["world_to_camera_R"], dtype=float)
        self.t = np.asarray(data["world_to_camera_t"], dtype=float)
        if (not np.isfinite([self.width, self.height, self.marker_height]).all()
                or self.width <= 0 or self.height <= 0
                or self.marker_height < 0):
            raise ValueError("바닥 가로·세로는 양수, 아이패드 높이는 0 이상의 미터 값이어야 합니다.")
        if not 0.8 <= self.floor_depth_scale <= 1.2:
            raise ValueError("바닥 깊이 보정값이 범위를 벗어났습니다. 다시 보정하세요.")
        if (self.R.shape != (3, 3) or self.t.size != 3
                or not np.isfinite(self.R).all() or not np.isfinite(self.t).all()
                or not np.allclose(self.R.T @ self.R, np.eye(3), atol=1e-4)
                or not np.isclose(np.linalg.det(self.R), 1, atol=1e-4)):
            raise ValueError("카메라 회전・이동 행렬이 올바르지 않습니다. 보정을 다시 계산하세요.")
        self.t = self.t.reshape(3)
        self.camera = -self.R.T @ self.t
        if not self.marker_height < self.camera[2] - 0.2:
            raise ValueError(
                "아이패드 높이 {:.2f}m가 추정 카메라 높이 {:.2f}m에 비해 너무 높습니다. "
                "바닥부터 아이패드 화면까지 실측하세요(예: 12cm = 0.12m). "
                "추정 카메라 높이도 실제와 다르면 바닥 치수와 네 점을 확인하세요.".format(
                    self.marker_height,self.camera[2]))

    @classmethod
    def fit(cls, pixels, width, height, marker_height, ray, metadata,
            offset_forward=0, offset_left=0):
        image = np.asarray(pixels, np.float32).reshape(4, 2)
        if not cv2.isContourConvex(image) or cv2.contourArea(image) < 3000:
            raise ValueError("네 모서리를 왼쪽 위→오른쪽 위→오른쪽 아래→왼쪽 아래 순서로 지정하세요.")
        world = np.array([[0, height, 0], [width, height, 0], [width, 0, 0], [0, 0, 0]], np.float64)
        return cls.fit_points(image,world,width,height,marker_height,ray,metadata,
                              offset_forward,offset_left)

    @classmethod
    def fit_points(cls, image, world, width, height, marker_height, ray, metadata,
                   offset_forward=0, offset_left=0, marker_ids=None):
        """Fit world-to-camera pose using known coplanar image/world correspondences."""
        values = [width, height, marker_height, offset_forward, offset_left]
        if not np.isfinite(values).all() or min(width, height) <= 0 or marker_height < 0:
            raise ValueError("바닥 가로·세로는 양수, 로봇 마커 높이는 0 이상의 미터 값으로 입력하세요.")
        image = np.asarray(image,np.float64)
        world = np.asarray(world,np.float64)
        normalized = np.array([ray(p)[:2] for p in image], np.float64)
        ok, rvec, tvec = cv2.solvePnP(world, normalized, np.eye(3), None,
                                     flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            raise ValueError("바닥 좌표 보정에 실패했습니다.")
        R = cv2.Rodrigues(rvec)[0]
        camera = -R.T @ tvec.reshape(3)
        if np.any((world @ R.T + tvec.reshape(3))[:,2] <= 0):
            raise ValueError("카메라 앞쪽에서 바닥을 찾지 못했습니다. 기준점 배치를 확인하세요.")
        if not 0.5 <= camera[2] <= 6.0:
            raise ValueError("계산된 카메라 높이가 0.5~6m 밖입니다. 치수와 점 순서를 확인하세요.")
        projected = cv2.projectPoints(world, rvec, tvec, np.eye(3), None)[0].reshape(-1, 2)
        intr = metadata["color_intrinsics"]
        errors = np.linalg.norm((projected-normalized)*[intr["fx"],intr["fy"]],axis=1)
        rms = float(np.sqrt(np.mean(errors**2)))
        if marker_ids is not None and (rms > 4 or errors.max() > 7):
            worst_id = marker_ids[int(np.argmax(errors))//4]
            raise ValueError(
                "마커 배치 오차 {:.1f}px(최대 {:.1f}px, ID {}). "
                "마커 중심 간 거리, 검은 정사각형 한 변, ID 위치와 화살표 방향을 확인하세요.".format(
                    rms,errors.max(),worst_id))
        if rms > 4:
            raise ValueError(
                "보정 오차 {:.1f}px(허용 4px 초과): 바닥의 실제 {:.2f}×{:.2f}m 직사각형 "
                "모서리 네 점인지 확인하세요. 가로·세로 대응과 점 순서도 확인하세요.".format(
                    rms,width,height))
        data = {"schema": 1, "width_m": width, "height_m": height,
                    "marker_height_m": marker_height, "offset_forward_m": offset_forward,
                    "offset_left_m": offset_left, "world_to_camera_R": R.tolist(),
                    "world_to_camera_t": tvec.reshape(3).tolist(), "reference_pixels": image.tolist(),
                    "camera_metadata": metadata, "reprojection_rms_px": rms}
        if marker_ids is not None:
            data["floor_marker_ids"] = list(marker_ids)
            data["floor_marker_side_mm"] = float((world[1,0]-world[0,0])*1000)
        return cls(data)

    def save(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.data, ensure_ascii=False, indent=2), encoding="utf-8")

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text(encoding="utf-8")))

    def validate_camera(self, metadata):
        expected = self.data["camera_metadata"]
        if expected["device"]["serial"] != metadata["device"]["serial"]:
            raise ValueError("보정할 때와 다른 카메라입니다. 다시 보정하세요.")
        a, b = expected["color_intrinsics"], metadata["color_intrinsics"]
        if a["model"] != b["model"] or any(abs(a[k]-b[k]) > 0.01 for k in ("width", "height", "fx", "fy", "ppx", "ppy")) or not np.allclose(a["coeffs"], b["coeffs"], atol=1e-6):
            raise ValueError("영상 프로파일/내부 파라미터가 달라졌습니다. 다시 보정하세요.")

    def intersect(self, ray, z=0):
        direction = self.R.T @ np.asarray(ray)
        if abs(direction[2]) < 1e-7:
            raise ValueError("바닥과 교차하지 않는 시선입니다.")
        distance = (z-self.camera[2]) / direction[2]
        if distance <= 0:
            raise ValueError("카메라 뒤쪽에 있는 평면입니다.")
        return self.camera + distance*direction

    def marker_pose(self, centre_ray, front_ray, stamp):
        centre = self.intersect(centre_ray, self.marker_height)
        front = self.intersect(front_ray, self.marker_height)
        direction = front-centre
        yaw = math.atan2(direction[1], direction[0])
        x = centre[0] - math.cos(yaw)*self.offset_forward + math.sin(yaw)*self.offset_left
        y = centre[1] - math.sin(yaw)*self.offset_forward - math.cos(yaw)*self.offset_left
        return Pose(float(x), float(y), yaw, stamp)


class OccupancyGrid:
    OBSERVATION_MAX_AGE = 0.6
    OBSTACLE_EVIDENCE_MAX_AGE = 0.5
    OBSTACLE_CLEAR_TIME = 1.2
    IMMEDIATE_OBSTACLE_LOOKAHEAD = 0.20

    def __init__(self, width, height, resolution=0.05, robot_radius=0.18, clearance=0.15):
        self.width, self.height, self.resolution = width, height, resolution
        self.robot_radius, self.clearance = robot_radius, clearance
        self.nx, self.ny = math.ceil(width/resolution), math.ceil(height/resolution)
        self.seen = np.full((self.ny, self.nx), -1e9, float)
        self.obstacles = np.zeros((self.ny, self.nx), bool)
        self.clear_since = np.full(self.obstacles.shape, np.inf)
        self.recent_occupied = deque(maxlen=2)
        self.cell_x, self.cell_y = np.meshgrid((np.arange(self.nx)+.5)*resolution,
                                               (np.arange(self.ny)+.5)*resolution)
        edge_distance = np.minimum(np.minimum(self.cell_x,width-self.cell_x),
                                   np.minimum(self.cell_y,height-self.cell_y))
        self.edge = edge_distance <= robot_radius+clearance
        self.static = np.zeros_like(self.obstacles)
        self.known = np.zeros_like(self.obstacles)
        self.blocked = np.ones_like(self.obstacles)
        self.coverage = 0.0
        r = math.ceil((robot_radius+clearance)/resolution)
        yy, xx = np.ogrid[-r:r+1, -r:r+1]
        self.kernel = ((xx*resolution)**2+(yy*resolution)**2 <= (robot_radius+clearance+resolution*.5)**2).astype(np.uint8)

    def cell(self, p):
        return int(math.floor(p[0]/self.resolution)), int(math.floor(p[1]/self.resolution))

    def world(self, c):
        return ((c[0]+0.5)*self.resolution, (c[1]+0.5)*self.resolution)

    def free(self, p):
        x, y = self.cell(p)
        return 0 <= x < self.nx and 0 <= y < self.ny and not self.blocked[y, x]

    def segment_free(self, a, b, *, blocked=None):
        blocked = self.blocked if blocked is None else blocked
        steps = max(1, math.ceil(math.dist(a, b)/(self.resolution*.3)))
        for u in np.linspace(0,1,steps+1):
            x,y = self.cell((a[0]+(b[0]-a[0])*u,a[1]+(b[1]-a[1])*u))
            if not (0 <= x < self.nx and 0 <= y < self.ny) or blocked[y,x]:
                return False
        return True

    def _cells_block_route(self, cells, segments):
        blocked = cv2.dilate(cells.astype(np.uint8),self.kernel,
                             borderType=cv2.BORDER_CONSTANT,borderValue=0).astype(bool)
        return any(not self.segment_free(start,end,blocked=blocked) for start,end in segments)

    def depth_obstacle_blocks(self, segments):
        return self._cells_block_route(self.obstacles & self.known,segments)

    def missing_depth_blocks(self, segments):
        return self._cells_block_route(~self.known,segments)

    def add_rectangle(self, a, b):
        xs = sorted([a[0], b[0]])
        ys = sorted([a[1], b[1]])
        x0, y0 = self.cell((xs[0], ys[0]))
        x1, y1 = self.cell((xs[1], ys[1]))
        self.static[max(0,y0):min(self.ny,y1+1), max(0,x0):min(self.nx,x1+1)] = True

    def update_points(self, points, pose, now):
        points = np.asarray(points).reshape(-1, 3)
        finite = np.isfinite(points).all(axis=1)
        inside = (finite & (points[:,0] >= 0) & (points[:,0] < self.width)
                  & (points[:,1] >= 0) & (points[:,1] < self.height)
                  & (points[:,2] >= -0.06) & (points[:,2] < 3.0))
        points = points[inside]
        free = np.zeros_like(self.obstacles)
        occupied = np.zeros_like(self.obstacles)
        if len(points):
            cells = np.floor(points[:,:2]/self.resolution).astype(int)
            floor = points[:,2] < 0.06
            free[cells[floor,1], cells[floor,0]] = True
            occupied[cells[~floor,1], cells[~floor,0]] = True
        nearby = np.zeros_like(occupied)
        if pose is not None:
            distance_squared = (self.cell_x-pose.x)**2 + (self.cell_y-pose.y)**2
            nearby = distance_squared <= (self.robot_radius+self.clearance+self.IMMEDIATE_OBSTACLE_LOOKAHEAD)**2
            under_robot = distance_squared <= (self.robot_radius+0.06)**2
            occupied[under_robot] = False
            free[under_robot] = True
            self.obstacles[under_robot] = False

        # Nearby returns are immediate. Far edge returns need corroboration even
        # when the current frame has several occupied neighbors.
        neighbors = cv2.boxFilter(occupied.astype(np.uint8), -1, (3,3), normalize=False,
                                  borderType=cv2.BORDER_CONSTANT)
        clustered = neighbors >= 3
        previous_hits = np.zeros_like(neighbors)
        for stamp, previous in self.recent_occupied:
            if 0 < now-stamp <= self.OBSTACLE_EVIDENCE_MAX_AGE:
                previous_hits += previous
        needed_hits = 1 + (self.edge & ~clustered)
        supported = nearby | (clustered & ~self.edge) | (previous_hits >= needed_hits)
        confirmed = occupied & supported
        if not self.recent_occupied or now > self.recent_occupied[-1][0]:
            self.recent_occupied.append((now, occupied))

        # Clearing requires sustained floor observations. Missing depth and
        # unconfirmed obstacle-only returns cannot refresh traversable cells.
        clean_floor = free & ~occupied
        self.clear_since[~clean_floor] = np.inf
        new_clear = clean_floor & (np.isinf(self.clear_since) | (now-self.seen > self.OBSERVATION_MAX_AGE))
        self.clear_since[new_clear] = now
        self.obstacles[now-self.clear_since >= self.OBSTACLE_CLEAR_TIME] = False
        self.obstacles[confirmed] = True
        self.seen[free | confirmed] = now
        self.known = now-self.seen <= self.OBSERVATION_MAX_AGE
        self.coverage = float(np.mean(self.known))
        base = ~self.known | self.obstacles | self.static
        self.blocked = cv2.dilate(base.astype(np.uint8), self.kernel,
                                  borderType=cv2.BORDER_CONSTANT, borderValue=1).astype(bool)

    def plan(self, start, goal):
        if not self.free(start) or not self.free(goal):
            return []
        source, target = self.cell(start), self.cell(goal)
        if source == target:
            return [tuple(start),tuple(goal)] if self.segment_free(start,goal) else []
        queue = [(0, source)]
        costs, parent = {source: 0.0}, {source: None}
        neighbours = [(dx,dy,math.hypot(dx,dy)) for dx in (-1,0,1) for dy in (-1,0,1) if dx or dy]
        while queue:
            _, cur = heapq.heappop(queue)
            if cur == target:
                break
            for dx, dy, distance in neighbours:
                nxt = cur[0]+dx, cur[1]+dy
                if not (0 <= nxt[0] < self.nx and 0 <= nxt[1] < self.ny) or self.blocked[nxt[1],nxt[0]]:
                    continue
                if dx and dy and (self.blocked[cur[1],nxt[0]] or self.blocked[nxt[1],cur[0]]):
                    continue
                cost = costs[cur]+distance
                if cost < costs.get(nxt, float("inf")):
                    costs[nxt], parent[nxt] = cost, cur
                    heapq.heappush(queue, (cost+math.dist(nxt,target), nxt))
        if target not in parent:
            return []
        route, node = [], target
        while node is not None:
            route.append(self.world(node))
            node = parent[node]
        route.reverse()
        # Keep the free cell centres. Replacing them with arbitrary subcell
        # endpoints can make the first/last diagonal clip an inflated corner.
        route = [tuple(start)]+route+[tuple(goal)]
        simplified, k = [tuple(start)], 0
        while k < len(route)-1:
            j = len(route)-1
            while j > k+1 and not self.segment_free(route[k], route[j]):
                j -= 1
            simplified.append(route[j])
            k = j
        return simplified


@dataclass(frozen=True)
class ReplanTiming:
    blocked_at: float
    started: float
    finished: float
    depth_obstacle: bool
    route_points: int


@dataclass
class BlockedRoute:
    detected: float
    depth_obstacle: bool
    stop_pose: Pose = None


class Navigator:
    """Stop/replan control, with a latched hold after loss of measurements."""
    def __init__(self, max_speed=0.08, measure=False):
        self.max_speed = min(max_speed, 0.10)
        self.goal = None
        self.path = []
        self.running = False
        self.message = "목표점을 선택하세요"
        self.last_stop_reason = None
        self._hold_until = 0
        self.replans = 0
        self.measure = measure
        self.last_replan = None
        self.blocked_route = None
        self._alignment = None

    def set_goal(self, goal):
        self.stop("경로 확인 후 주행 시작을 누르세요")
        self.goal, self.path = tuple(goal), []

    def stop(self, message="정지"):
        if self.running:
            self.last_stop_reason = message
        self.running = False
        self.message = message
        self._alignment = None
        self.blocked_route = None

    def _request_replan(self, grid, remaining):
        detected = time.perf_counter()
        depth_obstacle = grid.depth_obstacle_blocks(remaining)
        self.blocked_route = BlockedRoute(detected,depth_obstacle)
        self.path = []
        self._alignment = None
        if depth_obstacle:
            self.message = "깊이 장애물로 경로 차단 · 정지 확인 대기"
        elif grid.missing_depth_blocks(remaining):
            self.message = "깊이 미측정 구역으로 경로 차단 · 정지 확인 대기"
        else:
            self.message = "경로 차단 · 정지 확인 대기"

    def _replan_after_stop(self, blocked, pose, grid, stop_confirmed, verify_stop):
        if not stop_confirmed:
            blocked.stop_pose = None
            return
        if verify_stop:
            previous = blocked.stop_pose
            if (previous is None or math.hypot(pose.x-previous.x,pose.y-previous.y)>.02
                    or abs(wrap(pose.yaw-previous.yaw))>.10):
                blocked.stop_pose = replace(pose)
                return
            if pose.stamp-previous.stamp < .12:
                return
        started = time.perf_counter()
        self.path = grid.plan((pose.x,pose.y),self.goal)
        finished = time.perf_counter()
        if self.measure:
            self.last_replan = ReplanTiming(blocked.detected,started,finished,blocked.depth_obstacle,len(self.path))
        self.replans += 1
        self.blocked_route = None
        self.message = "새 경로 준비" if self.path else "우회 경로 없음 · 정지 대기"

    def step(self, pose, grid, now, healthy=True, *, stop_confirmed=True, verify_stop=False,
             measurement_issue=None):
        self.last_replan = None
        if pose is None or not healthy or now-pose.stamp > 0.35:
            self.stop(measurement_issue or "위치·깊이 측정 대기 — 복구 후 주행 시작을 누르세요")
            return 0.0, 0
        if self.goal is None:
            return 0.0, 0
        here = (pose.x, pose.y)
        if math.dist(here, self.goal) <= 0.12:
            self.stop("목표점 도착")
            self.path = []
            return 0.0, 0
        if self.blocked_route is not None:
            self._replan_after_stop(self.blocked_route,pose,grid,stop_confirmed,verify_stop)
            return 0.0, 0
        if self.path and len(self.path) > 1:
            while (len(self.path) > 2 and math.dist(here, self.path[1]) < 0.12
                   and grid.segment_free(here,self.path[2])):
                self.path.pop(0)
        remaining = list(zip([here]+self.path[1:],self.path[1:]))
        if remaining and not all(grid.segment_free(a,b) for a,b in remaining):
            self._request_replan(grid,remaining)
            return 0.0, 0
        if not remaining:
            self.path = grid.plan(here, self.goal)
            self.replans += 1
            self._hold_until = now+0.4
            self._alignment = None
            if not self.path:
                self.message = "경로 없음 — 장애물·미측정 구역을 확인하세요"
                return 0.0, 0
        if not self.running:
            self.message = "경로 준비 — 주행 시작을 누르세요"
            return 0.0, 0
        if now < self._hold_until:
            self.message = "정지 후 경로 재계산 중"
            return 0.0, 0
        target = self.path[1] if len(self.path)>1 else self.goal
        error = wrap(math.atan2(target[1]-pose.y, target[0]-pose.x)-pose.yaw)
        if abs(error) > 0.30:
            if self._alignment is None:
                self._alignment = (now, abs(error))
            elif now-self._alignment[0] > 1.0:
                if abs(error) > self._alignment[1]+0.18:
                    self.stop("회전 방향 불일치 — --turn-sign 설정을 확인하세요")
                    return 0.0, 0
                self._alignment = (now, abs(error))
            self.message = f"경로 방향으로 회전 · 방향 차이 {abs(math.degrees(error)):.0f}°"
            angular = int(np.clip(math.degrees(error)*1.0, -30, 30))
            return 0.0, angular
        self._alignment = None
        distance = math.dist(here,target)
        speed = min(self.max_speed,max(0.01,distance*0.4),distance/0.8)
        predicted = (pose.x+math.cos(pose.yaw)*speed*0.8,
                     pose.y+math.sin(pose.yaw)*speed*0.8)
        if not grid.segment_free(here,predicted):
            # A heading inside the normal tolerance can still point into an
            # inflated boundary. Align in place instead of replanning the same
            # valid route forever.
            if abs(error) > 0.002:
                self.message = "전방 여유 확보를 위해 방향 보정"
                angular = math.copysign(max(2,min(20,abs(math.degrees(error)))),error)
                return 0.0,int(angular)
            self._request_replan(grid,[(here,predicted)])
            return 0.0, 0
        self.message = "목표점으로 이동"
        return speed, int(np.clip(math.degrees(error), -15, 15))
