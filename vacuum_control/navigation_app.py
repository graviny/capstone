"""D455 calibration, clickable goals, depth obstacle planning and guarded drive.

Preview is the default. --drive requires an explicit start action in the window.
"""
import argparse
from functools import lru_cache
import getpass
from itertools import cycle
import math
from pathlib import Path
import time
import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageTk

from camera_demo import MarkerTracker, pixel_pose
from floor_markers import FLOOR_SIDE_MM, ROBOT_IDS, calibrate_floor_markers
from forward_avoid import fit_forward_floor, forward_target
from navigation_core import FloorCalibration, Navigator, OccupancyGrid, wrap
from navigation_io import CameraWorker, MotionWorker, NativeMiio
from replan_measurement import append_measurement, measure_replan
from tag_trial import fit_tag_trial, tag_squares

ROOT = Path(__file__).resolve().parent


class Session:
    def __init__(self, radius=0.18, clearance=0.15, marker_id=0):
        self.tracker = MarkerTracker(marker_id)
        self.radius, self.clearance = radius, clearance
        self.calibration = self.grid = self.pose = None
        self.previous_pose = None
        self.previous_square = None
        self.marker_overlays = (),None
        self.last_number = None
        self.healthy = False
        self.marker_pixels = 0.0
        self.trial_reference = None
        self.message = "바닥 보정이 필요합니다"

    def calibrate(self, calibration, packet):
        calibration.validate_camera(packet["metadata"])
        self.calibration = calibration
        self.grid = OccupancyGrid(calibration.width,calibration.height,
                                  robot_radius=self.radius,clearance=self.clearance)
        height,width = packet["frame"].shape[:2]
        samples = []
        # Alternate the sampled pixel positions across frames without increasing
        # the number of points projected from each depth image.
        for row,col in ((0,0),(1,2),(2,1)):
            rows,cols = np.mgrid[row:height:3,col:width:3]
            rows,cols = rows.ravel(),cols.ravel()
            rays = np.array([packet["ray"]((u,v)) for u,v in zip(cols,rows)])
            samples.append((rows,cols,rays@calibration.R))
        self.depth_samples = cycle(samples)
        self.pose = self.previous_pose = None
        self.previous_square = None
        self.last_number = None

    def _recover_marker(self, gray, stamp):
        if self.previous_square is None or not 0 <= stamp-self.previous_pose.stamp <= .35:
            return None
        prior = self.previous_square
        padding = max(32,math.ceil(np.ptp(prior,axis=0).max()))
        x0,y0 = np.maximum(np.floor(prior.min(axis=0)).astype(int)-padding,0)
        x1,y1 = np.minimum(np.ceil(prior.max(axis=0)).astype(int)+padding,(gray.shape[1],gray.shape[0]))
        crop = gray[y0:y1,x0:x1]
        search = cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_LINEAR)
        corners,ids,_ = self.tracker.detector.detectMarkers(search)
        matches = np.flatnonzero(ids.ravel()==self.tracker.marker_id) if ids is not None else []
        if len(matches)==1:
            return corners[matches[0]]*.5+np.array([x0,y0],dtype=np.float32)
        return None

    def annotate_frame(self, frame):
        frame = frame.copy()
        corners,ids = self.marker_overlays
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame,corners,ids)
        return frame

    def process(self, packet, *, annotate=True):
        frame = packet["frame"]
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        corners,ids,_ = self.tracker.detector.detectMarkers(gray)
        matches = np.flatnonzero(ids.ravel()==self.tracker.marker_id) if ids is not None else []
        if self.calibration is not None and len(matches)==0:
            recovered = self._recover_marker(gray,packet["stamp"])
            if recovered is not None:
                matches = [len(corners)]
                corners = (*corners,recovered)
                ids = np.append(ids.ravel() if ids is not None else [],self.tracker.marker_id)
                ids = ids.astype(np.int32).reshape(-1,1)
        self.marker_overlays = corners,ids
        if annotate:
            frame = self.annotate_frame(frame)
        self.pose = None
        self.healthy = False
        self.marker_pixels = 0.0
        if self.calibration is None:
            return frame
        self.calibration.validate_camera(packet["metadata"])
        if self.trial_reference is not None:
            visible = tag_squares(corners,ids.ravel()) if ids is not None else {}
            moved = any(np.max(np.linalg.norm(visible[i]-expected,axis=1))>5
                        for i,expected in self.trial_reference.items() if i in visible)
            if 11 not in visible or moved:
                self.message = "ID 11이 가려졌거나 바닥 태그 위치가 달라졌습니다. 태그를 다시 읽으세요."
                return frame
        if len(matches)==1:
            square = corners[int(matches[0])].reshape(4,2)
            self.marker_pixels = float(np.min(np.linalg.norm(square-np.roll(square,1,axis=0),axis=1)))
            if self.marker_pixels >= 24:
                p = pixel_pose(square)
                pose = self.calibration.marker_pose(packet["ray"]((p["u_px"],p["v_px"])),
                    packet["ray"]((p["front_u_px"],p["front_v_px"])),packet["stamp"])
                if 0 <= pose.x < self.calibration.width and 0 <= pose.y < self.calibration.height:
                    previous = self.previous_pose
                    if (previous is None or pose.stamp-previous.stamp > .7
                            or (math.hypot(pose.x-previous.x,pose.y-previous.y) < .13
                                and abs(wrap(pose.yaw-previous.yaw)) < .7)):
                        self.pose = self.previous_pose = pose
                        self.previous_square = square
        if self.pose is None:
            # Without a trusted pose the robot's own depth cannot be masked.
            # Age existing observations without adding occupied or free cells.
            self.grid.update_points((),None,packet["stamp"])
            if 0 < self.marker_pixels < 24:
                self.message = "마커가 작습니다 — 화면에서 확대하거나 해상도를 높이세요"
            else:
                self.message = f"마커 ID {self.tracker.marker_id}의 위치·방향 측정 실패"
            return frame

        rows,cols,world_rays = next(self.depth_samples)
        samples = packet["depth"][rows,cols]*packet["scale"]
        valid = (samples>.3)&(samples<6.0)
        corrected_depth = samples[valid,None] * self.calibration.floor_depth_scale
        points = corrected_depth*world_rays[valid]+self.calibration.camera
        self.grid.update_points(points,self.pose,packet["stamp"])
        self.healthy = self.grid.coverage >= .55
        self.message = "측정 정상" if self.healthy else "깊이 영상의 측정 범위가 부족합니다"
        return frame


def map_image(grid, pose, path, goal, size=(620,420), *, pose_stale=False):
    image = Image.new("RGB",size,"#edf0f3")
    draw = ImageDraw.Draw(image)
    if grid is None:
        draw.text((24,24),"바닥 보정 후 지도와 경로가 표시됩니다",font=font(16),fill="#334155")
        return image,None
    scale = min((size[0]-64)/grid.width,(size[1]-64)/grid.height)
    ox, oy = (size[0]-scale*grid.width)/2,(size[1]+scale*grid.height)/2
    def project(p):
        return ox+p[0]*scale,oy-p[1]*scale
    cells = np.full((*grid.blocked.shape,3),(248,250,252),np.uint8)
    cells[~grid.known] = (148,163,184)
    cells[grid.blocked & grid.known] = (246,211,203)
    cells[(grid.obstacles & grid.known)|grid.static] = (39,49,65)
    raster = Image.fromarray(np.flipud(cells)).resize((round(grid.width*scale),round(grid.height*scale)),Image.Resampling.NEAREST)
    image.paste(raster,(round(ox),round(oy-grid.height*scale)))
    draw.rectangle((*project((0,grid.height)),*project((grid.width,0))),outline="#64748b",width=1)
    if path and len(path)>1:
        draw.line([project(p) for p in path],fill="#205a9e",width=4)
    if goal is not None:
        x,y = project(goal)
        draw.ellipse((x-7,y-7,x+7,y+7),fill="#c06515",outline="white",width=2)
    if pose:
        x,y = project((pose.x,pose.y))
        radius = grid.robot_radius*scale
        fill,heading = ("#94a3b8","#475569") if pose_stale else ("#16796e","#083e38")
        draw.ellipse((x-radius,y-radius,x+radius,y+radius),fill=fill,outline="white",width=2)
        end = project((pose.x+math.cos(pose.yaw)*.3,pose.y+math.sin(pose.yaw)*.3))
        draw.line([(x,y),end],fill=heading,width=4)
    draw.text((int(ox),size[1]-25),"원점 (0, 0)     x →     y ↑",font=font(11),fill="#334155")
    return image,(ox,oy,scale)


@lru_cache(maxsize=8)
def font(size):
    try:
        return ImageFont.truetype("C:/Windows/Fonts/malgun.ttf",size)
    except OSError:
        return ImageFont.load_default()


class App:
    def __init__(self,args,token=None):
        self.args = args
        self.quick_trial = args.tag_test or args.forward_avoid
        self.measure_replan = args.measure_replan
        self.replan_frame = None
        self.root = tk.Tk()
        self.root.title("D455 목표점 주행")
        width,height = min(1320,self.root.winfo_screenwidth()-60),min(820,self.root.winfo_screenheight()-90)
        self.root.geometry("{}x{}".format(width,height))
        self.root.minsize(940,650)
        self.root.configure(bg="#f4f6f8")
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame",background="#f4f6f8")
        style.configure("TLabel",background="#f4f6f8",foreground="#17212f",font=("맑은 고딕",10))
        style.configure("TButton",font=("맑은 고딕",10),padding=(10,7))
        self.session = Session(args.robot_radius,args.clearance,args.marker_id)
        self.navigator = Navigator(max_speed=.05 if self.quick_trial else .08,measure=self.measure_replan)
        self.trial_goal = None
        self.trial_retry_at = 0.0
        self.worker = None
        if args.simulate:
            from navigation_sim import SimulatedCamera
            self.camera = SimulatedCamera(marker_id=args.marker_id,show_floor_tags=True)
        else:
            width,height = map(int,args.resolution.split("x"))
            self.camera = CameraWorker(width=width,height=height)
        if args.drive:
            self.worker = MotionWorker(lambda: NativeMiio(args.ip,token,args.turn_sign),
                                       event_log=ROOT/"runtime"/"drive_events.csv")
            self.worker.start_thread()
        self.packet = None
        self.video_image = None
        self.video_transform = None
        self.map_transform = None
        self.frozen = None
        self.points = []
        self.pending_calibration = None
        self.obstacle_first = None
        self.last_processed = None
        self.next_render_at = 0.0
        self.last_log = 0
        self.closed = False
        self._build()
        self.root.protocol("WM_DELETE_WINDOW",self.close)
        self.root.bind("<Escape>",lambda event:self.stop())
        self.root.after(100,self.tick)

    def _build(self):
        header = ttk.Frame(self.root,padding=(18,12))
        header.pack(fill="x")
        ttk.Label(header,text="목표점 주행",font=("맑은 고딕",17,"bold")).pack(side="left")
        mode = "모의 화면 · 실제 장치 연결 없음" if self.args.simulate else (
            "실물 제어 · 시작 버튼을 누르면 이동" if self.args.drive else "미리보기 · 로봇에 명령을 보내지 않음")
        ttk.Label(header,text=mode,foreground="#a5281b" if self.args.drive else "#205a9e").pack(side="right")
        config = ttk.Frame(self.root,padding=(18,2))
        if not self.quick_trial:
            config.pack(fill="x")
        self.fields = {}
        for label,key in [("바닥 가로(m)","width"),("세로(m)","height"),("로봇 마커 높이(m)","marker_height"),
                          ("바닥 마커 한 변(mm)","floor_side")]:
            ttk.Label(config,text=label).pack(side="left",padx=(0,5))
            value = tk.StringVar(value=str(FLOOR_SIDE_MM) if key=="floor_side" else "")
            ttk.Entry(config,textvariable=value,width=7).pack(side="left",padx=(0,14))
            self.fields[key] = value
        actions = ttk.Frame(self.root,padding=(14,5))
        actions.pack(fill="x")
        ttk.Button(actions,text="바닥 마커 자동 보정",command=self.auto_calibrate).pack(side="left",padx=4)
        ttk.Button(actions,text="바닥 네 점 지정",command=self.begin_calibration).pack(side="left",padx=4)
        self.save_button = ttk.Button(actions,text="보정 저장",command=self.save_calibration,state="disabled")
        self.save_button.pack(side="left",padx=4)
        ttk.Button(actions,text="저장된 보정 사용",command=self.load_calibration).pack(side="left",padx=4)
        self.notice = tk.StringVar(value="자동 보정은 바닥 마커 10–13의 중심 간 거리와 바닥부터 로봇 마커 표면까지의 높이를 사용합니다.")
        if self.quick_trial:
            actions.pack_forget()
            trial_actions = ttk.Frame(self.root,padding=(18,5))
            trial_actions.pack(fill="x")
            if self.args.forward_avoid:
                ttk.Button(trial_actions,text="전방 다시 읽기",command=self.reset_trial).pack(side="left")
                ttk.Label(trial_actions,text=f"전방 주행·우회 시험 · 정면 최대 {self.args.forward_distance:g}m",
                          padding=(12,0)).pack(side="left")
                self.notice.set("D455를 바닥으로 고정하고 로봇 위 태그의 화살표를 정면에 맞추세요.")
            else:
                ttk.Button(trial_actions,text="태그 다시 읽기",command=self.reset_trial).pack(side="left")
                ttk.Label(trial_actions,text="고정 목표 · 바닥 ID 10의 중심",padding=(12,0)).pack(side="left")
                self.notice.set("바닥 ID 10·11과 로봇 태그가 보이면 목표와 경로를 표시합니다.")
        ttk.Label(self.root,textvariable=self.notice,padding=(18,9),wraplength=1200).pack(fill="x")
        area = ttk.Frame(self.root,padding=(18,0))
        area.configure(height=160)
        area.grid_propagate(False)
        area.pack(fill="both",expand=True)
        area.columnconfigure(0,weight=1)
        area.columnconfigure(1,weight=1)
        area.rowconfigure(1,weight=1)
        ttk.Label(area,text=f"D455 컬러 영상 · 로봇 ID {self.args.marker_id}").grid(row=0,column=0,sticky="w",pady=(0,6))
        map_title = "실측 지도 · ID 10을 목표로 사용" if self.args.tag_test else "실측 지도 · 목표점을 클릭하세요"
        ttk.Label(area,text=map_title).grid(row=0,column=1,sticky="w",pady=(0,6),padx=(12,0))
        self.video_label = tk.Canvas(area,bg="#18212d",width=1,height=1,highlightthickness=0)
        self.video_label.grid(row=1,column=0,sticky="nsew")
        self.video_item = self.video_label.create_image(0,0,anchor="nw")
        self.map_label = tk.Canvas(area,bg="#edf0f3",width=1,height=1,highlightthickness=0)
        self.map_label.grid(row=1,column=1,sticky="nsew",padx=(12,0))
        self.map_item = self.map_label.create_image(0,0,anchor="nw")
        self.video_label.bind("<Button-1>",self.video_click)
        self.map_label.bind("<Button-1>",self.map_click)
        toolbar = ttk.Frame(self.root,padding=(18,12))
        toolbar.pack(fill="x")
        self.start_button = ttk.Button(toolbar,text="실물 주행 시작" if self.args.drive else "제어값 미리보기 시작",command=self.start)
        self.start_button.pack(side="left",padx=(0,10))
        tk.Button(toolbar,text="정지  Esc",command=self.stop,bg="#b42318",fg="white",
                  activebackground="#8c1b13",activeforeground="white",font=("맑은 고딕",12,"bold"),
                  padx=22,pady=5,relief="flat").pack(side="left",padx=(0,20))
        self.click_mode = tk.StringVar(value="목표점")
        ttk.Label(toolbar,text="지도 클릭:").pack(side="left",padx=(0,6))
        ttk.Combobox(toolbar,textvariable=self.click_mode,values=["목표점","장애물 사각형"],state="readonly",width=13).pack(side="left")
        ttk.Button(toolbar,text="수동 장애물 지우기",command=self.clear_obstacles).pack(side="left",padx=8)
        self.status = tk.StringVar(value="카메라 연결 중")
        ttk.Label(self.root,textvariable=self.status,font=("맑은 고딕",11,"bold"),padding=(18,4),
                  wraplength=1200).pack(fill="x")
        self.telemetry = tk.StringVar(value="")
        ttk.Label(self.root,textvariable=self.telemetry,padding=(18,4)).pack(fill="x")
        if self.measure_replan:
            self.replan_readout = tk.StringVar(value="재계산 측정 대기 · 파란 경로가 생긴 뒤 경로 중간을 막아 보세요.")
            ttk.Label(self.root,textvariable=self.replan_readout,padding=(18,4),wraplength=890).pack(fill="x")
        ttk.Label(self.root,text="지도: 흰색 통행 관측 · 진한색 장애물 · 회색 미측정 · 연분홍 여유 거리 / 보정 후 카메라를 움직이면 다시 보정하세요.",
                  padding=(18,6),wraplength=1200).pack(fill="x")

    def stop(self, reason="정지 버튼 또는 Esc"):
        self.navigator.stop(reason)
        self.replan_frame = None
        if self.worker:
            self.worker.submit(stop_reason=reason)
        if self.args.simulate:
            self.camera.velocity = self.camera.rotation = 0

    def reset_trial(self):
        self.stop("시험 구역 다시 읽기")
        self.session.calibration = self.session.grid = self.session.pose = None
        self.session.trial_reference = None
        self.session.healthy = False
        self.navigator.goal,self.navigator.path = None,[]
        self.trial_goal = None
        self.trial_retry_at = 0.0
        self.last_processed = None

    def prepare_trial(self,packet,now):
        if now<self.trial_retry_at or not 0<=now-packet["stamp"]<=.35:
            return
        self.trial_retry_at = now+.6
        try:
            if self.args.forward_avoid:
                calibration = fit_forward_floor(packet,self.session.tracker.detector,
                    self.args.marker_id,self.args.robot_marker_height)
                self.session.calibrate(calibration,packet)
                self.last_processed = None
                self.notice.set("로봇의 현재 정면으로 경로를 찾습니다. 파란 경로가 보이면 시작할 수 있습니다.")
                return
            calibration,goal,reference = fit_tag_trial(packet,self.session.tracker.detector,
                self.args.marker_id,self.args.robot_marker_height)
            self.session.calibrate(calibration,packet)
            self.session.trial_reference = reference
            self.trial_goal = goal
            self.navigator.set_goal(goal)
            self.last_processed = None
            self.notice.set("ID 10을 목표로 고정했습니다. 첫 시험은 빈 바닥에서, 두 번째 시험은 상자를 놓고 경로를 확인하세요.")
        except (ValueError,cv2.error,np.linalg.LinAlgError) as error:
            self.notice.set(str(error))

    def start(self):
        if self.frozen is not None or not self.session.healthy or self.session.calibration is None:
            self.notice.set("보정 완료 후 마커와 깊이 측정이 정상일 때 시작할 수 있습니다.")
            return
        if self.navigator.goal is None or not self.navigator.path:
            self.notice.set("통행 가능한 목표점을 선택하고 경로가 표시되는지 확인하세요.")
            return
        if self.worker and self.worker.fault:
            self.notice.set(self.worker.fault+" — 프로그램을 종료하고 연결 상태를 확인하세요.")
            return
        self.navigator.last_stop_reason = None
        self.navigator.running = True
        self.notice.set("Esc 또는 정지 버튼으로 중단합니다. 통신이 끊기면 본체 버튼으로 멈추세요.")

    def auto_calibrate(self):
        self.stop("자동 보정 시작")
        self.frozen = self.pending_calibration = None
        self.points = []
        self.save_button.configure(state="disabled")
        packet,error = self.camera.latest()
        if packet is None or not 0 <= time.monotonic()-packet["stamp"] <= .35:
            self.notice.set(error or "최신 D455 영상이 필요합니다. 영상이 들어오면 다시 누르세요.")
            return
        try:
            width,height,z,side = [float(self.fields[k].get()) for k in ("width","height","marker_height","floor_side")]
        except ValueError:
            self.notice.set("중심 간 가로·세로(m), 로봇 마커 높이(m), 바닥 마커 한 변(mm)을 숫자로 입력하세요.")
            return
        try:
            calibration,centres = calibrate_floor_markers(self.session.tracker.detector,packet,width,height,z,side)
        except (ValueError,cv2.error) as error:
            self.notice.set(str(error))
            return
        self.frozen = packet
        self.points = centres
        self.pending_calibration = calibration
        height = f"마커 기준 {calibration.camera[2]:.2f}m"
        if "floor_depth_scale" in calibration.data:
            height += f" · 깊이 기준 {calibration.camera[2]/calibration.floor_depth_scale:.2f}m"
        self.notice.set(f"ID 10–13 인식 · 오차 {calibration.data['reprojection_rms_px']:.2f}px · {height}. "
                        "실측과 맞으면 보정 저장을 누르세요.")
        self.save_button.configure(state="normal")
        self._show_video(packet["frame"])

    def begin_calibration(self):
        self.stop("수동 보정 시작")
        if self.packet is None:
            self.notice.set("카메라 영상을 먼저 받아야 합니다.")
            return
        try:
            width,height,z = [float(self.fields[k].get()) for k in ("width","height","marker_height")]
            if not np.isfinite([width,height,z]).all() or min(width,height)<=0 or z<0:
                raise ValueError()
        except ValueError:
            self.notice.set("바닥 가로·세로는 양수, 로봇 마커 높이는 0 이상의 미터 값으로 입력하세요.")
            return
        self.frozen = dict(self.packet)
        self.points = []
        self.pending_calibration = None
        self.save_button.configure(state="disabled")
        self.notice.set("바닥 점 0/4 — 왼쪽 영상에서 왼쪽 위 → 오른쪽 위 → 오른쪽 아래 → 왼쪽 아래를 클릭하세요.")
        self._show_video(self.frozen["frame"])

    def video_click(self,event):
        if self.frozen is None:
            self.notice.set("치수를 입력하고 '바닥 네 점 지정'을 먼저 누르세요.")
            return
        if self.video_transform is None or len(self.points)>=4:
            return
        ox,oy,scale = self.video_transform
        x,y = (event.x-ox)/scale,(event.y-oy)/scale
        frame = self.frozen["frame"]
        if not (0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]):
            self.notice.set("영상의 검은 여백이 아닌 바닥 모서리를 클릭하세요.")
            return
        self.points.append((x,y))
        # Repaint synchronously: point selection must work even if live camera
        # acquisition has stalled or disconnected after this frame was frozen.
        self._show_video(frame)
        next_corner = ["왼쪽 위","오른쪽 위","오른쪽 아래","왼쪽 아래"]
        if len(self.points)<4:
            self.notice.set("바닥 점 {}/4 — 다음: {} 모서리를 클릭하세요.".format(
                len(self.points),next_corner[len(self.points)]))
        if len(self.points)==4:
            try:
                self.pending_calibration = FloorCalibration.fit(self.points,
                    *[float(self.fields[k].get()) for k in ("width","height","marker_height")],
                    self.frozen["ray"],self.frozen["metadata"])
                self.notice.set("보정 오차 {:.2f}px / 추정 카메라 높이 {:.2f}m. 실측과 맞으면 보정 저장을 누르세요.".format(
                    self.pending_calibration.data["reprojection_rms_px"],self.pending_calibration.camera[2]))
                self.save_button.configure(state="normal")
            except Exception as error:
                self.notice.set(str(error)+" — 바닥 네 점 지정을 다시 누르세요.")

    def save_calibration(self):
        if self.pending_calibration is None:
            return
        try:
            calibration = self.pending_calibration
            expected = {"width":calibration.width,"height":calibration.height,"marker_height":calibration.marker_height}
            if "floor_marker_side_mm" in calibration.data:
                expected["floor_side"] = calibration.data["floor_marker_side_mm"]
            try:
                unchanged = all(math.isclose(float(self.fields[k].get()),v) for k,v in expected.items())
            except ValueError:
                unchanged = False
            if not unchanged:
                self.notice.set("치수가 바뀌었습니다. 바닥 보정을 다시 계산하세요.")
                return
            self.session.calibrate(calibration,self.packet)
            calibration.save(self.args.calibration)
            self.frozen = None
            self.pending_calibration = None
            self.last_processed = None
            self.navigator.goal,self.navigator.path = None,[]
            self.save_button.configure(state="disabled")
            self.notice.set("보정을 저장했습니다. 지도의 위치를 실제 바닥과 비교한 뒤 목표점을 선택하세요.")
        except Exception as error:
            self.notice.set(str(error))

    def load_calibration(self):
        self.stop("저장된 보정 불러오기")
        if self.packet is None:
            return
        try:
            calibration = FloorCalibration.load(self.args.calibration)
            self.session.calibrate(calibration,self.packet)
            for key,value in [("width",calibration.width),("height",calibration.height),("marker_height",calibration.marker_height)]:
                self.fields[key].set(str(value))
            self.fields["floor_side"].set(str(round(calibration.data.get("floor_marker_side_mm",FLOOR_SIDE_MM),3)))
            self.frozen = None
            self.pending_calibration = None
            self.last_processed = None
            self.save_button.configure(state="disabled")
            self.navigator.goal,self.navigator.path = None,[]
            self.notice.set("저장된 보정을 적용했습니다. 카메라/바닥 기준점이 바뀌었다면 새로 보정하세요.")
        except Exception as error:
            self.notice.set("보정을 사용할 수 없습니다: "+str(error))

    def map_click(self,event):
        if self.session.grid is None or self.map_transform is None:
            return
        self.stop("주행 중 지도 클릭")
        ox,oy,scale = self.map_transform
        point = ((event.x-ox)/scale,(oy-event.y)/scale)
        grid = self.session.grid
        if not (0 <= point[0] < grid.width and 0 <= point[1] < grid.height):
            return
        if self.click_mode.get()=="장애물 사각형":
            if self.obstacle_first is None:
                self.obstacle_first = point
                self.notice.set("장애물 사각형의 반대쪽 모서리를 클릭하세요.")
            else:
                grid.add_rectangle(self.obstacle_first,point)
                self.obstacle_first = None
                self.navigator.path = []
                self.notice.set("수동 장애물을 추가했습니다. 새 경로를 확인하세요.")
        else:
            self.obstacle_first = None
            if self.args.tag_test:
                self.notice.set("이 모드에서는 ID 10이 목표로 지정됩니다.")
                return
            if self.args.forward_avoid:
                self.trial_goal = point
            self.navigator.set_goal(point)
            self.notice.set("목표점 ({:.2f}, {:.2f})m을 선택했습니다.".format(*point))

    def clear_obstacles(self):
        self.stop("수동 장애물 지우기")
        if self.session.grid:
            self.session.grid.static.fill(False)
            self.navigator.path = []
        self.obstacle_first = None

    def _show_video(self,frame):
        image = Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
        if self.frozen is not None:
            draw = ImageDraw.Draw(image)
            names = self.pending_calibration.data.get("floor_marker_ids") if self.pending_calibration else None
            for i,p in enumerate(self.points):
                x,y = p
                draw.ellipse((x-5,y-5,x+5,y+5),fill="#e88616",outline="white",width=2)
                draw.text((x+8,y-12),str(names[i] if names else i+1),font=font(18),fill="#e88616")
            if len(self.points)>1:
                outline = self.points+[self.points[0]] if len(self.points)==4 else self.points
                draw.line(outline,fill="#e88616",width=3)
        w,h = max(100,self.video_label.winfo_width()),max(100,self.video_label.winfo_height())
        scale = min(w/image.width,h/image.height)
        resized = image.resize((max(1,round(image.width*scale)),max(1,round(image.height*scale))))
        canvas = Image.new("RGB",(w,h),"#18212d")
        ox,oy = (w-resized.width)//2,(h-resized.height)//2
        canvas.paste(resized,(ox,oy))
        self.video_transform = ox,oy,scale
        self.video_photo = ImageTk.PhotoImage(canvas)
        self.video_label.itemconfigure(self.video_item,image=self.video_photo)

    def tick(self):
        if self.closed:
            return
        now = time.monotonic()
        try:
            packet,error = self.camera.latest()
            if packet:
                self.packet = packet
                if self.quick_trial and self.session.calibration is None:
                    self.prepare_trial(packet,now)
                if self.args.simulate and self.session.calibration is None:
                    self.session.calibrate(self.camera.calibration,packet)
                    self.navigator.set_goal((2.3,1.45))
                    for key,value in [("width",3),("height",2),("marker_height",.12)]:
                        self.fields[key].set(str(value))
                processed_from = None
                if packet["number"] != self.last_processed:
                    if self.measure_replan:
                        processed_from = time.perf_counter()
                    self.video_image = self.session.process(packet,annotate=False)
                    self.last_processed = packet["number"]
                map_ready = time.perf_counter() if processed_from is not None else None
                now = time.monotonic()
                fresh = 0 <= now-packet["stamp"] <= .35
                healthy = self.session.healthy and fresh and self.frozen is None
                if not fresh:
                    measurement_issue = "D455 영상 갱신 지연"
                elif self.frozen is not None:
                    measurement_issue = "보정 화면 고정 중"
                else:
                    measurement_issue = self.session.message
                if self.args.forward_avoid and healthy and self.trial_goal is None:
                    self.trial_goal = forward_target(self.session.pose,self.session.grid,self.args.forward_distance)
                    if self.trial_goal is not None:
                        self.navigator.set_goal(self.trial_goal)
                pending_before = self.navigator.blocked_route
                stop_confirmed = self.worker is None or self.worker.stop_confirmed()
                velocity,rotation = self.navigator.step(self.session.pose,self.session.grid,now,healthy,
                    stop_confirmed=stop_confirmed,verify_stop=self.worker is not None,
                    measurement_issue=measurement_issue if not healthy else None)
                enabled = healthy and self.navigator.running and bool(velocity or rotation)
                if self.worker:
                    stop_reason = self.navigator.last_stop_reason or self.navigator.message
                    self.worker.submit(velocity,rotation,packet["stamp"],enabled,stop_reason=stop_reason)
                    if self.worker.fault:
                        self.navigator.stop(self.worker.fault)
                if (self.measure_replan and processed_from is not None and pending_before is None
                        and self.navigator.blocked_route is not None):
                    self.replan_frame = (packet["number"],packet["received_perf"],processed_from,map_ready)
                if self.measure_replan and self.replan_frame and self.navigator.last_replan:
                    row = measure_replan(*self.replan_frame,self.navigator.last_replan,
                        "simulated" if self.args.simulate else "D455-drive" if self.worker else "D455-preview")
                    cause = "깊이 장애물 감지" if row["cause"]=="depth_obstacle" else "기타 경로 차단"
                    result = "경로 찾음" if row["waypoints"] else "경로 없음"
                    self.replan_readout.set(
                        f"{cause} · 프레임 수신→재계산 {row['frame_to_plan_ms']:.1f}ms "
                        f"(정지 대기 {row['stop_wait_ms']:.1f}ms, 경로 {row['plan_ms']:.1f}ms) · {result}")
                    try:
                        append_measurement(ROOT/"runtime"/"replan_timing_stop_first.csv",row)
                    except OSError as error:
                        print(f"재계산 기록 저장 오류: {error}")
                if self.navigator.blocked_route is None:
                    self.replan_frame = None
                if self.args.simulate:
                    self.camera.velocity,self.camera.rotation = (velocity,rotation) if enabled else (0,0)
                grid = self.session.grid
                if now >= self.next_render_at:
                    self.next_render_at = now+.17
                    frame = self.frozen["frame"] if self.frozen is not None else self.session.annotate_frame(self.video_image)
                    self._show_video(frame)
                    display_pose = self.session.pose
                    previous = self.session.previous_pose
                    if display_pose is None and previous and 0 <= now-previous.stamp <= .35:
                        display_pose = previous
                    image,self.map_transform = map_image(grid,display_pose,self.navigator.path,self.navigator.goal,
                        (max(100,self.map_label.winfo_width()),max(100,self.map_label.winfo_height())),
                        pose_stale=display_pose is not self.session.pose)
                    self.map_photo = ImageTk.PhotoImage(image)
                    self.map_label.itemconfigure(self.map_item,image=self.map_photo)
                mode = self.worker.status if self.worker else ("모의 주행" if self.args.simulate else "미리보기")
                reason = measurement_issue
                if healthy:
                    reason = self.navigator.message
                    if self.quick_trial and not self.navigator.running and self.navigator.blocked_route is None:
                        pose = self.session.pose
                        if self.navigator.path:
                            route = "직선 경로" if len(self.navigator.path)==2 else "우회 경로"
                            target_name = "전방 주행 시험" if self.args.forward_avoid else "ID 10"
                            reason = f"{target_name} · {route} 준비"
                        elif not grid.free((pose.x,pose.y)):
                            reason = "출발점 주변 여유 부족 · 로봇 주변 바닥을 비우세요"
                        elif self.trial_goal is None:
                            reason = "전방 통행 공간 부족 · 화면 안쪽의 빈 바닥으로 로봇을 향하게 하세요"
                        elif not grid.free(self.trial_goal):
                            reason = ("전방 시험 구간이 막혔습니다" if self.args.forward_avoid else
                                "ID 10 주변 여유 부족 · 목표를 카메라 화면 안쪽에 놓으세요")
                if self.worker and self.worker.fault:
                    reason = self.worker.fault
                    previous = self.navigator.last_stop_reason
                    if previous and previous != reason:
                        reason = f"{reason} · 정지 계기 {previous}"
                elif self.navigator.last_stop_reason and not self.navigator.running:
                    reason = f"마지막 정지 사유 · {self.navigator.last_stop_reason}"
                self.status.set(mode+"  |  "+reason)
                pose = self.session.pose
                text = "위치 ({:.2f}, {:.2f})m · 방향 {:.1f}°".format(pose.x,pose.y,math.degrees(pose.yaw)) if pose else "위치 측정 대기"
                if pose and self.navigator.goal is not None:
                    distance = math.dist((pose.x,pose.y),self.navigator.goal)
                    text += f" · 목표까지 {distance*100:.0f}cm"
                self.telemetry.set(text+"  |  마커 {:.0f}px · 깊이 관측 {:.0f}%  |  제어 {:.2f}m/s, {}°/s".format(
                    self.session.marker_pixels,(grid.coverage*100) if grid else 0,velocity,rotation)+
                    f" · 영상 처리 시점 {max(0,now-packet['stamp'])*1000:.0f}ms")
                can_start = healthy and self.navigator.path and not (self.worker and self.worker.fault)
                self.start_button.configure(state="normal" if can_start else "disabled")
            else:
                self.stop(error or "D455 영상 없음")
                if self.frozen is not None:
                    self._show_video(self.frozen["frame"])
                self.start_button.configure(state="disabled")
                self.status.set(error or "영상 대기")
        except Exception as error:
            self.stop("오류: "+str(error))
            self.status.set("오류: "+str(error))
        self.root.after(50,self.tick)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.stop("프로그램 종료")
        if self.worker:
            self.worker.close()
        self.camera.close()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="D455 목표점 주행 / 기본은 실제 명령 없는 미리보기")
    parser.add_argument("--drive",action="store_true",help="시작 버튼을 누르면 실제 주행")
    parser.add_argument("--tag-test",action="store_true",help="바닥 ID 10·11을 깊이로 읽고 ID 10을 목표로 설정")
    parser.add_argument("--forward-avoid",action="store_true",help="로봇 정면으로 이동하고 전방 장애물을 우회")
    parser.add_argument("--measure-replan",action="store_true",help="기존 경로 차단 시 프레임 수신부터 재계산까지 측정")
    parser.add_argument("--forward-distance",type=float,default=1.2,help="전방 우회 시험의 최대 직선 거리(m)")
    parser.add_argument("--robot-marker-height",type=float,help="빠른 주행 시험에서 사용할 로봇 태그 실측 높이(m)")
    parser.add_argument("--simulate",action="store_true",help="실제 장치 없는 모의 화면")
    parser.add_argument("--ip",default="192.168.0.12")
    parser.add_argument("--marker-id",type=int,choices=ROBOT_IDS,default=0,help="추적할 로봇 마커 ID")
    parser.add_argument("--turn-sign",type=int,choices=[-1,1],default=1)
    parser.add_argument("--resolution",choices=["640x480","1280x720"],default="640x480",
                        help="3m 설치에서는 1280x720으로 마커 인식 크기를 확인하세요")
    parser.add_argument("--robot-radius",type=float,default=.18)
    parser.add_argument("--clearance",type=float,default=.15)
    parser.add_argument("--calibration",type=Path)
    args = parser.parse_args()
    if args.simulate and args.drive:
        parser.error("--simulate와 --drive는 함께 사용할 수 없습니다")
    if args.tag_test and args.forward_avoid:
        parser.error("--tag-test와 --forward-avoid 중 하나만 지정하세요")
    if args.simulate and (args.tag_test or args.forward_avoid):
        parser.error("이 시험 모드는 실제 D455를 사용합니다")
    if not .25 <= args.forward_distance <= 2:
        parser.error("--forward-distance는 .25~2m로 입력하세요")
    if args.robot_marker_height is not None and not ((args.tag_test or args.forward_avoid) and .02<=args.robot_marker_height<=.35):
        parser.error("--robot-marker-height는 태그 또는 전방 시험에서 0.02~0.35m로 입력하세요")
    if args.calibration is None:
        filename = "sim_floor_calibration.json" if args.simulate else "floor_calibration.json"
        args.calibration = ROOT/"runtime"/filename
    if not .17 <= args.robot_radius <= .5 or not .10 <= args.clearance <= .5:
        parser.error("로봇 반경은 0.17~0.5m, 여유 거리는 0.10~0.5m로 입력하세요")
    token = getpass.getpass("청소기 32자리 토큰: ").strip() if args.drive else None
    app = App(args,token)
    try:
        app.root.mainloop()
    finally:
        app.close()


if __name__ == "__main__":
    main()
