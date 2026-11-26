"""Deterministic RGB/depth fixture: no physical camera or robot is used."""
import math
import time

import cv2
import numpy as np

from camera_demo import dictionary
from floor_markers import FLOOR_IDS, FLOOR_SIDE_MM, floor_corners
from navigation_core import FloorCalibration, Pose


class SimulatedCamera:
    def __init__(self, marker_id=0, show_floor_tags=False):
        self.pose = Pose(0.65, 0.55, 0, time.monotonic())
        self.velocity, self.rotation = 0.0, 0
        self.last = time.monotonic()
        self.number = 0
        self.K = np.array([[420,0,320],[0,420,240],[0,0,1]],float)
        self.R = np.diag([1.,-1.,-1.])
        self.camera = np.array([1.5,1.0,2.5])
        self.t = -self.R@self.camera
        self.metadata = {"device": {"serial":"SIMULATED","name":"모의 카메라"},
                         "color_intrinsics": {"width":640,"height":480,"fx":420.,"fy":420.,
                                              "ppx":320.,"ppy":240.,"model":"none","coeffs":[0.]*5}}
        self.calibration = FloorCalibration({"width_m":3.,"height_m":2.,"marker_height_m":0.12,
            "world_to_camera_R":self.R.tolist(),"world_to_camera_t":self.t.tolist(),
            "camera_metadata":self.metadata,"reference_pixels":self.project([[0,2,0],[3,2,0],[3,0,0],[0,0,0]]).tolist(),
            "reprojection_rms_px":0.})
        self.marker = cv2.aruco.generateImageMarker(dictionary(),marker_id,180)
        self.floor_tags = []
        if show_floor_tags:
            for tag_id,square in zip(FLOOR_IDS,floor_corners(3,2,FLOOR_SIDE_MM)):
                left,top = np.rint(self.project(square)[0]).astype(int)
                tag = cv2.aruco.generateImageMarker(dictionary(),tag_id,27)
                self.floor_tags.append((left-6,top-6,cv2.copyMakeBorder(tag,6,6,6,6,cv2.BORDER_CONSTANT,value=255)))

    def ray(self, pixel):
        return np.array([(pixel[0]-320)/420,(pixel[1]-240)/420,1.])

    def project(self, points):
        camera = np.asarray(points)@self.R.T+self.t
        return (camera[:,:2]/camera[:,2,None])*[420,420]+[320,240]

    def latest(self):
        now = time.monotonic()
        dt = min(now-self.last,0.1)
        self.last = now
        self.pose.yaw += math.radians(self.rotation)*dt
        self.pose.x += math.cos(self.pose.yaw)*self.velocity*dt
        self.pose.y += math.sin(self.pose.yaw)*self.velocity*dt
        self.pose.stamp = now
        frame = np.full((480,640,3),225,np.uint8)
        depth = np.zeros((480,640),np.uint16)
        floor = self.project([[0,2,0],[3,2,0],[3,0,0],[0,0,0]]).astype(np.int32)
        cv2.fillConvexPoly(frame,floor,(245,246,248))
        cv2.fillConvexPoly(depth,floor,2500)
        for x in np.arange(0,3.01,0.25):
            p = self.project([[x,0,0],[x,2,0]]).astype(int)
            cv2.line(frame,tuple(p[0]),tuple(p[1]),(220,224,229),1)
        for y in np.arange(0,2.01,0.25):
            p = self.project([[0,y,0],[3,y,0]]).astype(int)
            cv2.line(frame,tuple(p[0]),tuple(p[1]),(220,224,229),1)
        box = self.project([[1.35,.8,.35],[1.65,.8,.35],[1.65,1.2,.35],[1.35,1.2,.35]]).astype(np.int32)
        cv2.fillConvexPoly(frame,box,(90,120,160))
        cv2.fillConvexPoly(depth,box,2150)
        for left,top,tag in self.floor_tags:
            h,w = tag.shape
            frame[top:top+h,left:left+w] = tag[:,:,None]
        angle = self.pose.yaw
        front = np.array([math.cos(angle),math.sin(angle)])
        left = np.array([-math.sin(angle),math.cos(angle)])
        centre = np.array([self.pose.x,self.pose.y])
        corners = [np.r_[centre+f*.10*front+l*.10*left,.12]
                   for f,l in [(1,1),(1,-1),(-1,-1),(-1,1)]]
        image_corners = self.project(corners).astype(np.float32)
        matrix = cv2.getPerspectiveTransform(np.array([[0,0],[179,0],[179,179],[0,179]],np.float32),image_corners)
        warped = cv2.warpPerspective(self.marker,matrix,(640,480),borderValue=255,flags=cv2.INTER_NEAREST)
        mask = np.zeros((480,640),np.uint8)
        cv2.fillConvexPoly(mask,image_corners.astype(np.int32),255)
        # White quiet zone around the black marker.
        quiet = cv2.dilate(mask,np.ones((11,11),np.uint8))
        frame[quiet>0] = 255
        frame[mask>0] = cv2.cvtColor(warped,cv2.COLOR_GRAY2BGR)[mask>0]
        depth[quiet>0] = 2380
        self.number += 1
        return {"frame":frame,"depth":depth,"scale":.001,"stamp":now,"received_perf":time.perf_counter(),"number":self.number,
                "metadata":self.metadata,"ray":self.ray}, ""

    def close(self):
        pass
