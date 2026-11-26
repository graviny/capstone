from pathlib import Path
import json
import math
import numpy as np
from pi_car.vehicle.planning import plan


class MapInspector:
    """Desktop tool only. Clicks never send motor/mission commands."""

    def __init__(self, grid, clearance_m, output_directory):
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap, BoundaryNorm
        self.plt, self.grid = plt, grid
        self.navigation = grid.inflated(clearance_m)
        self.output = Path(output_directory)
        self.output.mkdir(parents=True, exist_ok=True)
        self.start, self.goal, self.pose, self.path = None, None, None, []
        self.manual_start = False
        self.frame = self.calibration = None
        self.fig, (self.camera_ax, self.map_ax) = plt.subplots(1, 2, figsize=(13, 6))
        self.camera_ax.set_title("RGB: click to inspect measured XYZ / select floor goal")
        self.camera_ax.axis("off")
        m = grid.metadata
        extent = [m.origin_xy[0], m.origin_xy[0]+m.width*m.resolution_m,
                  m.origin_xy[1], m.origin_xy[1]+m.height*m.resolution_m]
        cmap = ListedColormap(["#9197a0", "#fafafa", "#262b33"])
        self.map_ax.imshow(grid.cells, origin="lower", extent=extent,
                           cmap=cmap, norm=BoundaryNorm([-1.5, -.5, .5, 1.5], 3))
        expanded = np.ma.masked_where((grid.cells != 0) | (self.navigation.cells == 0),
                                     np.ones(grid.cells.shape))
        self.map_ax.imshow(expanded, origin="lower", extent=extent,
                           cmap=ListedColormap(["#efb366"]), alpha=.65, vmin=0, vmax=1)
        self.map_ax.set(xlabel="world X (m)", ylabel="world Y (m)",
                        title="Left: goal | Right: manual start | r: use live pose")
        self.path_artist, = self.map_ax.plot([], [], "-", color="#3486d8", lw=2)
        self.start_artist, = self.map_ax.plot([], [], "bo", label="start")
        self.goal_artist, = self.map_ax.plot([], [], "r*", markersize=14, label="goal")
        self.pose_artist, = self.map_ax.plot([], [], "go", label="AprilTag vehicle center")
        self.heading_artist, = self.map_ax.plot([], [], "g-", lw=3)
        self.map_ax.legend(loc="upper right")
        self.rgb_artist = None
        self.status = self.fig.text(.02, .02, "Gray: unknown | Black: obstacle | Orange: clearance")
        self.fig.subplots_adjust(bottom=.15)
        self.fig.canvas.mpl_connect("button_press_event", self._click)
        self.fig.canvas.mpl_connect("key_press_event", self._key)

    def _key(self, event):
        if event.key == "r":
            self.manual_start = False
            self.start = (self.pose.x, self.pose.y) if self.pose else None
            self._replan()

    def _click(self, event):
        if event.xdata is None or event.ydata is None:
            return
        # Avoid accidental goals while the Matplotlib pan/zoom tool is active.
        toolbar = getattr(self.fig.canvas, "toolbar", None)
        if toolbar and toolbar.mode:
            return
        x, y = event.xdata, event.ydata
        if event.inaxes == self.camera_ax and self.frame is not None:
            u, v = int(round(x)), int(round(y))
            if not (0 <= u < self.frame.intrinsics.width and 0 <= v < self.frame.intrinsics.height):
                return
            depth = self.frame.depth_m[v, u]
            if not np.isfinite(depth) or depth <= 0:
                self._message("No depth at selected RGB pixel")
                return
            xyz = self.calibration.transform(self.frame.intrinsics.deproject([u, v], depth))
            self._message(f"RGB pixel ({u},{v}) → world XYZ {xyz.round(3).tolist()} m")
            print(self.status.get_text())
            if abs(xyz[2]) > .03:
                self._message("Selected surface is not floor (|Z| > 0.03m); goal rejected")
                return
            x, y = xyz[:2]
        elif event.inaxes != self.map_ax:
            return
        cell = self.navigation.world_to_cell(x, y)
        if not self.navigation.is_free(cell):
            self._message("Rejected: unknown, obstacle, clearance region or outside map")
            return
        snapped = self.grid.cell_to_world(*cell)
        if event.button == 3:
            self.start, self.manual_start = snapped, True
        elif event.button == 1:
            self.goal = snapped
            goal = dict(type="goal", schema_version=1, frame_id=self.grid.metadata.frame_id,
                        map_version=self.grid.metadata.version, goal=list(snapped),
                        requested_xy=[float(x), float(y)])
            (self.output / "goal.json").write_text(json.dumps(goal, indent=2), encoding="utf-8")
        else:
            return
        self._replan()

    def _message(self, message):
        self.status.set_text(message)
        self.fig.canvas.draw_idle()

    def _replan(self):
        self.path = []
        message = f"start={self.start} goal={self.goal}"
        if self.start and self.goal:
            try:
                self.path = plan(self.navigation, self.start, self.goal)
                length = sum(math.dist(a, b) for a, b in zip(self.path, self.path[1:]))
                message = f"A*: {len(self.path)} waypoints, {length:.2f} m | goal={self.goal}"
            except ValueError as error:
                message = str(error)
        self.path_artist.set_data(*zip(*self.path) if self.path else ([], []))
        self.start_artist.set_data(*([self.start[0]], [self.start[1]]) if self.start else ([], []))
        self.goal_artist.set_data(*([self.goal[0]], [self.goal[1]]) if self.goal else ([], []))
        payload = dict(frame_id=self.grid.metadata.frame_id, map_version=self.grid.metadata.version,
                       path=self.path)
        (self.output / "path.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
        self._message(message)

    def update(self, frame, calibration, pose):
        if calibration.frame_id != self.grid.metadata.frame_id:
            raise ValueError("Map and localization calibration IDs differ: rebuild map")
        calibration.check_frame(frame)
        self.frame, self.calibration, self.pose = frame, calibration, pose
        if self.rgb_artist is None:
            self.rgb_artist = self.camera_ax.imshow(frame.rgb)
        else:
            self.rgb_artist.set_data(frame.rgb)
        if pose:
            self.pose_artist.set_data([pose.x], [pose.y])
            self.heading_artist.set_data([pose.x, pose.x+.15*math.cos(pose.theta)],
                                         [pose.y, pose.y+.15*math.sin(pose.theta)])
            self.map_ax.set_title(f"{pose.robot_id} pose: ({pose.x:.3f}, {pose.y:.3f}) m, {math.degrees(pose.theta):.1f}°")
            (self.output / "pose.json").write_text(json.dumps(pose.to_dict(), indent=2), encoding="utf-8")
            if not self.manual_start:
                self.start = (pose.x, pose.y)
        else:
            self.pose_artist.set_data([], [])
            self.heading_artist.set_data([], [])
            self.map_ax.set_title("TAG LOST / invalid pose")
            (self.output / "pose.json").write_text(json.dumps(dict(valid=False, frame_id=calibration.frame_id)),
                                                    encoding="utf-8")
            if not self.manual_start:
                self.start = None
        self._replan()
