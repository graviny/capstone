"""Time replans triggered by an existing route becoming blocked."""
import csv
from datetime import datetime


def measure_replan(frame_number, received, processed_from, map_ready, replan, source):
    def ms(interval):
        return round(interval*1000,3)

    return {
        "time":datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "source":source,
        "frame":frame_number,
        "cause":"depth_obstacle" if replan.depth_obstacle else "other_blockage",
        "result":"path_found" if replan.route_points else "no_path",
        "waypoints":replan.route_points,
        "frame_wait_ms":ms(processed_from-received),
        "map_update_ms":ms(map_ready-processed_from),
        "path_check_ms":ms(replan.blocked_at-map_ready),
        "stop_wait_ms":ms(replan.started-replan.blocked_at),
        "plan_ms":ms(replan.finished-replan.started),
        "frame_to_plan_ms":ms(replan.finished-received),
    }


def append_measurement(path, row):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open("a",encoding="utf-8",newline="") as stream:
        writer = csv.DictWriter(stream,fieldnames=row.keys())
        if stream.tell()==0:
            writer.writeheader()
        writer.writerow(row)
