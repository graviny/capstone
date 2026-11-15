"""RGB/depth diagnostics before floor calibration is available."""
from pathlib import Path
import json
import numpy as np


def depth_stats(frame):
    valid = frame.depth_m[np.isfinite(frame.depth_m) & (frame.depth_m > 0)]
    return dict(serial=frame.serial, timestamp=frame.timestamp,
                width=frame.intrinsics.width, height=frame.intrinsics.height,
                valid_depth_fraction=float(len(valid)/frame.depth_m.size),
                depth_percentiles_m=(np.percentile(valid, [5, 50, 95]).tolist()
                                     if len(valid) else None))


class RGBDPreview:
    def __init__(self):
        import matplotlib.pyplot as plt
        self.plt = plt
        self.fig, self.axes = plt.subplots(1, 2, figsize=(13, 5))
        self.rgb_artist = self.depth_artist = None

    def update(self, frame):
        stats = depth_stats(frame)
        percentiles = stats["depth_percentiles_m"]
        maximum = max(.1, percentiles[-1]) if percentiles else 3.
        depth = np.ma.masked_where(~np.isfinite(frame.depth_m) | (frame.depth_m <= 0), frame.depth_m)
        if self.rgb_artist is None:
            self.rgb_artist = self.axes[0].imshow(frame.rgb)
            self.depth_artist = self.axes[1].imshow(depth, cmap="turbo", vmin=0, vmax=maximum)
            self.fig.colorbar(self.depth_artist, ax=self.axes[1], label="Optical depth (m)")
        else:
            self.rgb_artist.set_data(frame.rgb)
            self.depth_artist.set_data(depth)
            self.depth_artist.set_clim(0, maximum)
        self.axes[0].set_title("D455 RGB (rectified)")
        self.axes[1].set_title(f"Aligned depth | valid: {stats['valid_depth_fraction']:.1%}")
        self.fig.tight_layout()
        self.fig.canvas.draw_idle()
        return stats

    def save(self, frame, directory):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        stats = self.update(frame)
        self.fig.savefig(directory / "rgb_depth.png", dpi=140)
        (directory / "diagnostics.json").write_text(json.dumps(stats, indent=2), encoding="utf-8")
        return stats
