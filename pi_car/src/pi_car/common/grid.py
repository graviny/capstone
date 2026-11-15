from dataclasses import asdict, dataclass
from pathlib import Path
import json
import math
import numpy as np


@dataclass(frozen=True)
class GridMetadata:
    width: int
    height: int
    resolution_m: float
    origin_xy: tuple[float, float]
    frame_id: str
    version: str
    created_at: str
    axes: str = "columns:+X rows:+Y Z:up theta:CCW radians"

    def __post_init__(self):
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Grid dimensions must be positive")
        if not math.isfinite(self.resolution_m) or self.resolution_m <= 0:
            raise ValueError("Grid resolution must be positive and finite")
        if len(self.origin_xy) != 2 or not all(math.isfinite(x) for x in self.origin_xy):
            raise ValueError("Grid origin must contain finite x,y")
        if not self.frame_id:
            raise ValueError("Grid needs a calibration frame ID")


@dataclass
class OccupancyGrid:
    cells: np.ndarray  # [row=y, column=x], -1 unknown, 0 free, 1 occupied
    metadata: GridMetadata

    def __post_init__(self):
        if self.cells.shape != (self.metadata.height, self.metadata.width):
            raise ValueError("Grid and metadata dimensions disagree")
        if not np.isin(self.cells, [-1, 0, 1]).all():
            raise ValueError("Invalid occupancy value")

    def world_to_cell(self, x, y):
        m = self.metadata
        return (math.floor((x - m.origin_xy[0]) / m.resolution_m),
                math.floor((y - m.origin_xy[1]) / m.resolution_m))

    def cell_to_world(self, column, row):
        m = self.metadata
        return (m.origin_xy[0] + (column + .5) * m.resolution_m,
                m.origin_xy[1] + (row + .5) * m.resolution_m)

    def is_free(self, cell):
        x, y = cell
        return (0 <= x < self.metadata.width and 0 <= y < self.metadata.height
                and self.cells[y, x] == 0)

    def inflated(self, radius_m):
        """Conservative disk expansion of occupied/unknown space and map boundary.

        Radius includes car circumscribed radius + margin. One-cell quantization
        allowance prevents treating center-to-center distance as obstacle clearance.
        Raw measurements remain in map.npy; this grid is for navigation only.
        """
        if not math.isfinite(radius_m) or radius_m < 0:
            raise ValueError("Radius must be nonnegative and finite")
        if radius_m == 0:
            return OccupancyGrid(self.cells.copy(), self.metadata)
        radius = radius_m / self.metadata.resolution_m + math.sqrt(2)
        n = math.ceil(radius)
        h, w = self.cells.shape
        source = np.pad(self.cells != 0, n, constant_values=True)
        blocked = np.zeros((h, w), dtype=bool)
        for dy in range(-n, n + 1):
            for dx in range(-n, n + 1):
                if dx * dx + dy * dy <= radius * radius:
                    blocked |= source[n + dy:n + dy + h, n + dx:n + dx + w]
        result = self.cells.copy()
        result[(result == 0) & blocked] = 1
        return OccupancyGrid(result, self.metadata)

    def save(self, directory):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        np.save(directory / "map.npy", self.cells.astype(np.int8), allow_pickle=False)
        (directory / "map_metadata.json").write_text(
            json.dumps(asdict(self.metadata), indent=2), encoding="utf-8")

    @classmethod
    def load(cls, directory):
        directory = Path(directory)
        metadata = GridMetadata(**json.loads((directory / "map_metadata.json").read_text()))
        return cls(np.load(directory / "map.npy", allow_pickle=False), metadata)
