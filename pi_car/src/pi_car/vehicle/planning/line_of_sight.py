"""Conservative straight-segment checks on an already inflated navigation grid."""
import math


def segment_is_free(grid, start, goal):
    """Check every closed grid cell touched by the segment, including corners.

    Checking intersections with blocked-cell rectangles avoids thin-obstacle
    misses from point sampling. Unknown/outside cells remain blocked. Inflation
    is the caller's responsibility, exactly as for A*.
    """
    if not all(math.isfinite(v) for v in (*start,*goal)):
        return False
    if not grid.is_free(grid.world_to_cell(*start)) or not grid.is_free(grid.world_to_cell(*goal)):
        return False
    m=grid.metadata
    a=((start[0]-m.origin_xy[0])/m.resolution_m,
       (start[1]-m.origin_xy[1])/m.resolution_m)
    b=((goal[0]-m.origin_xy[0])/m.resolution_m,
       (goal[1]-m.origin_xy[1])/m.resolution_m)
    delta=(b[0]-a[0],b[1]-a[1])

    def intersects(x,y):
        lower,upper=0.,1.
        for origin,d,edge in zip(a,delta,(x,y)):
            if abs(d)<1e-12:
                if origin<edge-1e-12 or origin>edge+1+1e-12:
                    return False
            else:
                t0,t1=(edge-origin)/d,(edge+1-origin)/d
                lower=max(lower,min(t0,t1))
                upper=min(upper,max(t0,t1))
                if lower>upper+1e-12:
                    return False
        return True

    for y in range(math.floor(min(a[1],b[1]))-1,math.floor(max(a[1],b[1]))+1):
        for x in range(math.floor(min(a[0],b[0]))-1,math.floor(max(a[0],b[0]))+1):
            if not grid.is_free((x,y)) and intersects(x,y):
                return False
    return True
