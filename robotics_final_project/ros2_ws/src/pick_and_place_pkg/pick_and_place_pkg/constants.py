from geometry_msgs.msg import Pose

INCH = 0.0254
HOVER_ABOVE_BOARD_M = 2.0 * INCH
CENTER_HOVER_ABOVE_BOARD_M = 8.0 * INCH
PLACE_Z_OFFSET = 0.0
BOARD_Z = 8.2 * INCH
BOARD_ORIGIN = (-6.5 * INCH, 7.8 * INCH, BOARD_Z)
TOP_DOWN = (-0.7071, 0.7071, 0.0, 0.0)
CENTER_HOVER_Z = 0.4166

import math
BOARD_ROTATION_RAD = math.radians(0)
CAPTURE_JOINTS = [math.radians(a) for a in [-11, -21, 65, 90, 94, -99]]

# Square width (m) per column along the letter axis — columns d and e are slightly narrower than the rest.
COLUMN_WIDTHS_M = {
    'a': 1.625 * INCH, 'b': 1.625 * INCH, 'c': 1.625 * INCH,
    'd': 1.5625 * INCH, 'e': 1.5625 * INCH,
    'f': 1.625 * INCH, 'g': 1.625 * INCH, 'h': 1.625 * INCH,
}
# Square height (m) per row along the number axis — uniform for all rows.
ROW_HEIGHT_M = 1.625 * INCH

# Per-column x correction (m) — columns d–h shift 3.5mm right (positive x, toward h) to match the measured board (flat group offset, not compounding).
COLUMN_X_OFFSET_M = {
    'd': 0.0035, 'e': 0.0035, 'f': 0.0035, 'g': 0.0035, 'h': 0.0035,
}

# Per-square (x, y) correction (m), measured via calibrate.py.
PER_SQUARE_OFFSET_M = {
    'a1': (-0.1 * INCH, 0.0 * INCH), 'a2': (-0.1 * INCH, 0.2 * INCH), 'a3': (0.0 * INCH, 0.25 * INCH), 'a4': (0.0 * INCH, 0.2 * INCH), 'a5': (0.15 * INCH, 0.1 * INCH), 'a6': (0.1 * INCH, 0.15 * INCH), 'a7': (0.15 * INCH, 0.2 * INCH), 'a8': (0.15 * INCH, 0.2 * INCH),  # no direct data — copied from a7
    'b1': (0.0 * INCH, 0.2 * INCH), 'b2': (0.07 * INCH, 0.12 * INCH), 'b3': (0.05 * INCH, 0.25 * INCH), 'b4': (0.1 * INCH, 0.2 * INCH), 'b5': (0.15 * INCH, 0.15 * INCH), 'b6': (0.1 * INCH, 0.2 * INCH), 'b7': (0.2 * INCH, 0.2 * INCH), 'b8': (0.2 * INCH, 0.2 * INCH),  # no direct data — copied from b7
    'c1': (0.07 * INCH, 0.2 * INCH), 'c2': (0.05 * INCH, 0.22 * INCH), 'c3': (0.1 * INCH, 0.15 * INCH), 'c4': (0.15 * INCH, 0.15 * INCH), 'c5': (0.16 * INCH, 0.0 * INCH), 'c6': (0.22 * INCH, 0.03 * INCH), 'c7': (0.28 * INCH, 0.07 * INCH), 'c8': (0.3 * INCH, 0.1 * INCH),
    'd1': (0.0 * INCH, 0.1 * INCH), 'd2': (0.1 * INCH, 0.15 * INCH), 'd3': (0.15 * INCH, 0.15 * INCH), 'd4': (0.15 * INCH, 0.15 * INCH), 'd5': (0.2 * INCH, 0.1 * INCH), 'd6': (0.2 * INCH, 0.0 * INCH), 'd7': (0.18 * INCH, 0.05 * INCH), 'd8': (0.3 * INCH, 0.05 * INCH),
    'e1': (0.13 * INCH, 0.05 * INCH), 'e2': (0.15 * INCH, 0.1 * INCH), 'e3': (0.18 * INCH, 0.2 * INCH), 'e4': (0.17 * INCH, 0.1 * INCH), 'e5': (0.25 * INCH, 0.1 * INCH), 'e6': (0.2 * INCH, 0.1 * INCH), 'e7': (0.3 * INCH, 0.05 * INCH), 'e8': (0.32 * INCH, 0.1 * INCH),
    'f1': (0.25 * INCH, 0.07 * INCH), 'f2': (0.22 * INCH, 0.08 * INCH), 'f3': (0.3 * INCH, 0.1 * INCH), 'f4': (0.26 * INCH, 0.17 * INCH), 'f5': (0.32 * INCH, 0.07 * INCH), 'f6': (0.35 * INCH, 0.1 * INCH), 'f7': (0.33 * INCH, 0.05 * INCH), 'f8': (0.4 * INCH, 0.02 * INCH),
    'g1': (0.33 * INCH, 0.05 * INCH), 'g2': (0.32 * INCH, 0.1 * INCH), 'g3': (0.31 * INCH, 0.1 * INCH), 'g4': (0.36 * INCH, 0.1 * INCH), 'g5': (0.4 * INCH, 0.0 * INCH), 'g6': (0.4 * INCH, 0.0 * INCH), 'g7': (0.45 * INCH, 0.02 * INCH), 'g8': (0.3 * INCH, 0.1 * INCH),
    'h1': (0.42 * INCH, 0.0 * INCH), 'h2': (0.4 * INCH, 0.07 * INCH), 'h3': (0.45 * INCH, 0.05 * INCH), 'h4': (0.45 * INCH, 0.0 * INCH), 'h5': (0.45 * INCH, 0.0 * INCH), 'h6': (0.45 * INCH, 0.0 * INCH), 'h7': (0.45 * INCH, 0.0 * INCH), 'h8': (0.45 * INCH, 0.0 * INCH),  # no direct data — copied from h7
}


# Builds the 64-square coordinate table by accumulating square widths/heights from the a1 outer corner (origin).
def _generate_square_coords():
    columns = "abcdefgh"
    rows = "12345678"

    # Column-center x positions: walk the board edge, placing each center half a square width in.
    x_centers = {}
    edge = 0.0
    for col in columns:
        width = COLUMN_WIDTHS_M[col]
        x_centers[col] = edge + width / 2.0 + COLUMN_X_OFFSET_M.get(col, 0.0)
        edge += width

    # Row-center y positions: same accumulation with the uniform row height.
    y_centers = {}
    edge = 0.0
    for row in rows:
        y_centers[row] = edge + ROW_HEIGHT_M / 2.0
        edge += ROW_HEIGHT_M

    return {f"{col}{row}": (x_centers[col], y_centers[row]) for col in columns for row in rows}


SQUARE_COORDS_M = _generate_square_coords()


def make_pose(x, y, z, qx, qy, qz, qw) -> Pose:
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    p.orientation.x = float(qx)
    p.orientation.y = float(qy)
    p.orientation.z = float(qz)
    p.orientation.w = float(qw)
    return p


def square_center_in_world(square: str) -> Pose:
    sq = square.lower()
    if sq not in SQUARE_COORDS_M:
        raise ValueError(f"Unknown square: {sq}")
    x_rel, y_rel = SQUARE_COORDS_M[sq]
    x_rotated = x_rel * math.cos(BOARD_ROTATION_RAD) - y_rel * math.sin(BOARD_ROTATION_RAD)
    y_rotated = x_rel * math.sin(BOARD_ROTATION_RAD) + y_rel * math.cos(BOARD_ROTATION_RAD)
    bx, by, bz = BOARD_ORIGIN
    world_x = bx + x_rotated
    world_y = by + y_rotated
    dx, dy = PER_SQUARE_OFFSET_M.get(sq, (0.0, 0.0))
    return make_pose(world_x + dx, world_y + dy, bz, *TOP_DOWN)


def compute_board_center():
    xs = [v[0] for v in SQUARE_COORDS_M.values()]
    ys = [v[1] for v in SQUARE_COORDS_M.values()]
    center_x = (min(xs) + max(xs)) / 2.0
    center_y = (min(ys) + max(ys)) / 2.0
    bx, by, bz = BOARD_ORIGIN
    return (bx + center_x, by + center_y, bz)