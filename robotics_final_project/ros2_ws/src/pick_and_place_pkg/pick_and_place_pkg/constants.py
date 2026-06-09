from geometry_msgs.msg import Pose

INCH = 0.0254
HOVER_ABOVE_BOARD_M = 2.0 * INCH
CENTER_HOVER_ABOVE_BOARD_M = 8.0 * INCH
PLACE_Z_OFFSET = 0.0
BOARD_Z = 8.2 * INCH
BOARD_ORIGIN = (-6.35 * INCH, 7.4 * INCH, BOARD_Z)
TOP_DOWN = (-0.7071, 0.7071, 0.0, 0.0)
CENTER_HOVER_Z = 0.4166

import math
BOARD_ROTATION_RAD = math.radians(-1.6)
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
    return make_pose(bx + x_rotated, by + y_rotated, bz, *TOP_DOWN)


def compute_board_center():
    xs = [v[0] for v in SQUARE_COORDS_M.values()]
    ys = [v[1] for v in SQUARE_COORDS_M.values()]
    center_x = (min(xs) + max(xs)) / 2.0
    center_y = (min(ys) + max(ys)) / 2.0
    bx, by, bz = BOARD_ORIGIN
    return (bx + center_x, by + center_y, bz)