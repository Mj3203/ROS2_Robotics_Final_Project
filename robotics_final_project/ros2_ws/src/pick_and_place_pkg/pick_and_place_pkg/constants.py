from geometry_msgs.msg import Pose

# All physical measurements and coordinates for the chess board and arm positions.
# These values define where every square is in the real world relative to the robot.
# If the board is moved or the arm is repositioned, these values need to be recalibrated.
#
# Units are in meters unless otherwise noted.
# INCH is a conversion factor used to define constants in inches for readability
# since the board was measured in inches.
#
# BOARD_ORIGIN is the physical position of the board's corner (a1 square) in the
# robot's coordinate frame. All square coordinates are relative to this origin.
#
# TOP_DOWN is the fixed orientation quaternion that keeps the arm pointing straight
# down toward the board during all pick and place operations.
#
# SQUARE_COORDS_M maps each chess square (e.g. 'e4') to its physical (x, y) position
# in meters relative to BOARD_ORIGIN.
#
# Helper functions:
# - make_pose(x, y, z, qx, qy, qz, qw) : builds a ROS2 Pose object from coordinates
# - square_center_in_world(square) : converts a chess square like 'e4' into a real world Pose
# - compute_board_center() : calculates the physical center of the board, used by tune.py

SOLENOID_OFF_DELAY_DEFAULT = 0.2

INCH = 0.0254
HOVER_ABOVE_BOARD_M = 2.0 * INCH
CENTER_HOVER_ABOVE_BOARD_M = 8.0 * INCH
PLACE_Z_OFFSET = 0.0
BOARD_Z = 8.4 * INCH
BOARD_ORIGIN = (-6.55 * INCH, 7.4 * INCH, BOARD_Z)
TOP_DOWN = (-0.7071, 0.7071, 0.0, 0.0)

SQUARE_COORDS_M = {
    'a1': (0.020637, 0.020637), 'a2': (0.020637, 0.061912), 'a3': (0.020637, 0.103188), 'a4': (0.020637, 0.144462),
    'a5': (0.020637, 0.185737), 'a6': (0.020637, 0.227013), 'a7': (0.020637, 0.268287), 'a8': (0.020637, 0.309563),
    'b1': (0.061912, 0.020637), 'b2': (0.061912, 0.061912), 'b3': (0.061912, 0.103188), 'b4': (0.061912, 0.144462),
    'b5': (0.061912, 0.185737), 'b6': (0.061912, 0.227013), 'b7': (0.061912, 0.268287), 'b8': (0.061912, 0.309563),
    'c1': (0.103188, 0.020637), 'c2': (0.103188, 0.061912), 'c3': (0.103188, 0.103188), 'c4': (0.103188, 0.144462),
    'c5': (0.103188, 0.185737), 'c6': (0.103188, 0.227013), 'c7': (0.103188, 0.268287), 'c8': (0.103188, 0.309563),
    'd1': (0.144462, 0.020637), 'd2': (0.144462, 0.061912), 'd3': (0.144462, 0.103188), 'd4': (0.144462, 0.144462),
    'd5': (0.144462, 0.185737), 'd6': (0.144462, 0.227013), 'd7': (0.144462, 0.268287), 'd8': (0.144462, 0.309563),
    'e1': (0.185737, 0.020637), 'e2': (0.185737, 0.061912), 'e3': (0.185737, 0.103188), 'e4': (0.185737, 0.144462),
    'e5': (0.185737, 0.185737), 'e6': (0.185737, 0.227013), 'e7': (0.185737, 0.268287), 'e8': (0.185737, 0.309563),
    'f1': (0.227013, 0.020637), 'f2': (0.227013, 0.061912), 'f3': (0.227013, 0.103188), 'f4': (0.227013, 0.144462),
    'f5': (0.227013, 0.185737), 'f6': (0.227013, 0.227013), 'f7': (0.227013, 0.268287), 'f8': (0.227013, 0.309563),
    'g1': (0.268287, 0.020637), 'g2': (0.268287, 0.061912), 'g3': (0.268287, 0.103188), 'g4': (0.268287, 0.144462),
    'g5': (0.268287, 0.185737), 'g6': (0.268287, 0.227013), 'g7': (0.268287, 0.268287), 'g8': (0.268287, 0.309563),
    'h1': (0.309563, 0.020637), 'h2': (0.309563, 0.061912), 'h3': (0.309563, 0.103188), 'h4': (0.309563, 0.144462),
    'h5': (0.309563, 0.185737), 'h6': (0.309563, 0.227013), 'h7': (0.309563, 0.268287), 'h8': (0.309563, 0.309563)
}


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
    square = square.lower()
    if square not in SQUARE_COORDS_M:
        raise ValueError(f"Unknown square: {square}")
    x_relative, y_relative = SQUARE_COORDS_M[square]
    board_x, board_y, board_z = BOARD_ORIGIN
    return make_pose(board_x + x_relative, board_y + y_relative, board_z, *TOP_DOWN)


def compute_board_center():
    x_values = [coords[0] for coords in SQUARE_COORDS_M.values()]
    y_values = [coords[1] for coords in SQUARE_COORDS_M.values()]
    center_x = (min(x_values) + max(x_values)) / 2.0
    center_y = (min(y_values) + max(y_values)) / 2.0
    board_x, board_y, board_z = BOARD_ORIGIN
    return (board_x + center_x, board_y + center_y, board_z)