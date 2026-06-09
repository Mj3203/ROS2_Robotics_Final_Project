import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Returns the 4 corner positions of each ArUco marker in real world 3D coordinates
def get_chessboard_corner_obj_points(L):
    obj_points_TR = np.array([[-L, 0, 0], [0, 0, 0], [0, -L, 0], [-L, -L, 0]], dtype=np.float32)
    obj_points_BR = np.array([[-L, L, 0], [0, L, 0], [0, 0, 0], [-L, 0, 0]], dtype=np.float32)
    obj_points_TL = np.array([[0, 0, 0], [L, 0, 0], [L, -L, 0], [0, -L, 0]], dtype=np.float32)
    obj_points_BL = np.array([[0, L, 0], [L, L, 0], [L, 0, 0], [0, 0, 0]], dtype=np.float32)
    return [obj_points_TR, obj_points_BR, obj_points_TL, obj_points_BL]

# Projects the 3D origin of an ArUco marker into 2D pixel coordinates in the image
def get_src_coords(rvec, tvec, camera_intrinsics_matrix, dist_coeffs):
    origin = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
    pixel_coords, jacobian = cv2.projectPoints(origin, rvec, tvec, camera_intrinsics_matrix, dist_coeffs)
    return int(pixel_coords[0][0][0]), int(pixel_coords[0][0][1])

# Sorts detected ArUco markers numerically by ID so they consistently map to the correct board corners
def rearrange_order_of_detected_tags(corners, ids):
    reordered_corners = []
    reordered_ids = []
    for i in range(4):
        for j in range(4):
            if i == ids[j]:
                reordered_corners.append(corners[j])
                reordered_ids.append(ids[j])
    return tuple(reordered_corners), np.array(reordered_ids, dtype=ids.dtype)


class Homography_Transform(Node):

    # Initializes the node, creates the camera feed subscription and publisher, and runs all setup steps.
    def __init__(self):
        super().__init__('homography_transform_node')
        self.subscription = self.create_subscription(Image, 'raw_camera_feed', self.image_callback, 10)
        self.publisher_ = self.create_publisher(Image, 'processed_camera_feed', 10)
        self.setup_homography()
        self.setup_camera()
        self.get_logger().info("Homography Transform Node ready.")

    # Initializes the ArUco dictionary, camera intrinsics, and distortion coefficients
    def setup_homography(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.camera_intrinsics_matrix = np.array([
            [1423.6, 0.0,    983.92],
            [0.0,    1403.8, 572.46],
            [0.0,    0.0,    1.0   ]
        ], dtype=np.float64)
        self.dist_coeffs = np.array([
            [0.065642, -0.43046, 0.0054587, 0.00043214, 0.52854]
        ], dtype=np.float64)

    # Initializes the CvBridge and ArUco detector
    def setup_camera(self):
        self.bridge = CvBridge()
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

    # Converts incoming ROS2 Image message to an OpenCV frame
    def capture_frame(self, data):
        try:
            return self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS Image to OpenCV image: {e}")
            return None

    # Converts an OpenCV frame back to a ROS2 Image message
    def convert_cv2_to_ros(self, frame):
        return self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

    # Detects ArUco markers and warps the frame into a flat top-down view of the board
    def perform_homography(self, frame):
        corners, ids, rejected = self.detector.detectMarkers(frame)

        # Error: No ArUco markers are visible at all
        if ids is None or corners is None:
            self.get_logger().error(f"No ArUco markers detected. IDs: {ids}, Corners: {corners}")
            return None
        # Error: Num of detected markers is less than 4
        elif len(ids) != 4 or len(corners) != 4:
            self.get_logger().error(f"Expected 4 ArUco markers, detected {len(ids)}. IDs found: {ids.flatten().tolist()}")
            return None

        aruco_marker_size = 0.040
        obj_points = get_chessboard_corner_obj_points(aruco_marker_size)
        reordered_corners, reordered_ids = rearrange_order_of_detected_tags(corners, ids)

        src_coords = []
        for i in range(4):
            success, rvec, tvec = cv2.solvePnP(obj_points[i], reordered_corners[i], self.camera_intrinsics_matrix, self.dist_coeffs)
            if success:
                x, y = get_src_coords(rvec, tvec, self.camera_intrinsics_matrix, self.dist_coeffs)
                src_coords.append([x, y])

        src_coords = np.array(src_coords, dtype=np.float32)

        # Error: solvePnP failed to calculate a pose for all 4 markers
        if len(src_coords) != 4:
            self.get_logger().error(f"Expected 4 source coordinates, got {len(src_coords)}.")
            return None

        display_height_pixels = 800
        board_height = 0.37465
        board_width = 0.3467125
        display_width_pixels = int(board_width * (display_height_pixels / board_height))

        destination_coords = np.array([
            [0.0, display_height_pixels],
            [0.0, 0.0],
            [display_width_pixels, display_height_pixels],
            [display_width_pixels, 0.0],
        ], dtype=np.float32)

        h, mask = cv2.findHomography(src_coords, destination_coords)

        # Error: Homography matrix could not be computed from the src coords
        if h is None:
            self.get_logger().error(f"Homography matrix could not be computed. src_coords: {src_coords}")
            return None

        return cv2.warpPerspective(frame, h, (display_width_pixels, display_height_pixels))

    # Receives raw camera frame, applies homography, and publishes the processed result
    def image_callback(self, data):
        frame = self.capture_frame(data)
        if frame is None:
            return

        processed_frame = self.perform_homography(frame)
        if processed_frame is None:
            return

        self.publisher_.publish(self.convert_cv2_to_ros(processed_frame))
        self.get_logger().info('Publishing processed camera feed')

    # Cleans up and shuts the node down.
    def destroy_node(self):
        super().destroy_node()


# Initializes ROS, spins the node until interrupted, then shuts everything down.
def main(args=None):
    rclpy.init(args=args)
    homography_transform_node = Homography_Transform()
    try:
        rclpy.spin(homography_transform_node)
    except KeyboardInterrupt:
        pass
    homography_transform_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()