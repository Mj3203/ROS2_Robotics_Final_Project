import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # ROS2 message types
from cv_bridge import CvBridge # Bridge to convert between ROS and OpenCV images
import cv2 # OpenCV for image processing
import numpy as np

# Function to get locate "origin" for detected aruco markers
def get_chessboard_corner_obj_points(L):
    obj_points_TR = np.array([
        [-L, 0, 0],
        [0, 0, 0],
        [0, -L, 0],
        [-L, -L, 0]
    ], dtype=np.float32)
    
    obj_points_BR = np.array([
        [-L, L, 0],
        [0, L, 0],
        [0, 0, 0], 
        [-L, 0, 0]
    ], dtype=np.float32)
    
    obj_points_TL = np.array([
        [0, 0, 0],
        [L, 0, 0],
        [L, -L, 0],
        [0, -L, 0]
    ], dtype=np.float32)
    
    obj_points_BL = np.array([
        [0, L, 0],
        [L, L, 0],
        [L, 0, 0],
        [0, 0, 0]  
    ], dtype=np.float32)

    chessboard_corner_obj_points = [obj_points_TR, obj_points_BR, obj_points_TL, obj_points_BL]
    return chessboard_corner_obj_points

# Function that projects "origin" of aruco marker to pixel coordinates in the image plane
def get_src_coords(rvec, tvec, camera_intrinsics_matrix, dist_coeffs):
    origin = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
    pixel_coords, jacobian = cv2.projectPoints(
        origin,                      # The 3D point (the origin)
        rvec,                        # The marker's rotation
        tvec,                        # The marker's translation
        camera_intrinsics_matrix,    # Camera intrinsic matrix
        dist_coeffs                  # Camera distortion coefficients
    )
    x_src_coord = int(pixel_coords[0][0][0])
    y_src_coord = int(pixel_coords[0][0][1])
    return x_src_coord, y_src_coord

# Function to rearrange detected aruco tags in order of their IDs numerically
def rearrange_order_of_detected_tags(corners, ids):
    reordered_corners = []
    reordered_ids = []

    for i in range(4):
        for j in range(4):
            if i == ids[j]:
                reordered_corners.append(corners[j])
                reordered_ids.append(ids[j])

    reordered_corners = tuple(reordered_corners)
    reordered_ids = np.array(reordered_ids, dtype=ids.dtype)
    return reordered_corners, reordered_ids

# Intrinsic matrix: focal lengths (fx, fy) and principal point (cx, cy)
camera_intrinsics_matrix = np.array([
    [1423.6, 0.0,    983.92],
    [0.0,    1403.8, 572.46],
    [0.0,    0.0,    1.0   ]
], dtype=np.float64)

# Distortion coefficients: radial (k1, k2, k3) and tangential (p1, p2)
dist_coeffs = np.array([
    [0.065642, -0.43046, 0.0054587, 0.00043214, 0.52854]
], dtype=np.float64)

# Predefined aruco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)

class Homography_Transform(Node):
    def __init__(self):
        # Initialize the Node with the name 'homography_transform_node'
        super().__init__('homography_transform_node')

        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Create a subscriber for Image messages on the 'raw_camera_feed' topic
        self.subscription = self.create_subscription(Image, 'raw_camera_feed', self.image_callback, 10)

        # Create a publisher for Image messages on the 'processed_camera_feed' topic
        self.publisher_ = self.create_publisher(Image, 'processed_camera_feed', 10)

        # Initialize the ArUco detector
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, self.parameters)

        self.get_logger().info("Homography Transform Node Ready. Subscribing to /raw_camera_feed.")

    def image_callback(self, data):
        # Convert ROS Image message to OpenCV image
        try:
            raw_camera_feed = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS Image to OpenCV image: {e}")
            return

        # Perform homography on the raw image
        processed_image = self.perform_homography(raw_camera_feed)

        if processed_image is not None:
            cv2.imshow("Homography Output", processed_image)
            cv2.waitKey(1)

            # Convert the processed OpenCV image back to a ROS Image message
            # OpenCV uses BGR format by default
            processed_ros_image = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')

            # Publish the processed ROS Image message
            self.publisher_.publish(processed_ros_image)
            self.get_logger().info('Publishing processed camera feed')

    def perform_homography(self, image):
        # Detect ArUco markers in the image
        corners, ids, rejected = self.detector.detectMarkers(image)

        # Check if any ArUco markers were detected
        if ids is None or corners is None:
            self.get_logger().error(f"No ArUco markers detected. IDs: {ids}, Corners: {corners}")
            return None
        elif len(ids) != 4 or len(corners) != 4:
            self.get_logger().error(f"Expected 4 ArUco markers, detected {len(ids)}. IDs found: {ids.flatten().tolist()}")
            return None

        # Get object points for each chessboard corner marker
        aruco_marker_size = 0.040 # meters
        obj_points = get_chessboard_corner_obj_points(aruco_marker_size)

        # Reorder detected markers based on their IDs numerically
        reordered_corners, reordered_ids = rearrange_order_of_detected_tags(corners, ids)

        # Check reordered markers
        if len(reordered_ids) != 4 or len(reordered_corners) != 4:
            self.get_logger().error(f"Failed to reorder markers. Expected 4, got {len(reordered_ids)}. IDs: {reordered_ids.flatten().tolist()}")
            return None

        # Solve PnP for each marker to get its origin in pixel coordinates
        src_coords = []
        for i in range(4):
            success, rvec, tvec = cv2.solvePnP(obj_points[i], reordered_corners[i], camera_intrinsics_matrix, dist_coeffs)
            if success:
                # Project the origin of each ArUco marker to pixel coordinates
                x_src_coord, y_src_coord = get_src_coords(rvec, tvec, camera_intrinsics_matrix, dist_coeffs)
                src_coords.append([x_src_coord, y_src_coord])

        src_coords = np.array(src_coords, dtype=np.float32)

        # Check if all 4 source coordinates were obtained
        if len(src_coords) != 4:
            self.get_logger().error(f"Expected 4 source coordinates, got {len(src_coords)}. solvePnP may have failed for one or more markers.")
            return None

        # Display size parameters
        display_height_pixels = 800
        board_height = 0.37465 # meters
        board_width = 0.3467125 # meters
        scale_factor = display_height_pixels / board_height
        display_width_pixels = int(board_width * scale_factor)

        # Destination coordinates for homography (top-down view of the board)
        destination_coords = np.array([
            [0.0, display_height_pixels],
            [0.0, 0.0],
            [display_width_pixels, display_height_pixels],
            [display_width_pixels, 0.0],
        ], dtype=np.float32)

        # Compute the homography matrix
        h, mask = cv2.findHomography(src_coords, destination_coords)

        # Check if homography matrix was computed successfully
        if h is None:
            self.get_logger().error(f"Homography matrix could not be computed. src_coords: {src_coords}")
            return None

        # Warp the image to a top-down perspective
        warped_image = cv2.warpPerspective(image, h, (display_width_pixels, display_height_pixels))

        return warped_image

    def destroy_node(self):
        # Shut down the node cleanly
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    homography_transform_node = Homography_Transform()

    # Node runs indefinitely until interrupted
    # Ctrl+C stops the node and shuts down rather than crashes
    try:
        rclpy.spin(homography_transform_node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    homography_transform_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()