import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # ROS2 message types
from std_msgs.msg import String # ROS2 message types
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
def get_src_coords(rvec, tvec, camera_matrix, dist_coeffs):
    origin = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
    pixel_coords, jacobian = cv2.projectPoints(
        origin,  # 1. The 3D point (the origin)
        rvec,  # 2. The marker's rotation
        tvec,  # 3. The marker's translation
        camera_matrix,  # 4. Camera intrinsic matrix
        dist_coeffs  # 5. Camera distortion coefficients
    )
    x_src_coord = int(pixel_coords[0][0][0])
    y_src_coord = int(pixel_coords[0][0][1])
    return  x_src_coord, y_src_coord

# Function to rearrange detected aruco tags in order of their IDs numerically
def rearrange_order_of_detected_tags(corners, ids):
    reordered_corners = []
    reordered_ids = []

    for i in range(4):
        for j in range(4):
            if i == ids[j]:
                #print(f"looking for id {i}, found tag {ids[j]} at index {j}")
                reordered_corners.append(corners[j])
                reordered_ids.append(ids[j])

    reordered_corners = tuple(reordered_corners)
    reordered_ids = np.array(reordered_ids, dtype=ids.dtype)
    return reordered_corners, reordered_ids

# Camera calibration parameters
camera_matrix = np.array([
    [1423.6, 0.0, 983.92],
    [0.0, 1403.8, 572.46],
    [0.0, 0.0, 1.0]
    ], dtype=np.float64)    

dist_coeffs = np.array([
    [0.065642, -0.43046, 0.0054587, 0.00043214, 0.52854]
    ], dtype=np.float64)

# Predefined aruco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)

class HomographyProcessor(Node):
    def __init__(self):
        # Initialize the Node with the name 'homography_processor_node'
        super().__init__('homography_processor_node') 
        
        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Create a subscriber for Image messages on the 'video_feed' topic
        self.subscription = self.create_subscription(Image,'video_feed', self.image_callback, 10)

        # Create a publisher for Image messages on the 'processed_video_feed' topic
        self.publisher_ = self.create_publisher(Image, 'processed_video_feed', 10)

        self.get_logger().info("Homography Processor Node Ready. Subscribing to /video_feed.")

        # Detecting Aruco markers
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, self.parameters)

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting ROS Image to OpenCV image: {e}")
            return
        
        processed_image = self.perform_homography(raw_image)

        if processed_image is not None:
            cv2.imshow("Homography Output", processed_image)
            cv2.waitKey(1)
            
            # Convert the processed OpenCV image back to ROS Image message
            processed_ros_image = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            
            # Publish the processed ROS Image message
            self.publisher_.publish(processed_ros_image)
            self.get_logger().info('Published processed video frame')

    def perform_homography(self, image):
        corners, ids, rejected = self.detector.detectMarkers(image)
        
        # Debugging logs for detection
        if ids is None or corners is None:
            self.get_logger().warning("No Aruco markers detected.")
            return None
        elif len(ids) != 4 or len(corners) != 4:
            self.get_logger().warning("4 Aruco markers were not detected.")
            return None
        
        aruco_marker_size = 0.020 #mm
        obj_points = get_chessboard_corner_obj_points(aruco_marker_size)

        # Reorder detected markers based on their IDs numerically
        reordered_corners, reordered_ids = rearrange_order_of_detected_tags(corners, ids)
        
        # Debugging logs for detection
        if len(reordered_ids) != 4 or len(reordered_corners) != 4:
            self.get_logger().warning("4 Aruco marker were not detected.")
            return None
        
        src_coords = []
        for i in range(4):
            success, rvec, tvec = cv2.solvePnP(obj_points[i], reordered_corners[i], camera_matrix, dist_coeffs)
            if success:
                # Returns the "origin" of each aruco marker in pixels and stores it
                x_src_coord, y_src_coord = get_src_coords(rvec, tvec, camera_matrix, dist_coeffs)
                src_coords.append([x_src_coord, y_src_coord])
        
        src_coords = np.array(src_coords, dtype=np.float32)

        # Display size parameters
        display_height_pixels = 800
        board_height = 0.37465 # meters
        board_width = 0.3467125 # meters
        scale_factor = display_height_pixels / board_height
        display_width_pixels = int(board_width * scale_factor)

        # Destination coordinates for homography
        destination_coords = (
            [0.0, display_height_pixels],
            [0.0, 0.0],
            [display_width_pixels, display_height_pixels],
            [display_width_pixels, 0.0],
        )

        destination_coords = np.array(destination_coords, dtype=np.float32)

        # Perform homography if all 4 source coordinates are obtained
        if len(src_coords) != 4:
            self.get_logger().warning("4 source coordinates were not detected.")
            return None
        
        h, mask = cv2.findHomography(src_coords, destination_coords)

        # Debugging logs for detection
        if h is None:
            self.get_logger().error("Homography matrix could not be computed.")
            return None
        
        warped_image = cv2.warpPerspective(image, h, (display_width_pixels, display_height_pixels))

        return warped_image
    
    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    homography_processor_node = HomographyProcessor()

    try:
        rclpy.spin(homography_processor_node)
    except KeyboardInterrupt:
        pass

    homography_processor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()