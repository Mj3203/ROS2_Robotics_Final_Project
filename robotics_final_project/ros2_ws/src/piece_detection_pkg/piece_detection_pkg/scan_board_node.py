import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # ROS2 message types
from cv_bridge import CvBridge # Bridge to convert between ROS and OpenCV images
import cv2 # OpenCV for image processing
import numpy as np
import time # FIXED HERE (for logging/timing the detection)
 
from ultralytics import YOLO # YOLO model for object detection
 
from custom_interface.srv import GetBoardState # Service definition
 
from ament_index_python.packages import get_package_share_directory
 
import os
import json
 
class ScanBoard(Node):
    def __init__(self):
        super().__init__('scan_board_node')
 
        # Initialize CvBridge
        self.bridge = CvBridge()
 
        # Placeholder for the latest image
        self.latest_image = None
        
        # FIXED HERE: Placeholder for the latest image with YOLO annotations
        self.annotated_image = None
 
        # NEW: Placeholder for the last board scan snapshot (only updated on service call)
        self.scan_result_image = None
 
        # CHANGED: Updated topic name from 'processed_video_feed' to 'processed_camera_feed'
        # OLD: self.image_subscriber = self.create_subscription(Image, 'processed_video_feed', self.image_callback, 10)
        self.image_subscriber = self.create_subscription(Image, 'processed_camera_feed', self.image_callback, 10)
 
        # Create a service server
        self.srv = self.create_service(GetBoardState, "get_board_state", self.get_board_state_callback)
        self.get_logger().info('ScanBoard node is ready. Service server listening')
 
        # FIXED HERE: Create a timer to periodically display the image using cv2.imshow
        # NEW: Also runs continuous YOLO detection for live feed
        self.display_timer = self.create_timer(0.1, self.display_timer_callback)
 
        # NEW: Publisher for continuous live YOLO detection feed
        self.live_detection_pub = self.create_publisher(Image, 'live_detection_feed', 10)
 
        # NEW: Publisher for board scan snapshot (only published on service call)
        self.scan_result_pub = self.create_publisher(Image, 'scan_result_feed', 10)
 
        # Load YOLO model
        pkg_share = get_package_share_directory('piece_detection_pkg')
        model_path = os.path.join(pkg_share, 'trained_models', 'best.pt')
        self.model = YOLO(model_path)
 
    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            self.latest_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting ROS Image to OpenCV image: {e}")
            self.latest_image = None
            return
 
    # FIXED HERE: Timer callback for OpenCV display
    # NEW: Also runs continuous YOLO on latest image and publishes to /live_detection_feed
    def display_timer_callback(self):
        """Periodically displays the latest images using cv2.imshow() and processes key events."""
 
        # NEW: Run YOLO continuously on the latest image for the live detection feed
        if self.latest_image is not None:
            contiguous_image = np.ascontiguousarray(self.latest_image)
            input_image = contiguous_image.copy()
            results = self.model(input_image, verbose=False)
 
            if len(results) > 0:
                # NEW: Store annotated image for display
                self.annotated_image = results[0].plot()
 
                # NEW: Publish live detection feed to /live_detection_feed
                try:
                    live_ros_image = self.bridge.cv2_to_imgmsg(self.annotated_image, encoding='bgr8')
                    self.live_detection_pub.publish(live_ros_image)
                except Exception as e:
                    self.get_logger().error(f"Failed to publish live detection feed: {e}")
 
        # Display the raw warped image (the input to detection)
        # OLD: cv2.imshow("Homographed Video Feed (Input)", self.latest_image)
        if self.latest_image is not None:
            cv2.imshow("Homographed Video Feed (Input)", self.latest_image)
 
        # Display the YOLO-annotated result, if detection has run
        if self.annotated_image is not None:
            cv2.imshow("YOLO Detection Result", self.annotated_image)
 
        # This is essential for cv2.imshow to update the windows and process key presses
        key = cv2.waitKey(1)
        if key == ord('q'):
            # Close all windows if 'q' is pressed, allowing the ROS node to continue running
            cv2.destroyAllWindows()
            self.get_logger().info("Closed OpenCV visualization windows.")
 
    def detect_board_state(self, homographed_image):
        self.get_logger().info("Starting YOLO detection...") # FIXED HERE (added logging)
        start_time = time.time() # FIXED HERE (for timing)
        
        # 1. Ensure the array is memory contiguous (necessary for PyTorch/CUDA)
        contiguous_image = np.ascontiguousarray(homographed_image)
        
        # 2. Force a deep copy to create a brand new, clean NumPy object
        input_image = contiguous_image.copy()
 
        # 3. Pass the new, cleaned array to the model (original call retained)
        results = self.model(input_image)
 
        detection_time = time.time() - start_time # FIXED HERE (for timing)
        self.get_logger().info(f"YOLO detection finished in {detection_time:.4f} seconds.") # FIXED HERE (added logging)
 
        if len(results) == 0:
            self.get_logger().warning("No detections from YOLO model.")
            self.annotated_image = None # FIXED HERE (reset annotated image)
            return {}
        
        res = results[0]
 
        # FIXED HERE: Store the annotated image result for the display timer to show
        self.annotated_image = res.plot()
 
        # NEW: Store snapshot of this specific scan result for /scan_result_feed
        self.scan_result_image = self.annotated_image.copy()
 
        board_dict = {}
 
        file_index = ['a','b','c','d','e','f','g','h']
        rank_index = ['8','7','6','5','4','3','2','1']
 
        h, w, _ = homographed_image.shape
        square_x_length = w/8
        square_y_length = h/8
        
        for box in res.boxes:
            cx = int(box.xywh[0][0])
            cy = int(box.xywh[0][1])
 
            cls = int(box.cls[0])
            name = self.model.names[cls]
 
            file = int(cx // square_x_length)
            rank = int(cy // square_y_length)
 
            fen_square = file_index[file] + rank_index[rank]
            board_dict[fen_square] = name
 
        return board_dict
        
    def get_board_state_callback(self, request, response):
        self.get_logger().info(f"Received request for board state: {request.request_message}")
        
        if self.latest_image is None:
            self.get_logger().error("No image available for processing.")
            response.board_json = "{}"
            return response
        
        # This function updates self.annotated_image which is displayed by the timer
        board_dict = self.detect_board_state(self.latest_image)
 
        # NEW: Publish the scan result snapshot to /scan_result_feed
        if self.scan_result_image is not None:
            try:
                scan_ros_image = self.bridge.cv2_to_imgmsg(self.scan_result_image, encoding='bgr8')
                self.scan_result_pub.publish(scan_ros_image)
                self.get_logger().info("Published board scan snapshot to /scan_result_feed")
            except Exception as e:
                self.get_logger().error(f"Failed to publish scan result feed: {e}")
        
        response.board_json = json.dumps(board_dict)
        return response
 
    def destroy_node(self):
        # FIXED HERE: Ensure all OpenCV windows are closed when the node is destroyed
        cv2.destroyAllWindows()
        super().destroy_node()
 
def main(args=None):
    rclpy.init(args=args)
    scan_board_node = ScanBoard()
 
    try:
        rclpy.spin(scan_board_node)
    except KeyboardInterrupt:
        pass
    
    # Destroy the node explicitly
    scan_board_node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
