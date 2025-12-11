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
        
        # Create a subscriber for Image messages on the 'processed_video_feed' topic
        self.image_subscriber = self.create_subscription(Image, 'processed_video_feed', self.image_callback, 10)

        # Create a service server 
        self.srv = self.create_service(GetBoardState, "get_board_state", self.get_board_state_callback)
        self.get_logger().info('ScanBoard node is ready. Service server listening')

        # FIXED HERE: Create a timer to periodically display the image using cv2.imshow
        self.display_timer = self.create_timer(0.1, self.display_timer_callback)
        
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
    def display_timer_callback(self):
        """Periodically displays the latest images using cv2.imshow() and processes key events."""
        # Display the raw homographed image (the input to detection)
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