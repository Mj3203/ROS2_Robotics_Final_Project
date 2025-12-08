import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # ROS2 message types
from cv_bridge import CvBridge # Bridge to convert between ROS and OpenCV images
import cv2 # OpenCV for image processing
import numpy as np

from ultralytics import YOLO # YOLO model for object detection

from custom_interface.srv import GetBoardState # Service definition

from ament_index_python.packages import get_package_share_directory

import os
import json

class PieceDetection(Node):
    def __init__(self):
        super().__init__('get_board_state')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Placeholder for the latest image
        self.latest_image = None

        # Create a subscriber for Image messages on the 'processed_video_feed' topic
        self.image_subscriber = self.create_subscription(Image, 'processed_video_feed', self.image_callback, 10)

        # Create a service server 
        self.srv = self.create_service(GetBoardState, "get_board_state", self.get_board_state_callback)
        self.get_logger().info('PieceDetection node is ready. Service server listening')

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
        
    def detect_board_state(self, homographed_image):
        # 1. Ensure the array is memory contiguous (necessary for PyTorch/CUDA)
        contiguous_image = np.ascontiguousarray(homographed_image) 
        
        # 2. Force a deep copy to create a brand new, clean NumPy object
        input_image = contiguous_image.copy()

        # 3. Pass the new, cleaned array to the model
        results = self.model(input_image)

        if len(results) == 0:
            self.get_logger().warning("No detections from YOLO model.")
            return {}
        
        res = results[0]

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
        
        board_dict = self.detect_board_state(self.latest_image)
        response.board_json = json.dumps(board_dict)
        return response

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    piece_detection = PieceDetection()

    try:
        rclpy.spin(piece_detection)
    except KeyboardInterrupt:
        pass
    
    # Destroy the node explicitly
    piece_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()