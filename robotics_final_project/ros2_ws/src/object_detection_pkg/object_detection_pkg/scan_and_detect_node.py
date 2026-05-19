import os
import json
import time
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from custom_interface.srv import GetBoardState


class Scan_And_Detect(Node):

    def __init__(self):
        super().__init__('scan_and_detect_node')
        self.image_subscriber = self.create_subscription(Image, 'processed_camera_feed', self.image_callback, 10)
        self.publisher_ = self.create_publisher(Image, 'live_detection_feed', 10)
        self.srv = self.create_service(GetBoardState, 'get_board_state', self.get_board_state_callback)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.setup_cv()
        self.setup_yolo()
        self.get_logger().info('Scan and Detect Node is ready.')

    def setup_cv(self):
        self.bridge = CvBridge()
        self.latest_image = None
        self.annotated_image = None

    def setup_yolo(self):
        pkg_share = get_package_share_directory('object_detection_pkg')
        model_path = os.path.join(pkg_share, 'trained_models', 'best.pt')
        self.model = YOLO(model_path)

    def capture_frame(self, data):
        try:
            return self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS Image to OpenCV image: {e}")
            return None

    def convert_cv2_to_ros(self, frame):
        return self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

    def run_yolo(self, frame):
        yolo_input = np.ascontiguousarray(frame)
        yolo_input = yolo_input.copy()
        results = self.model(yolo_input, verbose=False)
        if len(results) == 0:
            return None
        return results[0]

    def detect_board_state(self, frame):
        self.get_logger().info("Starting YOLO detection...")
        start_time = time.time()

        result = self.run_yolo(frame)

        detection_time = time.time() - start_time
        self.get_logger().info(f"YOLO detection finished in {detection_time:.4f} seconds.")

        if result is None:
            self.get_logger().warning("No detections from YOLO model.")
            self.annotated_image = None
            return {}

        self.annotated_image = result.plot()

        board_dict = {}
        file_index = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
        rank_index = ['8', '7', '6', '5', '4', '3', '2', '1']

        h, w, _ = frame.shape
        square_x_length = w / 8
        square_y_length = h / 8

        for box in result.boxes:
            cx = int(box.xywh[0][0])
            cy = int(box.xywh[0][1])
            cls = int(box.cls[0])
            name = self.model.names[cls]
            file = int(cx // square_x_length)
            rank = int(cy // square_y_length)
            fen_square = file_index[file] + rank_index[rank]
            board_dict[fen_square] = name

        return board_dict

    def image_callback(self, data):
        self.latest_image = self.capture_frame(data)

    def timer_callback(self):
        if self.latest_image is None:
            return

        result = self.run_yolo(self.latest_image)
        if result is None:
            return

        self.annotated_image = result.plot()

        try:
            self.publisher_.publish(self.convert_cv2_to_ros(self.annotated_image))
        except Exception as e:
            self.get_logger().error(f"Failed to publish live detection feed: {e}")

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
    scan_and_detect_node = Scan_And_Detect()
    try:
        rclpy.spin(scan_and_detect_node)
    except KeyboardInterrupt:
        pass
    scan_and_detect_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()