import rclpy
import json
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

DISPLAY_HEIGHT = 360
DISPLAY_WIDTH = 640
INFO_BAR_HEIGHT = 120


class DisplayOutput(Node):

    def __init__(self):
        super().__init__('display_output_node')
        self.raw_sub = self.create_subscription(Image, 'raw_camera_feed', self.raw_camera_feed_callback, 10)
        self.processed_sub = self.create_subscription(Image, 'processed_camera_feed', self.processed_camera_feed_callback, 10)
        self.live_detection_sub = self.create_subscription(Image, 'live_detection_feed', self.live_detection_feed_callback, 10)
        self.game_status_sub = self.create_subscription(String, 'game_status_feed', self.game_status_callback, 10)
        self.timer = self.create_timer(0.1, self.display_timer_callback)
        self.setup_display()
        self.get_logger().info('Display Output Node is ready.')

    def setup_display(self):
        self.bridge = CvBridge()
        self.raw_frame = None
        self.processed_frame = None
        self.live_detection_frame = None
        self.game_state = "WAITING"
        self.human_move = ""
        self.move_status = ""

    def raw_camera_feed_callback(self, data):
        try:
            self.raw_frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert raw camera feed: {e}")

    def processed_camera_feed_callback(self, data):
        try:
            self.processed_frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert processed camera feed: {e}")

    def live_detection_feed_callback(self, data):
        try:
            self.live_detection_frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert live detection feed: {e}")

    def game_status_callback(self, data):
        try:
            status = json.loads(data.data)
            self.game_state = status.get("game_state", "")
            self.human_move = status.get("human_move", "")
            self.move_status = status.get("move_status", "")
        except Exception as e:
            self.get_logger().error(f"Failed to parse game status: {e}")

    def make_panel(self, frame, label):
        h, w = frame.shape[:2]
        scale = DISPLAY_HEIGHT / h
        new_width = int(w * scale)
        resized = cv2.resize(frame, (new_width, DISPLAY_HEIGHT))

        if new_width < DISPLAY_WIDTH:
            pad = DISPLAY_WIDTH - new_width
            left = pad // 2
            right = pad - left
            resized = cv2.copyMakeBorder(resized, 0, 0, left, right, cv2.BORDER_CONSTANT, value=(0, 0, 0))
        elif new_width > DISPLAY_WIDTH:
            resized = resized[:, :DISPLAY_WIDTH]

        cv2.putText(resized, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        return resized

    def make_info_bar(self):
        total_width = DISPLAY_WIDTH * 2
        bar = np.zeros((INFO_BAR_HEIGHT, total_width, 3), dtype=np.uint8)

        col_width = total_width // 3
        headers = ["Game State", "Human Move", "Move Status"]
        values = [self.game_state, self.human_move, self.move_status]

        for i, (header, value) in enumerate(zip(headers, values)):
            x = i * col_width

            cv2.rectangle(bar, (x, 0), (x + col_width - 4, INFO_BAR_HEIGHT - 4), (50, 50, 50), -1)
            cv2.rectangle(bar, (x, 0), (x + col_width - 4, INFO_BAR_HEIGHT - 4), (100, 100, 100), 1)

            cv2.putText(bar, header, (x + 10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            if i == 2:
                color = (0, 255, 0) if value == "VALID" else (0, 0, 255) if value == "INVALID" else (255, 255, 255)
            else:
                color = (0, 255, 255)

            cv2.putText(bar, value, (x + 10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        return bar

    def display_timer_callback(self):
        raw_display = self.raw_frame if self.raw_frame is not None else np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)
        processed_display = self.processed_frame if self.processed_frame is not None else np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)
        live_detection_display = self.live_detection_frame if self.live_detection_frame is not None else np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)

        top_left = self.make_panel(raw_display, "Raw Camera Feed")
        top_right = self.make_panel(processed_display, "Warped Live Feed")
        bottom_left = self.make_panel(live_detection_display, "Warped + YOLO (Live)")
        bottom_right = np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)
        cv2.putText(bottom_right, "Board Scan Result", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        top_row = np.hstack([top_left, top_right])
        bottom_row = np.hstack([bottom_left, bottom_right])
        grid = np.vstack([top_row, bottom_row])

        info_bar = self.make_info_bar()
        combined = np.vstack([grid, info_bar])

        cv2.imshow("Chessbot Display", combined)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    display_output_node = DisplayOutput()
    try:
        rclpy.spin(display_output_node)
    except KeyboardInterrupt:
        pass
    display_output_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()