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
MAX_LOG_LINES = 6

# ADDED: mapping from raw game_state to human readable log message
STATE_LABELS = {
    "STARTING_SYSTEM":          "Starting system",
    "WAITING_FOR_PLAYER_MOVE":  "Waiting on human",
    "SCANNING":                 "Scanning board",
    "VALIDATING_MOVE":          "Validating move",
    "CALLING_AI":               "Sending move to AI",
    "AI_COMPLETE":              "AI completed",
    "SENDING_TO_ARM":           "Sending move to arm",
    "ARM_MOVING":               "Arm moving piece",
    "GAME_OVER":                "Game over",
    "ERROR":                    "Error",
}


class DisplayOutput(Node):

    # Initializes the node, creates the feed subscriptions and render timer, and runs setup.
    def __init__(self):
        super().__init__('display_output_node')
        self.raw_sub = self.create_subscription(Image, 'raw_camera_feed', self.raw_camera_feed_callback, 10)
        self.processed_sub = self.create_subscription(Image, 'processed_camera_feed', self.processed_camera_feed_callback, 10)
        self.live_detection_sub = self.create_subscription(Image, 'live_detection_feed', self.live_detection_feed_callback, 10)
        self.game_status_sub = self.create_subscription(String, 'game_status_feed', self.game_status_callback, 10)
        self.timer = self.create_timer(0.1, self.display_timer_callback)
        self.setup_display()
        self.get_logger().info('Display Output Node is ready.')

    # Initializes the CvBridge and all display state fields.
    def setup_display(self):
        self.bridge = CvBridge()
        self.raw_frame = None
        self.processed_frame = None
        self.live_detection_frame = None
        self.game_state = "WAITING"
        self.human_move = ""
        self.ai_move = ""
        self.move_status = ""
        self.invalid_reason = ""
        self.game_status = ""
        self.move_number = 0
        self.human_color = ""
        self.ai_color = ""
        self.log_lines = []

    # Appends a message to the rolling game log, trimming the oldest line past the limit.
    def add_log(self, message):
        self.log_lines.append(message)
        if len(self.log_lines) > MAX_LOG_LINES:
            self.log_lines.pop(0)

    # Resizes a frame to the panel size, pads or crops it to fit, and draws its label.
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

    # Builds the game log panel listing the most recent status messages.
    def make_log_panel(self):
        panel = np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)
        cv2.putText(panel, "Game Log", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        for i, line in enumerate(self.log_lines):
            y = 65 + i * 45
            cv2.putText(panel, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        return panel

    # Builds the human-readable move/result text shown in the status column.
    def make_move_status_text(self):
        # CHANGED: all em dashes replaced with hyphens for OpenCV ASCII compatibility
        if self.game_status == "CHECKMATE_WHITE_WINS":
            return "Checkmate - White Wins"
        if self.game_status == "CHECKMATE_BLACK_WINS":
            return "Checkmate - Black Wins"
        if self.game_status == "CHECKMATE":
            return "Checkmate"
        if self.game_status == "STALEMATE":
            return "Stalemate"
        if self.game_status == "DRAW_INSUFFICIENT_MATERIAL":
            return "Draw - Insufficient Material"
        if self.game_status == "DRAW_REPETITION":
            return "Draw - Repetition"
        if self.move_status == "INVALID":
            return "Invalid Move"
        if self.move_number > 0:
            if self.game_status == "CHECK":
                return f"Move {self.move_number} - Check"
            return f"Move {self.move_number} - {self.human_color} Turn"
        return ""

    # Builds the bottom info bar showing the human move, AI move, and move status.
    def make_info_bar(self):
        total_width = DISPLAY_WIDTH * 2
        bar = np.zeros((INFO_BAR_HEIGHT, total_width, 3), dtype=np.uint8)
        col_width = total_width // 3

        human_label = f"Human ({self.human_color})" if self.human_color else "Human Move"
        ai_label = f"AI ({self.ai_color})" if self.ai_color else "AI Move"
        headers = [human_label, ai_label, "Move Status"]
        values = [self.human_move, self.ai_move, self.make_move_status_text()]

        for i, (header, value) in enumerate(zip(headers, values)):
            x = i * col_width
            cv2.rectangle(bar, (x, 0), (x + col_width - 4, INFO_BAR_HEIGHT - 4), (50, 50, 50), -1)
            cv2.rectangle(bar, (x, 0), (x + col_width - 4, INFO_BAR_HEIGHT - 4), (100, 100, 100), 1)
            cv2.putText(bar, header, (x + 10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            if i == 2:
                if "Checkmate" in value:
                    color = (0, 215, 255)    # gold
                elif "Check" in value:
                    color = (0, 165, 255)    # orange
                elif "Draw" in value or "Stalemate" in value:
                    color = (255, 255, 0)    # cyan
                elif "Invalid" in value:
                    color = (0, 0, 255)      # red
                else:
                    color = (0, 255, 0)      # green
            else:
                color = (0, 255, 255)
            cv2.putText(bar, value, (x + 10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        return bar

    # Stores the latest raw camera frame for display.
    def raw_camera_feed_callback(self, data):
        try:
            self.raw_frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert raw camera feed: {e}")

    # Stores the latest warped/processed camera frame for display.
    def processed_camera_feed_callback(self, data):
        try:
            self.processed_frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert processed camera feed: {e}")

    # Stores the latest YOLO live-detection frame for display.
    def live_detection_feed_callback(self, data):
        try:
            self.live_detection_frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert live detection feed: {e}")

    # Parses the incoming game status JSON and updates the display fields and log.
    def game_status_callback(self, data):
        try:
            status = json.loads(data.data)
            new_state = status.get("game_state", "")
            self.human_move = status.get("human_move", "")
            self.ai_move = status.get("ai_move", "")
            self.move_status = status.get("move_status", "")
            self.invalid_reason = status.get("invalid_reason", "")
            self.game_status = status.get("game_status", "")
            self.move_number = status.get("move_number", 0)
            self.human_color = status.get("human_color", "")
            self.ai_color = status.get("ai_color", "")

            # CHANGED: log readable state label instead of raw state string
            if new_state != self.game_state:
                self.game_state = new_state
                label = STATE_LABELS.get(self.game_state, self.game_state)
                self.add_log(label)

            # log invalid reason if present
            if self.move_status == "INVALID" and self.invalid_reason:
                self.add_log(f"Reason: {self.invalid_reason[:38]}")

        except Exception as e:
            self.get_logger().error(f"Failed to parse game status: {e}")

    # Assembles the 2x2 camera grid plus info bar and renders the combined window.
    def display_timer_callback(self):
        raw_display = self.raw_frame if self.raw_frame is not None else np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)
        processed_display = self.processed_frame if self.processed_frame is not None else np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)
        live_detection_display = self.live_detection_frame if self.live_detection_frame is not None else np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)

        top_left = self.make_panel(raw_display, "Raw Camera Feed")
        top_right = self.make_panel(processed_display, "Warped Live Feed")
        bottom_left = self.make_panel(live_detection_display, "Warped + YOLO (Live)")
        bottom_right = self.make_log_panel()

        top_row = np.hstack([top_left, top_right])
        bottom_row = np.hstack([bottom_left, bottom_right])
        grid = np.vstack([top_row, bottom_row])
        info_bar = self.make_info_bar()
        combined = np.vstack([grid, info_bar])

        cv2.imshow("Chessbot Display", combined)
        cv2.waitKey(1)

    # Closes OpenCV windows and shuts the node down.
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


# Initializes ROS, spins the node until interrupted, then shuts everything down.
def main(args=None):
    rclpy.init(args=args)
    display_output_node = DisplayOutput()
    try:
        rclpy.spin(display_output_node)
    except KeyboardInterrupt:
        pass
    display_output_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
