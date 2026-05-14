import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # ROS2 message type
from cv_bridge import CvBridge # Bridge to convert between ROS and OpenCV images
import cv2 # OpenCV for image processing
import numpy as np

# Target display height for each panel in the 2x2 grid (width scales proportionally)
# OLD: DISPLAY_HEIGHT = 480
# NEW: Reduced to 360 so the 2x2 grid fits comfortably on screen
DISPLAY_HEIGHT = 360

# Target display width for each panel (used for black placeholder frames)
DISPLAY_WIDTH = 640

class DisplayOutput(Node):
    def __init__(self):
        # Initialize the Node with the name 'display_output_node'
        super().__init__('display_output_node')

        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Store the latest frames from each feed
        self.raw_frame = None
        self.processed_frame = None

        # NEW: Store the latest live detection frame and last board scan snapshot
        self.live_detection_frame = None
        self.scan_result_frame = None

        # Subscribe to the raw camera feed
        self.raw_sub = self.create_subscription(
            Image,
            'raw_camera_feed',
            self.raw_camera_feed_callback,
            10
        )

        # Subscribe to the processed camera feed
        self.processed_sub = self.create_subscription(
            Image,
            'processed_camera_feed',
            self.processed_camera_feed_callback,
            10
        )

        # NEW: Subscribe to the live YOLO detection feed
        self.live_detection_sub = self.create_subscription(
            Image,
            'live_detection_feed',
            self.live_detection_feed_callback,
            10
        )

        # NEW: Subscribe to the board scan snapshot feed
        self.scan_result_sub = self.create_subscription(
            Image,
            'scan_result_feed',
            self.scan_result_feed_callback,
            10
        )

        # Set a timer to update the display window at 10Hz
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.display_timer_callback)

        # OLD: self.get_logger().info("Display Output Node Ready. Subscribing to /raw_camera_feed and /processed_camera_feed.")
        # NEW: Updated log message to reflect all 4 subscriptions
        self.get_logger().info("Display Output Node Ready. Subscribing to /raw_camera_feed, /processed_camera_feed, /live_detection_feed, /scan_result_feed.")

    def raw_camera_feed_callback(self, data):
        # Convert ROS Image message to OpenCV image
        try:
            self.raw_frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert raw camera feed: {e}")

    def processed_camera_feed_callback(self, data):
        # Convert ROS Image message to OpenCV image
        try:
            self.processed_frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert processed camera feed: {e}")

    # NEW: Callback for live YOLO detection feed
    def live_detection_feed_callback(self, data):
        # Convert ROS Image message to OpenCV image
        try:
            self.live_detection_frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert live detection feed: {e}")

    # NEW: Callback for board scan snapshot feed
    def scan_result_feed_callback(self, data):
        # Convert ROS Image message to OpenCV image
        # Only updates when game_operation_node triggers a scan
        try:
            self.scan_result_frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert scan result feed: {e}")

    def resize_to_height(self, frame, height):
        # Resize frame to target height while maintaining aspect ratio
        h, w = frame.shape[:2]
        scale = height / h
        new_width = int(w * scale)
        return cv2.resize(frame, (new_width, height))

    # OLD: resize_to_dimensions forced exact dimensions causing distortion
    # def resize_to_dimensions(self, frame, width, height):
    #     return cv2.resize(frame, (width, height))

    def make_panel(self, frame, label):
        # NEW: Resize to target height maintaining aspect ratio, then pad width with
        # black bars to reach DISPLAY_WIDTH — prevents distortion on non-16:9 images
        h, w = frame.shape[:2]
        scale = DISPLAY_HEIGHT / h
        new_width = int(w * scale)
        resized = cv2.resize(frame, (new_width, DISPLAY_HEIGHT))

        # Pad or crop width to match DISPLAY_WIDTH
        if new_width < DISPLAY_WIDTH:
            # Add black bars on left and right (letterbox)
            pad = DISPLAY_WIDTH - new_width
            left = pad // 2
            right = pad - left
            resized = cv2.copyMakeBorder(resized, 0, 0, left, right, cv2.BORDER_CONSTANT, value=(0, 0, 0))
        elif new_width > DISPLAY_WIDTH:
            # Crop if wider than target (rare case)
            resized = resized[:, :DISPLAY_WIDTH]

        # Add label to the panel
        cv2.putText(resized, label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        return resized

    def display_timer_callback(self):
        # Use placeholder black frames if a feed is not yet available
        raw_display = self.raw_frame if self.raw_frame is not None else np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)
        processed_display = self.processed_frame if self.processed_frame is not None else np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)

        # NEW: Placeholder black frames for the two new panels
        live_detection_display = self.live_detection_frame if self.live_detection_frame is not None else np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)
        scan_result_display = self.scan_result_frame if self.scan_result_frame is not None else np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)

        # NEW: Build each panel with label using make_panel helper
        # OLD: Resize both frames to the same height and hstack
        top_left     = self.make_panel(raw_display,           "Raw Camera Feed")
        top_right    = self.make_panel(processed_display,     "Warped Live Feed")
        bottom_left  = self.make_panel(live_detection_display,"Warped + YOLO (Live)")
        bottom_right = self.make_panel(scan_result_display,   "Last Board Scan")

        # NEW: Stack panels into a 2x2 grid
        # OLD: combined = np.hstack([raw_display, processed_display])
        top_row    = np.hstack([top_left, top_right])
        bottom_row = np.hstack([bottom_left, bottom_right])
        combined   = np.vstack([top_row, bottom_row])

        # Display the combined 2x2 window
        cv2.imshow("Chessbot Display", combined)
        cv2.waitKey(1)

    def destroy_node(self):
        # Close all OpenCV windows when shutting down
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    display_output_node = DisplayOutput()

    # Node runs indefinitely until interrupted
    # Ctrl+C stops the node and shuts down rather than crashes
    try:
        rclpy.spin(display_output_node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    display_output_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()