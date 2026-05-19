import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Raw_Camera_Feed(Node):

    def __init__(self):
        super().__init__('raw_camera_feed_node')
        self.publisher_ = self.create_publisher(Image, 'raw_camera_feed', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.setup_camera()
        self.get_logger().info("Raw Camera Feed Node ready.")

    def setup_camera(self):
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    def capture_frame(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return None
        return frame

    def convert_cv2_to_ros(self, frame):
        return self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

    def timer_callback(self):
        frame = self.capture_frame()
        if frame is None:
            self.get_logger().error('Failed to capture video frame')
            return
        self.publisher_.publish(self.convert_cv2_to_ros(frame))
        self.get_logger().info('Publishing raw camera feed')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    raw_camera_feed_node = Raw_Camera_Feed()
    try:
        rclpy.spin(raw_camera_feed_node)
    except KeyboardInterrupt:
        pass
    raw_camera_feed_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()