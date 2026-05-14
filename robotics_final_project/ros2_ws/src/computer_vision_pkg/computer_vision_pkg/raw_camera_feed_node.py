import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # ROS2 message types
from std_msgs.msg import String # ROS2 message types
from cv_bridge import CvBridge # Bridge to convert between ROS and OpenCV images
import cv2 # OpenCV for image processing

class Raw_Camera_Feed(Node):
    def __init__(self):
        # Initialize the Node with the name 'raw_camera_feed'
        super().__init__('raw_camera_feed_node') 
        
        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Initialize the video capture object to read from the default camera (0)
        self.cap = cv2.VideoCapture(0)

        # Set resolution of video feed (optional)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        # Create a publisher for Image messages on the 'raw_camera_feed' topic
        self.publisher_ = self.create_publisher(Image, 'raw_camera_feed', 10) 

        # Set a timer to call the timer_callback function every x amount of seconds
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("Raw Camera Feed Node ready.")

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.get_logger().error('Failed to capture video frame')
            return
        
        cv2.imshow("Raw Camera Feed", frame)
        cv2.waitKey(1)
        
        # Convert the OpenCV image (frame or a numpy array) to a ROS Image message
        # OpenCV uses BGR format by default
        raw_camera_feed = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            
        # Publish the ROS Image message
        self.publisher_.publish(raw_camera_feed)
        self.get_logger().info('Publishing raw camera feed')
    
    def destroy_node(self):
        # Release the video capture object when destroying the node
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    raw_camera_feed_node = Raw_Camera_Feed()

    # Node runs indefinitely until interrupted
    # Ctrl+C stops the timer and shuts down the node
    try:
        rclpy.spin(raw_camera_feed_node)
    except KeyboardInterrupt:
        pass
    
    # Destroy the node explicitly
    raw_camera_feed_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()