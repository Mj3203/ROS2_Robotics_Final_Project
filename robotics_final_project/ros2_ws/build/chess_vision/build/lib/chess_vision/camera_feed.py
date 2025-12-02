import rclpy
from rclpy.node import Node
from std_msgs.msg import Image, String # ROS2 message types
from cv_bridge import CvBridge # Bridge to convert between ROS and OpenCV images
import cv2 # OpenCV for image processing

class Camera_Feed(Node):
    def __init__(self):
        # Initialize the Node with the name 'camera_feed'
        super().__init__('camera_feed') 
        
        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Initialize the video capture object to read from the default camera (0)
        self.cap = cv2.VideoCapture(0)

        # Create a publisher for Image messages on the 'video_feed' topic
        self.publisher_ = self.create_publisher(Image, 'video_feed', 10) 

        # Set a timer to call the timer_callback function every x amount of seconds
        timer_period = 0.033  # seconds (~30 FPS)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("Camera Feed Node has been started.")

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            # Convert the OpenCV image (frame or a numpy array) to a ROS Image message
            # OpenCV uses BGR format by default
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            
            # Publish the ROS Image message
            self.publisher_.publish(ros_image)
            self.get_logger().info('Published video frame')
        else:
            self.get_logger().error('Failed to capture video frame')
    
    def destroy_node(self):
        # Release the video capture object when destroying the node
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_feed = Camera_Feed()

    # Node runs indefinitely until interrupted
    # Ctrl+C stops the timer and shuts down the node rather than crashes
    try:
        rclpy.spin(camera_feed)
    except KeyboardInterrupt:
        pass
    
    # Destroy the node explicitly
    camera_feed.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()