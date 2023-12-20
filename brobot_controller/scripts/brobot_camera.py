#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from sensor_msgs.msg import Image

class brobot_camera(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, '/video_frames', 10)
      
        # We will publish a message every 0.1 seconds
        timer_period = 0.05  # seconds
      
        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
                
        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(-1)
                
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
   
    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = self.cap.read()

        if ret == True:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
        
            # Display the message on the console
            # self.get_logger().info('Publishing video frame')

def main(args=None):
    rclpy.init(args=args)
    node = brobot_camera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
