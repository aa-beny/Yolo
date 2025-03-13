import rclpy
from rclpy.node import Node
# 用這指令找  ros2 interface list
# sensor_msgs/msg/Image
import cv2
# from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class USBCamera(Node):

    def __init__(self):
        super().__init__('usb_camera')
        self.publisher_ = self.create_publisher(Image, '/image', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("無法開啟相機")
        self.bridge = CvBridge()
        

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('no')
            print("無法讀取相機畫面")
        frame = self.bridge. cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(frame)



def main(args=None):
    rclpy.init(args=args)

    usb_publisher = USBCamera()

    rclpy.spin(usb_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    usb_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()