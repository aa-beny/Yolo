import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import json
from ament_index_python.packages import get_package_share_directory
import os

class FishEyeCamera120(Node):
    def __init__(self, config):
        # Initialize the node with a name from the config
        super().__init__(config["node_name"])
        
        # Create a publisher with the topic name from the config
        self.publisher_ = self.create_publisher(Image, config["topic_name"], 10)
        
        # Set up a timer for the FPS specified in the config
        timer_period = 1 / config["fps"]  # Timer period in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Open the camera device
        self.cap = cv2.VideoCapture(config["camera_device"])
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            return
        
        # Configure camera parameters from the config
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config["fourcc"]))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config["frame_width"])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config["frame_height"])
        self.cap.set(cv2.CAP_PROP_FPS, config["fps"])
        
        # Verify that the camera settings are applied
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        codec = "".join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])
        self.get_logger().info(f"Camera initialized: Resolution={width}x{height}, FPS={fps}, Codec={codec}")
        
        # Initialize CvBridge for converting OpenCV images to ROS messages
        self.bridge = CvBridge()

    def timer_callback(self):
        # Capture a frame from the camera
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return
        
        # Convert the frame to a ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        
        # Publish the image message
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing image frame')


def load_config():
    # 獲取 package 的 share 目錄
    package_share_directory = get_package_share_directory('camera')
    config_path = os.path.join(package_share_directory, 'json', 'camera_config.json')
    
    # 從指定路徑讀取 JSON 設定檔
    try:
        with open(config_path, 'r') as file:
            config = json.load(file)
            return config
    except FileNotFoundError:
        print(f"Configuration file '{config_path}' not found.")
        exit(1)
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON: {e}")
        exit(1)


def main(args=None):
    # Load the configuration file
    config = load_config()
    
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the FishEyeCamera120 node with the loaded config
    node = FishEyeCamera120(config)
    
    # Keep the node running
    rclpy.spin(node)
    
    # Cleanup on shutdown
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
