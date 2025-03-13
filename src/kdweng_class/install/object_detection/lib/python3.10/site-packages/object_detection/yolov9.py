import rclpy
from rclpy.node import Node
import cv2
from ultralytics import YOLO
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json
class YOLOv9(Node):

    def __init__(self):
        super().__init__('yolov9')
        self.subscription = self.create_subscription(
            Image,
            'image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(String, 'bbox', 10)

        self.bridge = CvBridge()
        self.model = YOLO('yolov9s.pt')

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # cv2.imshow('image',frame)
        # cv2.waitKey(1)

        results = self.model(frame)

    # Extract bounding boxes and other details
        detections = []
        for result in results[0].boxes:
            bbox = result.xyxy[0].tolist()  # Bounding box coordinates [x_min, y_min, x_max, y_max]
            confidence = result.conf[0].item()  # Confidence score
            class_id = result.cls[0].item()  # Class ID
            detections.append({
                "bbox": bbox,
                "confidence": confidence,
                "class_id": class_id
            })
        # json.dump(detections) 

        msg = String()
        msg.data = json.dump(detections)
        self.publisher_.publish(msg)
def main(args=None):
    rclpy.init(args=args)

    yolov9_subscriber = YOLOv9()

    rclpy.spin(yolov9_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    yolov9_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()