import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.bridge = CvBridge()
        
        # 訂閱相機影像
        self.subscription = self.create_subscription(
            Image, 
            '/image', 
            self.image_callback, 
            10
        )

        # 發佈處理後的影像
        self.publisher = self.create_publisher(
            Image, 
            '/yolo/detections', 
            10
        )

        # 載入 YOLO 模型
        #self.model = YOLO('/mnt/data/best.pt')  # 載入你的 YOLOv8 模型
        self.model = YOLO('/workspace/best.pt')

        self.get_logger().info("YOLO 模型已載入！")

    def image_callback(self, msg):
        # 轉換 ROS 影像到 OpenCV 格式
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 進行 YOLO 偵測
        results = self.model(frame)[0]
        
        # 畫出偵測結果
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])  # 取得邊界框座標
            conf = box.conf[0].item()  # 取得信心分數
            label = results.names[int(box.cls[0])]  # 取得類別名稱

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'{label} {conf:.2f}', (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 發佈結果影像
        img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher.publish(img_msg)

        self.get_logger().info("發佈 YOLO 偵測結果")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

