# my_image_subscriber/yolo_person_detector_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        self.subscription = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # Pretrained YOLOv8 model
        self.frame_count = 0  # Counter to process only every 10th frame

        self.get_logger().info("YOLOv8 person detector started.")

    def image_callback(self, msg):
        self.frame_count += 1
        # Only process every 10th frame
        if self.frame_count % 10 != 0:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(frame)[0]
            for box in results.boxes:
                cls_id = int(box.cls[0])
                if cls_id == 0:  # Class 0 = person
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    x1, y1, x2, y2 = xyxy
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, 'Person', (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.imshow("YOLO Person Detection", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in YOLO detection: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
