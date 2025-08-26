import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import py_trees
import py_trees_ros2
import cv2


class RotateUntilPersonDetected(py_trees_ros2.behaviours.Behaviour):
    def __init__(self, name="RotateUntilPersonDetected"):
        super().__init__(name)
        self.node = rclpy.create_node(name + "_node")
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.image_sub = self.node.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            10)
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

        self.frame_count = 0
        self.person_detected = False
        self.rotating = False

        self.get_logger = self.node.get_logger

    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % 10 != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(frame)[0]
            self.person_detected = False
            for box in results.boxes:
                cls_id = int(box.cls[0])
                if cls_id == 0:  # Person class
                    self.person_detected = True
                    # Draw rectangle and label (optional)
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    x1, y1, x2, y2 = xyxy
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, 'Person', (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.imshow("YOLO Person Detection", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error in YOLO detection: {e}")

    def update(self):
        rclpy.spin_once(self.node, timeout_sec=0.01)

        if self.person_detected:
            # Stop robot rotation
            twist = Twist()
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return py_trees.common.Status.SUCCESS
        else:
            # Rotate robot to scan environment
            twist = Twist()
            twist.angular.z = 0.3  # Rotate at 0.3 rad/s, adjust as needed
            self.cmd_vel_pub.publish(twist)
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # Stop rotation when behavior finishes or halts
        twist = Twist()
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        cv2.destroyAllWindows()
        self.node.destroy_node()
