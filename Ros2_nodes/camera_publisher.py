import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import time

class CarDetectorNode(Node):
    def __init__(self):
        super().__init__('car_detector_node')
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.alert_pub = self.create_publisher(Bool, '/car_speed_alert', 10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.prev_positions = {}
        self.prev_time = None

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)
        current_time = time.time()
        alert = False

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                if cls == 2 or cls == 7:
                    x_center = int((box.xywh[0][0]))
                    id = hash((cls, x_center))
                    if id in self.prev_positions:
                        dx = abs(x_center - self.prev_positions[id])
                        dt = current_time - self.prev_time
                        speed = dx / dt
                        if speed > 30:
                            alert = True
                    self.prev_positions[id] = x_center

        self.prev_time = current_time
        msg_out = Bool()
        msg_out.data = alert
        self.alert_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = CarDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
