import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial
import time

class DiffDriveSerialNode(Node):
    def __init__(self):
        super().__init__('diff_drive_serial')

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        self.wheel_base = 0.3  
        self.max_speed = 1.0  

        self.get_logger().info("DiffDrive Serial Node Started")

    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        v_left = linear_vel - (angular_vel * self.wheel_base / 2)
        v_right = linear_vel + (angular_vel * self.wheel_base / 2)

        left_speed = int(v_left * 100)
        right_speed = int(v_right * 100)

        left_speed = max(min(left_speed, 255), -255)
        right_speed = max(min(right_speed, 255), -255)

        command = f"L{left_speed} R{right_speed}\n"
        self.ser.write(command.encode('utf-8'))

        self.get_logger().info(f"Sent to Arduino: {command.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
