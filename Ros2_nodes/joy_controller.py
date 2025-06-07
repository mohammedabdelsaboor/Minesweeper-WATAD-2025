
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoyToCmdVel(Node):
    def __init__(self):
        super().__init__('joy_to_cmdvel_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info("🕹️ Joy to CmdVel node started")

    def joy_callback(self, msg: Joy):
        twist = Twist()

        linear_x = msg.axes[1]  
        linear_y = msg.axes[0] 
        angular_z = msg.axes[3] if len(msg.axes) > 3 else 0.0 

        
        speed = 1
        twist.linear.x = linear_x * speed
        twist.linear.y = linear_y * speed
        twist.angular.z = angular_z * speed

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
