# Sample ROS 2 Node for Humanoid Robot Control
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('Humanoid Controller Node Started')

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'Received command: linear={msg.linear}, angular={msg.angular}')

def main(args=None):
    rclpy.init(args=args)
    humanoid_controller = HumanoidController()

    try:
        rclpy.spin(humanoid_controller)
    except KeyboardInterrupt:
        pass
    finally:
        humanoid_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()