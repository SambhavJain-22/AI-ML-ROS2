import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')

        # Subscribe to predicted room topic
        self.subscription = self.create_subscription(
            String,
            '/target_room',
            self.listener_callback,
            10
        )

        self.get_logger().info('🚀 Navigator Node started. Waiting for room target...')

    def listener_callback(self, msg):
        room = msg.data
        self.get_logger().info(f"🧭 Moving robot to {room}...")

        # Simulated delay (pretend the robot is navigating)
        for i in range(3):
            time.sleep(1)
            self.get_logger().info(f"  ...moving...{'.' * (i + 1)}")

        self.get_logger().info(f"✅ Reached {room}!")

def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
