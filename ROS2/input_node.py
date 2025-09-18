import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')
        self.publisher_ = self.create_publisher(String, '/task_conditions', 10)
        self.timer = self.create_timer(5.0, self.publish_task)

        # Lists of options
        self.times = ['morning', 'afternoon', 'evening', 'night']
        self.tasks = ['delivery', 'cleaning', 'charging']
        self.dirt_levels = ['low', 'medium', 'high']

        self.get_logger().info('🟡 Input Node started... Publishing random tasks every 5s')

    def publish_task(self):
        time = random.choice(self.times)
        task = random.choice(self.tasks)
        dirt = random.choice(self.dirt_levels)

        msg = String()
        msg.data = f"{time},{task},{dirt}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"📤 Published task: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = InputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

