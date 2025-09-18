import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import joblib
import os
import pandas as pd  # ✅ Used for clean prediction input

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        # Locate installed model directory
        install_base = os.environ['AMENT_PREFIX_PATH'].split(':')[0]
        model_dir = os.path.join(install_base, 'share', 'room_navigator', 'models')
        model_path = os.path.join(model_dir, 'room_decision_tree.pkl')
        encoder_path = os.path.join(model_dir, 'label_encoders.pkl')

        # Logging model paths
        self.get_logger().info(f"📦 Loading model from: {model_path}")
        self.get_logger().info(f"📦 Loading encoders from: {encoder_path}")

        # Load model and encoders
        self.model = joblib.load(model_path)
        self.le_time, self.le_task, self.le_dirt, self.le_target = joblib.load(encoder_path)

        # ROS2 Subscriber and Publisher
        self.subscription = self.create_subscription(
            String,
            '/task_conditions',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(String, '/target_room', 10)

        self.get_logger().info('✅ Decision Node started... Waiting for task input.')

    def listener_callback(self, msg):
        self.get_logger().info(f"📩 Received task input: {msg.data}")
        try:
            time_str, task_str, dirt_str = msg.data.strip().lower().split(',')

            # ✅ Use DataFrame to avoid warning
            X_input = pd.DataFrame([{
                'time_encoded': self.le_time.transform([time_str])[0],
                'task_encoded': self.le_task.transform([task_str])[0],
                'dirt_encoded': self.le_dirt.transform([dirt_str])[0]
            }])

            pred_class = self.model.predict(X_input)[0]
            predicted_room = self.le_target.inverse_transform([pred_class])[0]

            # Publish predicted room
            out_msg = String()
            out_msg.data = predicted_room
            self.publisher_.publish(out_msg)

            self.get_logger().info(f"🤖 Predicted Room: {predicted_room} ✅ Published to /target_room")

        except Exception as e:
            self.get_logger().error(f"❌ Prediction failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

