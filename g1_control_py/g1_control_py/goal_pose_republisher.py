import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import json

class GoalPoseRepublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_republisher')

        # Subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.goal_pose_callback,
            10  # QoS profile depth
        )
        self.subscription  # Prevent unused variable warning
        self.get_logger().info("Listening to /move_base_simple/goal")

        # Publisher
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def goal_pose_callback(self, goal_pose_msg):
        self.publisher.publish(goal_pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
