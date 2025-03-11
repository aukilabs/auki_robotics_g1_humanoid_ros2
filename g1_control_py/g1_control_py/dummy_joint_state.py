import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class DummyJointStatePublisher(Node):
    def __init__(self):
        super().__init__('dummy_joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz
        self.joint_names = [
            'L_index_proximal_joint', 'L_middle_proximal_joint', 'L_pinky_proximal_joint',
            'L_ring_proximal_joint', 'L_thumb_proximal_yaw_joint', 'L_thumb_proximal_pitch_joint',
            'R_index_proximal_joint', 'R_middle_proximal_joint', 'R_pinky_proximal_joint',
            'R_ring_proximal_joint', 'R_thumb_proximal_yaw_joint', 'R_thumb_proximal_pitch_joint'
        ]
        self.joint_positions = [0.0] * len(self.joint_names)  # Initialize positions to zero
        self.joint_velocities = [0.0] * len(self.joint_names)
        self.joint_efforts = [0.0] * len(self.joint_names)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = self.joint_velocities
        msg.effort = self.joint_efforts
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published joint states: {msg}')


def main(args=None):
    rclpy.init(args=args)
    node = DummyJointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
