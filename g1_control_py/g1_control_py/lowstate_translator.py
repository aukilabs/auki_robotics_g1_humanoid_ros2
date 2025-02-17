import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from unitree_hg.msg import LowState, MotorState
import json

g1_joints_name = [
                    "left_hip_pitch_joint" ,    # 0
                    "left_hip_roll_joint" ,     # 1
                    "left_hip_yaw_joint" ,      # 2
                    "left_knee_joint" ,         # 3
                    "left_ankle_pitch_joint" ,  # 4
                    "left_ankle_roll_joint" ,   # 5

                    "right_hip_pitch_joint" ,   # 6
                    "right_hip_roll_joint" ,    # 7
                    "right_hip_yaw_joint" ,     # 8
                    "right_knee_joint" ,        # 9
                    "right_ankle_pitch_joint" , # 10
                    "right_ankle_roll_joint" ,  # 11
                    
                    "waist_yaw_joint" ,         # 12
                    "waist_roll_joint" ,        # 13
                    "waist_pitch_joint" ,       # 14
                    
                    "left_shoulder_pitch_joint", # 15
                    "left_shoulder_roll_joint",  # 16
                    "left_shoulder_yaw_joint",   # 17
                    "left_elbow_joint",          # 18
                    "left_wrist_roll_joint",     # 19
                    "left_wrist_pitch_joint",    # 20
                    "left_wrist_yaw_joint",      # 21

                    "right_shoulder_pitch_joint", # 22
                    "right_shoulder_roll_joint",  # 23
                    "right_shoulder_yaw_joint",   # 24
                    "right_elbow_joint",          # 25
                    "right_wrist_roll_joint",     # 26
                    "right_wrist_pitch_joint",    # 27
                    "right_wrist_yaw_joint",      # 28

                    "not_used_0", # 29
                    "not_used_1", # 30
                    "not_used_2", # 31
                    "not_used_3", # 32
                    "not_used_4", # 33
                    "not_used_5", # 34
                ]


class LowStateListener(Node):
    def __init__(self):
        super().__init__('lowstate_listener')

        # Subscriber
        self.subscription = self.create_subscription(
            LowState,
            '/lowstate',
            self.lowstate_callback,
            10  # QoS profile depth
        )
        self.subscription  # Prevent unused variable warning
        self.get_logger().info("Listening to /lowstate")

        # Publisher
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

    def lowstate_callback(self, low_state_msg):

        motor_states = low_state_msg.motor_state

        joint_states_msg = JointState()
        joint_states_msg.header.stamp = self.get_clock().now().to_msg()

        for i, motor_state in enumerate(motor_states):
            motor_name = g1_joints_name[i]
            if 'not_used' in motor_name:
                continue
            joint_states_msg.name.append(motor_name)
            joint_states_msg.position.append(motor_state.q)
            joint_states_msg.velocity.append(motor_state.dq)
            joint_states_msg.effort.append(motor_state.tau_est)

        self.publisher.publish(joint_states_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LowStateListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
