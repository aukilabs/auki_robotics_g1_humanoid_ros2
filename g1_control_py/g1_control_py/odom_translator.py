import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from unitree_go.msg import SportModeState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import json

class OdomModeStateListener(Node):
    def __init__(self):
        super().__init__('odommodestate_listener')

        # Subscriber
        self.subscription = self.create_subscription(
            SportModeState,
            '/odommodestate',
            self.odommodestate_callback,
            10  # QoS profile depth
        )
        self.subscription  # Prevent unused variable warning
        self.get_logger().info("Listening to /odommodestate")

        # Publisher
        self.publisher = self.create_publisher(Odometry, '/odom', 10)

        # Transform Broaddcaster
        self.tf_broadcaster = TransformBroadcaster(self)


    def odommodestate_callback(self, odom_mode_state_msg):
      
        odom_msg = Odometry()

        # Header
        # This won't work because the odom_mode_state_msg return time 0
        # odom_msg.header.stamp = rclpy.time.Time(
        #     seconds=int(odom_mode_state_msg.stamp.sec), 
        #     nanoseconds=int(odom_mode_state_msg.stamp.nanosec)).to_msg()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id="odom"
        odom_msg.child_frame_id="pelvis"

        # Pose
        quat = odom_mode_state_msg.imu_state.quaternion # [w, x, y, z]
        odom_msg.pose.pose.orientation.w = float(quat[0])
        odom_msg.pose.pose.orientation.x = float(quat[1])
        odom_msg.pose.pose.orientation.y = float(quat[2])
        odom_msg.pose.pose.orientation.z = float(quat[3])

        position = odom_mode_state_msg.position
        odom_msg.pose.pose.position.x = float(position[0])
        odom_msg.pose.pose.position.y = float(position[1])
        odom_msg.pose.pose.position.z = float(position[2])

        # Velocity
        velocity = odom_mode_state_msg.velocity
        odom_msg.twist.twist.linear.x = float(velocity[0])
        odom_msg.twist.twist.linear.y = float(velocity[1])
        odom_msg.twist.twist.linear.z = float(velocity[2])

        odom_msg.twist.twist.angular.z = float(odom_mode_state_msg.yaw_speed)

        self.publisher.publish(odom_msg)

        t = TransformStamped()
        # Read message content and assign it to
        # corresponding tf variables
        t.header = odom_msg.header
        t.child_frame_id = "pelvis"

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        t.transform.rotation.x = odom_msg.pose.pose.orientation.x
        t.transform.rotation.y = odom_msg.pose.pose.orientation.y
        t.transform.rotation.z = odom_msg.pose.pose.orientation.z
        t.transform.rotation.w = odom_msg.pose.pose.orientation.w

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

        t2 = TransformStamped()
        # Read message content and assign it to
        # corresponding tf variables
        t2.header = odom_msg.header
        t2.header.frame_id = "pelvis"
        t2.child_frame_id = "pelvis_base"

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = -odom_msg.pose.pose.position.z

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0

        # Send the transformation
        self.tf_broadcaster.sendTransform(t2)




def main(args=None):
    rclpy.init(args=args)
    node = OdomModeStateListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
