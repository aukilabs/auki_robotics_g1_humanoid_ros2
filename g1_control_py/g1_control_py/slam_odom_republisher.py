import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class SlamOdomRepublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_republisher')

        # Subscriber
        self.subscription = self.create_subscription(
            Odometry,
            '/unitree/slam_mapping/odom',
            self.odom_callback,
            10  # QoS profile depth
        )
        self.subscription  # Prevent unused variable warning
        self.get_logger().info("Listening to /unitree/slam_mapping/odom")

        # Publisher
        self.publisher = self.create_publisher(Odometry, '/odom_slam', 10)

        # Transform Broaddcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, odom_msg):
        odom_msg.header.frame_id="odom"
        odom_msg.child_frame_id="pelvis_base"
        self.publisher.publish(odom_msg)

        t = TransformStamped()
        # Read message content and assign it to
        # corresponding tf variables
        t.header = odom_msg.header
        t.child_frame_id = "pelvis_base"

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

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = SlamOdomRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
