import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener

class TFListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(5.0, self.publish_initpose)

    def publish_initpose(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'pelvis_base', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0))
            
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            
            pose_msg.pose.pose.position.x = transform.transform.translation.x
            pose_msg.pose.pose.position.y = transform.transform.translation.y
            pose_msg.pose.pose.position.z = transform.transform.translation.z
            
            pose_msg.pose.pose.orientation = transform.transform.rotation
            
            # Set covariance (identity matrix for simplicity)
            pose_msg.pose.covariance = [
                0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.1
            ]
            
            self.publisher.publish(pose_msg)
            self.get_logger().info('Published /initialpose')
        except Exception as e:
            self.get_logger().warn(f'Could not transform pelvis to map: {str(e)}')


def main():
    rclpy.init()
    node = TFListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
