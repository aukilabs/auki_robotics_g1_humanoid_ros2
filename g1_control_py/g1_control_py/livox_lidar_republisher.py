import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class LivoxLidarRepublisher(Node):
    def __init__(self):
        super().__init__('livox_lidar_republisher')

        # Subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud_livox_mid360',
            self.lidar_callback,
            10  # QoS profile depth
        )
        self.subscription  # Prevent unused variable warning
        self.get_logger().info("Listening to /utlidar/cloud_livox_mid360")

        # Publisher
        self.publisher = self.create_publisher(PointCloud2, '/livox/lidar', 10)

    def lidar_callback(self, lidar_msg):
        lidar_msg.header.stamp = self.get_clock().now().to_msg()
        # lidar_msg.header.frame_id = "mid360_link"
        self.publisher.publish(lidar_msg)
        # self.get_logger().info("published lidar")

def main(args=None):
    rclpy.init(args=args)
    node = LivoxLidarRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
