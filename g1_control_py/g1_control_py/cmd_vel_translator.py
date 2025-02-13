import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_api.msg import Request, RequestHeader, RequestIdentity
import json

class CmdVelListener(Node):
    def __init__(self):
        super().__init__('cmd_vel_listener')

        # Parameters
        self.declare_parameter('network_interface', 'eth0')

        # Subscriber
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10  # QoS profile depth
        )
        self.subscription  # Prevent unused variable warning
        self.get_logger().info("Listening to /cmd_vel")

        # Publisher
        self.publisher = self.create_publisher(Request, '/api/loco/request', 10)

    def cmd_vel_callback(self, msg):
        self.get_logger().debug(f"Received cmd_vel: linear=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), "
                               f"angular=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})")

        req_id = RequestIdentity()
        req_id.api_id = 7105

        req_header = RequestHeader()
        req_header.identity = req_id

        req_msg = Request()
        req_msg.header = req_header

        cmd_dict = {
            "velocity": [msg.linear.x, msg.linear.y, msg.angular.z],
            "duration": 1
        } 
        req_msg.parameter = json.dumps(cmd_dict)
        self.get_logger().debug(f"sending parameter: {req_msg.parameter}")
        self.publisher.publish(req_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
