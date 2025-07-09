import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
# from unitree_go.msg import Go2FrontVideoData
from unitree_api.msg import Request, RequestHeader, RequestIdentity, Response
import numpy as np
import json


class FrontVideoListener(Node):
    def __init__(self):
        super().__init__('front_video_listener')

        self.group = MutuallyExclusiveCallbackGroup()
        # self.group_2 = ReentrantCallbackGroup()
        self.group_2 = MutuallyExclusiveCallbackGroup()
        # Subscribers
        self.subscription = self.create_subscription(
            Response,
            '/api/videohub/response',
            self.video_frame_callback,
            100,  # QoS profile depth
            callback_group=self.group
        )
        self.get_logger().info("Listening to /api/videohub/response")

        # Publishers
        self.img_publisher = self.create_publisher(Image, '/rs_camera/d435i/color/image_raw', 100)
        self.info_publisher = self.create_publisher(CameraInfo, '/rs_camera/d435i/color/camera_info', 100)
        self.req_publisher = self.create_publisher(Request, '/api/videohub/request', 100)

        # Wall Timer
        self.timer = self.create_timer(0.033, self.video_frame_request, callback_group=self.group_2)  # Request Image Frame @10Hz

        self.bridge = CvBridge()

    def video_frame_request(self):
        req_id = RequestIdentity()
        req_id.api_id = 1001

        req_header = RequestHeader()
        req_header.identity = req_id

        req_msg = Request()
        req_msg.header = req_header

        self.req_publisher.publish(req_msg)


    def video_frame_callback(self, msg):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'd435i_color_optical_frame'

        image_data = np.frombuffer(msg.binary, dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        image = cv2.resize(image, (1280, 720))
        self.get_logger().debug(f"Received image frame: {np.shape(image)}")
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")

        info_msg = CameraInfo()
        info_msg.width = 1280
        info_msg.height = 720
        info_msg.distortion_model = 'plumb_bob'
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info_msg.k = [911.931396484375, 0.0, 655.7067260742188, 
                      0.0, 911.4285278320312, 373.84521484375, 
                      0.0, 0.0, 1.0]
        info_msg.r = [1.0, 0.0, 0.0, 
                      0.0, 1.0, 0.0, 
                      0.0, 0.0, 1.0 ]
        info_msg.p = [911.931396484375, 0.0, 655.7067260742188, 0.0,
                      0.0, 911.4285278320312, 373.84521484375, 0.0,
                      0.0, 0.0, 1.0, 0.0]

        img_msg.header = header
        info_msg.header = header
        
        # publishes message
        self.img_publisher.publish(img_msg)
        self.info_publisher.publish(info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrontVideoListener()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
