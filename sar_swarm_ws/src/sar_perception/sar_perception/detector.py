import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth',
            self.depth_callback,
            10)

        self.target_pub = self.create_publisher(
            PoseStamped,
            '/perception/target_coords',
            10)

        self.current_depth_image = None
        self.get_logger().info("Perception Node Initialized with 45ms artificial latency")

    def depth_callback(self, msg):
        self.current_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_callback(self, msg):
        # Inject artificial latency to simulate VPU lag (crucial for testing controller stability)
        time.sleep(0.045)

        if self.current_depth_image is None:
            return

        # Mock Detection Logic
        # Assume center pixel depth check
        height, width = self.current_depth_image.shape
        center_depth = self.current_depth_image[height // 2, width // 2]

        # OAK-D depth is usually in mm or meters depending on driver.
        # Requirement says center pixel depth < 2.0m
        # If it's float (meters)
        if center_depth < 2.0:
            self.get_logger().info(f"Person Found! Depth: {center_depth:.2f}m")

            target_msg = PoseStamped()
            target_msg.header.stamp = self.get_clock().now().to_msg()
            target_msg.header.frame_id = "camera_link"
            target_msg.pose.position.x = float(center_depth) # In camera coordinates
            target_msg.pose.position.y = 0.0
            target_msg.pose.position.z = 0.0
            self.target_pub.publish(target_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
