import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration
import time

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()

        # Load YOLOv8 model
        self.model = YOLO('yolov8n.pt')

        # Subscriptions
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
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.info_callback,
            10)

        # Publishers
        self.victim_pub = self.create_publisher(
            PointStamped,
            '/sar/victim/detected',
            10)
        self.marker_pub = self.create_publisher(
            Marker,
            '/sar/victim/marker',
            10)

        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_depth_image = None
        self.camera_info = None
        self.get_logger().info("Perception Node (YOLOv8) Initialized")

    def info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        # Gazebo depth images are often 32FC1 (meters)
        self.current_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_callback(self, msg):
        if self.current_depth_image is None or self.camera_info is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLOv8 detection
        results = self.model(cv_image, classes=[0], conf=0.5, verbose=False) # class 0 is person

        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Get center of bounding box
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)

                # Sample depth with window averaging
                z = self.get_depth_at(u, v)

                if z is None or z <= 0:
                    continue

                # Deprojection to 3D point in Camera Optical Frame
                point_camera = self.deproject(u, v, z)

                # Transform to map frame
                self.transform_and_publish(point_camera, msg.header.stamp)

    def get_depth_at(self, u, v, window=5):
        h, w = self.current_depth_image.shape
        u_min = max(0, u - window)
        u_max = min(w, u + window)
        v_min = max(0, v - window)
        v_max = min(h, v + window)

        roi = self.current_depth_image[v_min:v_max, u_min:u_max]
        valid_depths = roi[np.isfinite(roi) & (roi > 0)]

        if len(valid_depths) == 0:
            return None
        return np.mean(valid_depths)

    def deproject(self, u, v, z):
        # Pinhole Camera Model
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        return (x, y, float(z))

    def transform_and_publish(self, point_camera, stamp):
        try:
            # Create PointStamped in camera optical frame
            # The requirement specifies rotated optical frame
            p = PointStamped()
            p.header.stamp = stamp
            p.header.frame_id = "oakd_pro_optical_frame"
            p.point.x = point_camera[0]
            p.point.y = point_camera[1]
            p.point.z = point_camera[2]

            # Transform to map frame
            # Wait for transform to be available
            transform = self.tf_buffer.lookup_transform('map', p.header.frame_id, stamp, timeout=Duration(seconds=0.1))
            p_map = tf2_geometry_msgs.do_transform_point(p, transform)

            # Publish detection
            self.victim_pub.publish(p_map)
            self.get_logger().info(f"Victim detected at: {p_map.point.x:.2f}, {p_map.point.y:.2f}, {p_map.point.z:.2f}")

            # Publish Marker for RViz
            marker = Marker()
            marker.header = p_map.header
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = p_map.point
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.id = int(time.time() * 1000) % 10000
            marker.lifetime = Duration(seconds=5.0).to_msg()
            self.marker_pub.publish(marker)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
