import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration
import message_filters

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('sar_perception')
        self.bridge = CvBridge()

        # Load YOLOv8 model
        self.model = YOLO('yolov8n.pt')

        # QoS Profile: Best Effort for PX4 and high-bandwidth sensor data
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Synchronized Subscriptions
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/image_raw', qos_profile=qos_best_effort)
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth', qos_profile=qos_best_effort)

        # ApproximateTimeSynchronizer to sync RGB and Depth
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.synchronized_callback)

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.info_callback,
            qos_best_effort)

        # Publisher
        self.victim_pub = self.create_publisher(
            PointStamped,
            '/sar/perception/victim_location',
            10)

        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.camera_info = None
        self.get_logger().info("SAR Perception Node (YOLOv8 + Depth Fusion) Initialized")

    def info_callback(self, msg):
        self.camera_info = msg

    def synchronized_callback(self, rgb_msg, depth_msg):
        if self.camera_info is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # Run YOLOv8 detection for "person" (class 0)
        results = self.model(cv_image, classes=[0], conf=0.5, verbose=False)

        for result in results:
            for box in result.boxes:
                # Get center of bounding box (u, v)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)

                # Read depth Z at (u, v) with 5x5 kernel average to handle NaNs
                z = self.get_depth_at(depth_image, u, v)

                if z is None or np.isnan(z) or z <= 0:
                    continue

                # 3D Projection (Pinhole Camera Model)
                # X = (u - cx) * Z / fx
                # Y = (v - cy) * Z / fy
                fx = self.camera_info.k[0]
                fy = self.camera_info.k[4]
                cx = self.camera_info.k[2]
                cy = self.camera_info.k[5]

                x_c = (u - cx) * z / fx
                y_c = (v - cy) * z / fy
                z_c = float(z)

                # Transform to map frame
                self.transform_and_publish(x_c, y_c, z_c, rgb_msg.header.frame_id, rgb_msg.header.stamp)

    def get_depth_at(self, depth_image, u, v, kernel_size=5):
        h, w = depth_image.shape
        half_k = kernel_size // 2
        u_min = max(0, u - half_k)
        u_max = min(w, u + half_k + 1)
        v_min = max(0, v - half_k)
        v_max = min(h, v + half_k + 1)

        roi = depth_image[v_min:v_max, u_min:u_max]
        valid_depths = roi[np.isfinite(roi) & (roi > 0)]

        if len(valid_depths) == 0:
            return None
        return np.mean(valid_depths)

    def transform_and_publish(self, x, y, z, frame_id, stamp):
        try:
            # Create PointStamped in camera optical frame
            p = PointStamped()
            p.header.stamp = stamp
            p.header.frame_id = frame_id
            p.point.x = x
            p.point.y = y
            p.point.z = z

            # Look up transform from camera_link_optical (frame_id) to map frame
            # Using 0.1s timeout as requested
            transform = self.tf_buffer.lookup_transform('map', frame_id, stamp, timeout=Duration(seconds=0.1))
            p_map = tf2_geometry_msgs.do_transform_point(p, transform)

            # Publish result
            self.victim_pub.publish(p_map)
            self.get_logger().info(f"Victim detected at: {p_map.point.x:.2f}, {p_map.point.y:.2f}, {p_map.point.z:.2f}")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
