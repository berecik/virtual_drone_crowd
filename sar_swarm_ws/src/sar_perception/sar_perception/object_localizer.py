import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point, Pose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
import image_geometry
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration
from typing import Tuple, Optional, List

class ObjectLocalizerNode(Node):
    """
    ROS 2 Node for estimating 3D global positions of detected objects using OAK-D.

    This node synchronizes RGB and Depth streams, detects objects (simulated),
    projects the detection to 3D camera space, and transforms it to the global map frame.
    """

    def __init__(self) -> None:
        """Initialize the ObjectLocalizerNode."""
        super().__init__('object_localizer_node')

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Initialize TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize Camera Model
        self.camera_model = image_geometry.PinholeCameraModel()

        # QoS Profile: RELIABLE, TRANSIENT_LOCAL
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Publisher for the global 3D position
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/perception/human_found',
            qos_profile
        )

        # Synchronized Subscriptions
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/oak/rgb/image_raw'
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/oak/stereo/image_raw'
        )

        # ApproximateTimeSynchronizer with queue_size=10 and slop=0.1
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.image_callback)

        # Camera Info Subscription
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/oak/rgb/camera_info',
            self.info_callback,
            10
        )

        self.get_logger().info('ObjectLocalizerNode has been started')

    def info_callback(self, msg: CameraInfo) -> None:
        """Callback for camera information to initialize the pinhole camera model.

        Args:
            msg: The CameraInfo message.
        """
        self.camera_model.fromCameraInfo(msg)

    def detect_object(self, cv_image: np.ndarray) -> Optional[Tuple[Tuple[int, int], Tuple[int, int, int, int]]]:
        """Simulates YOLO detection.

        Args:
            cv_image: The OpenCV image.

        Returns:
            A tuple containing the centroid (u, v) and bounding box (x, y, w, h),
            or None if no object is detected.
        """
        # Placeholder for YOLO detection.
        # For simulation, we'll return a dummy detection in the center if needed.
        h, w = cv_image.shape[:2]
        if h > 0 and w > 0:
            centroid = (w // 2, h // 2)
            bbox = (w // 2 - 25, h // 2 - 25, 50, 50)  # x, y, w, h
            return centroid, bbox
        return None

    def get_median_depth(self, depth_image: np.ndarray, bbox: Tuple[int, int, int, int]) -> float:
        """Calculates the median depth value within the bounding box ROI.

        Args:
            depth_image: The depth image (likely in millimeters).
            bbox: Bounding box (x, y, w, h).

        Returns:
            The median depth value in meters.
        """
        x, y, w, h = bbox
        # Ensure ROI is within image boundaries
        img_h, img_w = depth_image.shape[:2]
        x1 = max(0, x)
        y1 = max(0, y)
        x2 = min(img_w, x + w)
        y2 = min(img_h, y + h)

        roi = depth_image[y1:y2, x1:x2]

        # Filter out invalid depth values (0 or NaNs)
        valid_depths = roi[np.isfinite(roi) & (roi > 0)]

        if valid_depths.size == 0:
            return 0.0

        median_depth = np.median(valid_depths)

        # OAK-D Depth is typically uint16 in millimeters. ROS convention is meters.
        if depth_image.dtype == np.uint16:
            return float(median_depth) / 1000.0

        return float(median_depth)

    def image_callback(self, rgb_msg: Image, depth_msg: Image) -> None:
        """Synchronized callback for RGB and Depth images.

        Args:
            rgb_msg: The RGB image message.
            depth_msg: The depth image message.
        """
        if not self.camera_model.initialized():
            self.get_logger().warn('Camera model not initialized yet. Skipping frame.', throttle_duration_sec=5.0)
            return

        try:
            # Convert images to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

            # Detect object
            detection = self.detect_object(cv_image)
            if detection is None:
                return

            centroid, bbox = detection
            u, v = centroid

            # Get median depth in ROI
            depth = self.get_median_depth(depth_image, bbox)

            if depth <= 0:
                self.get_logger().debug('Invalid depth detected')
                return

            # 3D Projection to camera optical frame
            # ray is a (x, y, z) unit vector where z=1
            ray = self.camera_model.projectPixelTo3dRay((u, v))

            # Compute the 3D point in the camera optical frame: P_cam = r * depth
            p_cam = [r * depth for r in ray]

            # Transform to map frame
            self.transform_to_map(p_cam, rgb_msg.header)

        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {str(e)}')

    def transform_to_map(self, p_cam: List[float], header: Header) -> None:
        """Transforms computed point from camera optical frame to map frame and publishes it.

        Args:
            p_cam: 3D point in camera optical frame [x, y, z].
            header: Header from original image message for timestamp and frame_id.
        """
        try:
            # Create a PoseStamped message in the camera frame
            pose_cam = PoseStamped()
            pose_cam.header = header
            pose_cam.pose.position.x = p_cam[0]
            pose_cam.pose.position.y = p_cam[1]
            pose_cam.pose.position.z = p_cam[2]
            pose_cam.pose.orientation.w = 1.0 # Neutral orientation

            # Query the transform at the time specified in the image header
            transform = self.tf_buffer.lookup_transform(
                'map',
                header.frame_id,
                header.stamp,
                timeout=Duration(seconds=0.1)
            )

            # Transform the pose
            pose_map = tf2_geometry_msgs.do_transform_pose(pose_cam, transform)

            # Publish result
            self.pose_pub.publish(pose_map)
            self.get_logger().info(
                f'Human detected and localized at map: '
                f'x={pose_map.pose.position.x:.2f}, '
                f'y={pose_map.pose.position.y:.2f}, '
                f'z={pose_map.pose.position.z:.2f}'
            )

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF2 Exception: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
