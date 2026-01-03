import sys
from unittest.mock import MagicMock, patch
import unittest
import numpy as np

# Robust mocking for ROS 2 and other dependencies
class MockNode:
    def __init__(self, *args, **kwargs):
        self._parameters = {}
    def create_subscription(self, *args, **kwargs): return MagicMock()
    def create_publisher(self, *args, **kwargs): return MagicMock()
    def create_timer(self, *args, **kwargs): return MagicMock()
    def get_logger(self): return MagicMock()
    def declare_parameter(self, name, value):
        self._parameters[name] = value
        return MagicMock()
    def get_parameter(self, name):
        m = MagicMock()
        m.get_parameter_value.return_value.string_value = self._parameters.get(name, "map")
        return m
    def destroy_node(self): pass

class LookupException(Exception): pass
class ConnectivityException(Exception): pass
class ExtrapolationException(Exception): pass

# Mocking all required modules
mock_rclpy = MagicMock()
mock_rclpy.node.Node = MockNode
mock_rclpy.qos = MagicMock()
mock_rclpy.duration.Duration = MagicMock()
sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.node'] = mock_rclpy.node
sys.modules['rclpy.qos'] = mock_rclpy.qos
sys.modules['rclpy.duration'] = mock_rclpy.duration
sys.modules['sensor_msgs.msg'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['cv_bridge'] = MagicMock()
sys.modules['message_filters'] = MagicMock()
sys.modules['image_geometry'] = MagicMock()

mock_tf2_ros = MagicMock()
mock_tf2_ros.LookupException = LookupException
mock_tf2_ros.ConnectivityException = ConnectivityException
mock_tf2_ros.ExtrapolationException = ExtrapolationException
sys.modules['tf2_ros'] = mock_tf2_ros
sys.modules['tf2_geometry_msgs'] = MagicMock()

# Mock cv2
sys.modules['cv2'] = MagicMock()

from sar_perception.object_localizer import ObjectLocalizerNode

class TestObjectLocalizer(unittest.TestCase):
    def setUp(self):
        self.node = ObjectLocalizerNode()

    def test_get_median_depth(self):
        # Create a 10x10 depth image (uint16, millimeters)
        depth_image = np.ones((10, 10), dtype=np.uint16) * 2000 # 2 meters
        # ROI from (2, 2) to (7, 7)
        bbox = (2, 2, 5, 5)

        # Inject some noise
        depth_image[3, 3] = 0
        depth_image[4, 4] = 10000

        depth = self.node.get_median_depth(depth_image, bbox)
        # Median of 2000, 2000, ..., 10000 (0 is excluded) should be 2.0 meters
        self.assertEqual(depth, 2.0)

    def test_get_median_depth_float(self):
        # Create a 10x10 depth image (float32, meters)
        depth_image = np.ones((10, 10), dtype=np.float32) * 1.5
        bbox = (0, 0, 10, 10)
        depth = self.node.get_median_depth(depth_image, bbox)
        self.assertEqual(depth, 1.5)

    def test_detect_object(self):
        cv_image = np.zeros((480, 640, 3), dtype=np.uint8)
        detection = self.node.detect_object(cv_image)
        self.assertIsNotNone(detection)
        centroid, bbox = detection
        self.assertEqual(centroid, (320, 240))
        self.assertEqual(bbox, (320 - 25, 240 - 25, 50, 50))

    def test_image_callback_no_info(self):
        self.node.camera_model.initialized.return_value = False
        self.node.image_callback(MagicMock(), MagicMock())
        self.assertFalse(self.node.bridge.imgmsg_to_cv2.called)

    @patch('tf2_geometry_msgs.do_transform_pose')
    def test_transform_to_map_success(self, mock_do_transform):
        # Mock setup
        mock_pose_map = MagicMock()
        mock_pose_map.pose.position.x = 10.0
        mock_pose_map.pose.position.y = 20.0
        mock_pose_map.pose.position.z = 5.0
        mock_do_transform.return_value = mock_pose_map

        self.node.tf_buffer.lookup_transform = MagicMock()
        self.node.pose_pub = MagicMock()

        p_cam = [1.0, 2.0, 3.0]
        header = MagicMock()
        header.frame_id = "camera_optical_frame"
        header.stamp = MagicMock()

        self.node.transform_to_map(p_cam, header)

        self.assertTrue(self.node.pose_pub.publish.called)
        published_msg = self.node.pose_pub.publish.call_args[0][0]
        self.assertEqual(published_msg.pose.position.x, 10.0)

if __name__ == '__main__':
    unittest.main()
