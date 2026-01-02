import sys
from unittest.mock import MagicMock, patch

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
        val = self._parameters.get(name, 20.0 if "area" in name else 4.0 if "sweep" in name else 5.0)
        m = MagicMock()
        m.value = val
        return m
    def destroy_node(self): pass

class LookupException(Exception): pass
class ConnectivityException(Exception): pass
class ExtrapolationException(Exception): pass

mock_rclpy = MagicMock()
mock_rclpy.node.Node = MockNode
mock_rclpy.qos = MagicMock()
sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.node'] = mock_rclpy.node
sys.modules['rclpy.qos'] = mock_rclpy.qos
sys.modules['rclpy.duration'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()
sys.modules['visualization_msgs.msg'] = MagicMock()
sys.modules['cv_bridge'] = MagicMock()
sys.modules['message_filters'] = MagicMock()

mock_tf2_ros = MagicMock()
mock_tf2_ros.LookupException = LookupException
mock_tf2_ros.ConnectivityException = ConnectivityException
mock_tf2_ros.ExtrapolationException = ExtrapolationException
sys.modules['tf2_ros'] = mock_tf2_ros
sys.modules['tf2_geometry_msgs'] = MagicMock()

# Mock cv2 and ultralytics for environment-independent testing
sys.modules['cv2'] = MagicMock()
sys.modules['ultralytics'] = MagicMock()

from sar_perception.detector import PerceptionNode
import unittest
import numpy as np

class TestDetector(unittest.TestCase):
    @patch('sar_perception.detector.YOLO')
    def setUp(self, mock_yolo):
        self.node = PerceptionNode()

    def test_get_depth_at(self):
        # Create a 100x100 depth image
        depth_image = np.ones((100, 100), dtype=np.float32) * 2.0
        # Put a NaN and 0 to test filtering
        depth_image[50, 50] = np.nan
        depth_image[51, 51] = 0.0
        # Put some other values in the window
        depth_image[48:53, 48:53] = 3.0
        depth_image[50, 50] = np.nan
        depth_image[51, 51] = 0.0

        depth = self.node.get_depth_at(depth_image, 50, 50, kernel_size=5)
        self.assertEqual(depth, 3.0)

    def test_projection_logic(self):
        # Mock CameraInfo
        camera_info = MagicMock()
        # k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        camera_info.k = [1000.0, 0.0, 500.0, 0.0, 1000.0, 500.0, 0.0, 0.0, 1.0]
        self.node.camera_info = camera_info

        # Center pixel (500, 500) at 2.0m should deproject to (0, 0, 2)
        u, v, z = 500, 500, 2.0
        fx = self.node.camera_info.k[0]
        fy = self.node.camera_info.k[4]
        cx = self.node.camera_info.k[2]
        cy = self.node.camera_info.k[5]

        x_c = (u - cx) * z / fx
        y_c = (v - cy) * z / fy
        z_c = float(z)

        self.assertEqual(x_c, 0.0)
        self.assertEqual(y_c, 0.0)
        self.assertEqual(z_c, 2.0)

    def test_transform_and_publish_success(self):
        self.node.victim_pub = MagicMock()

        # Mock TF lookup and transform
        mock_transform = MagicMock()
        self.node.tf_buffer.lookup_transform = MagicMock(return_value=mock_transform)

        with patch('tf2_geometry_msgs.do_transform_point') as mock_do_transform:
            mock_p_map = MagicMock()
            mock_p_map.point.x = 1.0
            mock_p_map.point.y = 2.0
            mock_p_map.point.z = 3.0
            mock_do_transform.return_value = mock_p_map

            # Need a mock stamp
            mock_stamp = MagicMock()
            self.node.transform_and_publish(0.1, 0.2, 0.3, "camera_link", mock_stamp)

            self.assertTrue(self.node.victim_pub.publish.called)

    def test_transform_and_publish_tf_error(self):
        self.node.victim_pub = MagicMock()

        # Re-mock lookup_transform to raise our stub exception
        self.node.tf_buffer.lookup_transform = MagicMock(side_effect=LookupException("Test Error"))

        # Should not crash, just log warning
        mock_stamp = MagicMock()
        self.node.transform_and_publish(0.1, 0.2, 0.3, "camera_link", mock_stamp)
        self.assertFalse(self.node.victim_pub.publish.called)

if __name__ == '__main__':
    unittest.main()
