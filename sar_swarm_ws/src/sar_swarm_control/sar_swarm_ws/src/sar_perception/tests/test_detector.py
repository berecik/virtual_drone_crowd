import unittest
import numpy as np
from sar_perception.detector import PerceptionNode
from unittest.mock import MagicMock, patch

class TestPerceptionNode(unittest.TestCase):
    def setUp(self):
        # Mock rclpy.init to avoid initializing actual ROS 2
        with patch('rclpy.init'):
            # Mock the node itself to avoid ROS 2 subscription/publisher creation errors
            # We will test the internal logic
            with patch('rclpy.node.Node.create_subscription'), \
                 patch('rclpy.node.Node.create_publisher'), \
                 patch('rclpy.node.Node.get_logger'), \
                 patch('tf2_ros.Buffer'), \
                 patch('tf2_ros.TransformListener'), \
                 patch('ultralytics.YOLO'):
                self.node = PerceptionNode()

    def test_get_depth_at_valid(self):
        # Create a mock depth image (3x3)
        depth_image = np.array([
            [1.0, 1.1, 1.2],
            [1.3, 1.4, 1.5],
            [1.6, 1.7, 1.8]
        ], dtype=np.float32)

        # Center pixel (1, 1) has value 1.4
        # Kernel 3x3 covers all pixels
        depth = self.node.get_depth_at(depth_image, 1, 1, kernel_size=3)
        self.assertAlmostEqual(depth, np.mean(depth_image))

    def test_get_depth_at_with_nan(self):
        depth_image = np.array([
            [1.0, np.nan, 1.2],
            [1.3, 1.4, 1.5],
            [1.6, 1.7, 1.8]
        ], dtype=np.float32)

        depth = self.node.get_depth_at(depth_image, 1, 1, kernel_size=3)
        # Should ignore NaN
        valid_values = [1.0, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8]
        self.assertAlmostEqual(depth, np.mean(valid_values))

    def test_get_depth_at_all_nan(self):
        depth_image = np.array([
            [np.nan, np.nan],
            [np.nan, np.nan]
        ], dtype=np.float32)
        depth = self.node.get_depth_at(depth_image, 0, 0, kernel_size=3)
        self.assertIsNone(depth)

    def test_projection_logic(self):
        # Mock camera info
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.node.camera_info = MagicMock()
        self.node.camera_info.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]

        u, v, z = 320, 240, 1.0 # Center of image
        # X = (u - cx) * z / fx = (320 - 320) * 1.0 / 500 = 0
        # Y = (v - cy) * z / fy = (240 - 240) * 1.0 / 500 = 0

        fx = self.node.camera_info.k[0]
        fy = self.node.camera_info.k[4]
        cx = self.node.camera_info.k[2]
        cy = self.node.camera_info.k[5]

        x_c = (u - cx) * z / fx
        y_c = (v - cy) * z / fy

        self.assertEqual(x_c, 0.0)
        self.assertEqual(y_c, 0.0)

        # Test offset
        u, v, z = 370, 290, 2.0
        # X = (370 - 320) * 2.0 / 500 = 50 * 2 / 500 = 100 / 500 = 0.2
        # Y = (290 - 240) * 2.0 / 500 = 50 * 2 / 500 = 0.2
        x_c = (u - cx) * z / fx
        y_c = (v - cy) * z / fy
        self.assertAlmostEqual(x_c, 0.2)
        self.assertAlmostEqual(y_c, 0.2)

if __name__ == '__main__':
    unittest.main()
