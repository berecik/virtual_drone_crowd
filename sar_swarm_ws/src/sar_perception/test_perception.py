"""
Virtual Drone Crowd - Perception Test Placeholder
Author: beret <beret@hipisi.org.pl>
Company: Marysia Software Limited <ceo@marysia.app>
Domain: app.marysia.drone
Website: https://marysia.app
"""

import numpy as np
import pytest
import sys
import os

# Add the package to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'sar_perception'))

# Mock rclpy and other ROS stuff if they are missing
from unittest.mock import MagicMock
sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
sys.modules['geometry_msgs'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()
sys.modules['cv_bridge'] = MagicMock()

from detector import DetectorNode

def test_process_detection_target_found():
    node = DetectorNode()
    # Create a dummy depth image (100x100) with 1.5m at center
    depth_image = np.ones((100, 100), dtype=np.float32) * 5.0
    depth_image[50, 50] = 1.5

    result = node.process_detection(depth_image)
    assert result == 1.5

def test_process_detection_no_target():
    node = DetectorNode()
    # Create a dummy depth image with 3.0m at center
    depth_image = np.ones((100, 100), dtype=np.float32) * 3.0

    result = node.process_detection(depth_image)
    assert result is None

def test_process_detection_none_input():
    node = DetectorNode()
    result = node.process_detection(None)
    assert result is None
