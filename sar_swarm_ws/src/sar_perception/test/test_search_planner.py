import sys
from unittest.mock import MagicMock

# Robust mocking for ROS 2 and other dependencies
class Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

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

mock_rclpy = MagicMock()
mock_rclpy.node.Node = MockNode
sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.node'] = mock_rclpy.node

mock_gm = MagicMock()
mock_gm.Point = Point
sys.modules['geometry_msgs.msg'] = mock_gm

from sar_perception.search_planner import SearchPlannerNode
import unittest

class TestSearchPlanner(unittest.TestCase):
    def test_generate_lawnmower_path(self):
        node = SearchPlannerNode()
        # Default: length=20, width=20, sweep=4
        # x_steps = 20/4 = 5
        # i in range(6): 0, 1, 2, 3, 4, 5
        # points per x: 2
        # total points = 12
        self.assertEqual(len(node.waypoints), 12)

        # Check first few points
        self.assertEqual(node.waypoints[0].x, 0.0)
        self.assertEqual(node.waypoints[0].y, 0.0)
        self.assertEqual(node.waypoints[1].x, 0.0)
        self.assertEqual(node.waypoints[1].y, 20.0)
        self.assertEqual(node.waypoints[2].x, 4.0)
        self.assertEqual(node.waypoints[2].y, 20.0) # Boustrophedon
        self.assertEqual(node.waypoints[3].x, 4.0)
        self.assertEqual(node.waypoints[3].y, 0.0)

    def test_pose_callback_advances_waypoint(self):
        node = SearchPlannerNode()
        node.target_pub = MagicMock()

        self.assertEqual(node.current_wp_idx, 0)
        target_wp = node.waypoints[0]

        # Create pose message at target
        msg = MagicMock()
        msg.pose.position.x = float(target_wp.x)
        msg.pose.position.y = float(target_wp.y)
        msg.pose.position.z = float(target_wp.z)

        node.pose_callback(msg)

        self.assertEqual(node.current_wp_idx, 1)
        self.assertTrue(node.target_pub.publish.called)

    def test_pose_callback_ignores_far_pose(self):
        node = SearchPlannerNode()
        node.current_wp_idx = 0

        # Waypoint 0 is at (0,0,5)
        msg = MagicMock()
        msg.pose.position.x = 10.0
        msg.pose.position.y = 10.0
        msg.pose.position.z = 5.0

        node.pose_callback(msg)

        self.assertEqual(node.current_wp_idx, 0)

    def test_publish_target_at_end(self):
        node = SearchPlannerNode()
        node.current_wp_idx = len(node.waypoints)
        node.target_pub = MagicMock()

        node.publish_target()

        self.assertFalse(node.target_pub.publish.called)

if __name__ == '__main__':
    unittest.main()
