import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
import numpy as np

class SearchPlannerNode(Node):
    def __init__(self):
        super().__init__('search_planner_node')

        # Parameters
        self.declare_parameter('area_length', 20.0)
        self.declare_parameter('area_width', 20.0)
        self.declare_parameter('sweep_width', 4.0)
        self.declare_parameter('altitude', 5.0)
        self.declare_parameter('dist_threshold', 1.0)

        self.area_length = self.get_parameter('area_length').value
        self.area_width = self.get_parameter('area_width').value
        self.sweep_width = self.get_parameter('sweep_width').value
        self.altitude = self.get_parameter('altitude').value
        self.dist_threshold = self.get_parameter('dist_threshold').value

        # Publishers
        self.target_pub = self.create_publisher(
            Point,
            '/sar/mission/target',
            10)

        # Subscriptions - to track current drone pose
        # Assuming the controller or a state estimator publishes drone pose here
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/sar/drone_pose',
            self.pose_callback,
            10)

        self.waypoints = self.generate_lawnmower_path()
        self.current_wp_idx = 0

        self.get_logger().info(f"Search Planner Initialized. Waypoints: {len(self.waypoints)}")

        # Timer to publish target periodically
        self.timer = self.create_timer(1.0, self.publish_target)

    def generate_lawnmower_path(self):
        waypoints = []
        x_steps = int(self.area_length / self.sweep_width)

        for i in range(x_steps + 1):
            x = i * self.sweep_width
            # Boustrophedon pattern
            if i % 2 == 0:
                waypoints.append(Point(x=float(x), y=0.0, z=float(self.altitude)))
                waypoints.append(Point(x=float(x), y=float(self.area_width), z=float(self.altitude)))
            else:
                waypoints.append(Point(x=float(x), y=float(self.area_width), z=float(self.altitude)))
                waypoints.append(Point(x=float(x), y=0.0, z=float(self.altitude)))

        return waypoints

    def pose_callback(self, msg):
        if self.current_wp_idx >= len(self.waypoints):
            return

        current_pos = msg.pose.position
        target_wp = self.waypoints[self.current_wp_idx]

        # Calculate distance to target
        dist = np.sqrt(
            (current_pos.x - target_wp.x)**2 +
            (current_pos.y - target_wp.y)**2 +
            (current_pos.z - target_wp.z)**2
        )

        if dist < self.dist_threshold:
            self.get_logger().info(f"Reached waypoint {self.current_wp_idx} (dist: {dist:.2f}m). Switching to next.")
            self.current_wp_idx += 1
            self.publish_target()

    def publish_target(self):
        if self.current_wp_idx < len(self.waypoints):
            self.target_pub.publish(self.waypoints[self.current_wp_idx])
        else:
            self.get_logger().info("Search mission complete!", once=True)

def main(args=None):
    rclpy.init(args=args)
    node = SearchPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
