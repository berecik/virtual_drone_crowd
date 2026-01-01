import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
import numpy as np
import time

class LawnmowerSearchTask(Node):
    def __init__(self):
        super().__init__('search_task')
        self.goal_pub = self.create_publisher(VehicleOdometry, '/swarm/goal', 10)

        # Search area: 0 to 20 in X, 0 to 20 in Y
        self.x_min, self.x_max = 0.0, 20.0
        self.y_min, self.y_max = 0.0, 20.0
        self.step = 4.0

    def generate_waypoints(self):
        waypoints = []
        for y in np.arange(self.y_min, self.y_max + self.step, self.step):
            if (y / self.step) % 2 == 0:
                # Left to right
                waypoints.append([self.x_min, y, 0.0])
                waypoints.append([self.x_max, y, 0.0])
            else:
                # Right to left
                waypoints.append([self.x_max, y, 0.0])
                waypoints.append([self.x_min, y, 0.0])
        return waypoints

    def run(self):
        waypoints = self.generate_waypoints()
        self.get_logger().info(f"Starting Lawnmower Search with {len(waypoints)} waypoints")

        for wp in waypoints:
            self.get_logger().info(f"Moving to: {wp}")
            msg = VehicleOdometry()
            msg.position = wp
            for _ in range(10): # Publish multiple times to ensure receipt
                self.goal_pub.publish(msg)
                time.sleep(0.1)

            # Simple wait for drones to reach the waypoint (in a real scenario, we'd check feedback)
            time.sleep(8)

        self.get_logger().info("Search Task Completed")

def main():
    rclpy.init()
    node = LawnmowerSearchTask()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
