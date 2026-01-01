import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
import numpy as np
import os
import time

class SwarmVisualizer(Node):
    def __init__(self):
        super().__init__('swarm_visualizer')
        self.drone_positions = {}
        self.goal = None

        # Subscribe to any drone odom
        self.create_subscription(VehicleOdometry, '/drone_1/fmu/out/vehicle_odometry', lambda msg: self.odom_cb(msg, 1), 10)
        self.create_subscription(VehicleOdometry, '/drone_2/fmu/out/vehicle_odometry', lambda msg: self.odom_cb(msg, 2), 10)
        self.create_subscription(VehicleOdometry, '/swarm/goal', self.goal_cb, 10)

        self.width = 40
        self.height = 20
        self.scale = 2.0 # meters per character

        self.timer = self.create_timer(0.5, self.draw)
        print("ASCII Visualizer Started")

    def odom_cb(self, msg, drone_id):
        self.drone_positions[drone_id] = np.array(msg.position)

    def goal_cb(self, msg):
        self.goal = np.array(msg.position)

    def draw(self):
        if not self.drone_positions and not self.goal:
            return

        # Clear screen (approximate for many terminals)
        # os.system('clear') # Might be too slow or flickering
        print("\033[H\033[J", end="") # ANSI escape code to clear screen

        grid = [[' ' for _ in range(self.width)] for _ in range(self.height)]

        # Helper to convert world coords to grid coords
        def world_to_grid(pos):
            x, y = pos[0], pos[1]
            gx = int(x / self.scale) + self.width // 4
            gy = int(y / self.scale) + self.height // 2
            return gx, gy

        # Draw Goal
        if self.goal is not None:
            gx, gy = world_to_grid(self.goal)
            if 0 <= gx < self.width and 0 <= gy < self.height:
                grid[gy][gx] = 'X'

        # Draw Drones
        for did, pos in self.drone_positions.items():
            gx, gy = world_to_grid(pos)
            if 0 <= gx < self.width and 0 <= gy < self.height:
                grid[gy][gx] = str(did)

        # Print Grid
        print("-" * (self.width + 2))
        for row in grid:
            print("|" + "".join(row) + "|")
        print("-" * (self.width + 2))

        if self.drone_positions:
            for did, pos in self.drone_positions.items():
                print(f"Drone {did}: [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}]")
        if self.goal is not None:
            print(f"Goal: [{self.goal[0]:.1f}, {self.goal[1]:.1f}, {self.goal[2]:.1f}]")

def main():
    rclpy.init()
    node = SwarmVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
