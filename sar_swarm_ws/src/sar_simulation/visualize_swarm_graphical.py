import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import numpy as np

class GraphicalVisualizer(Node):
    def __init__(self):
        super().__init__('graphical_visualizer')
        self.drone_positions = {}
        self.goal_position = None
        self.lock = threading.Lock()

        # Subscriptions
        self.create_subscription(VehicleOdometry, '/drone_1/fmu/out/vehicle_odometry',
                                 lambda msg: self.odom_callback(1, msg), 10)
        self.create_subscription(VehicleOdometry, '/drone_2/fmu/out/vehicle_odometry',
                                 lambda msg: self.odom_callback(2, msg), 10)
        self.create_subscription(VehicleOdometry, '/swarm/goal', self.goal_callback, 10)

        self.get_logger().info("Graphical Visualizer Node started")

    def odom_callback(self, drone_id, msg):
        with self.lock:
            self.drone_positions[drone_id] = (msg.position[0], msg.position[1])

    def goal_callback(self, msg):
        with self.lock:
            self.goal_position = (msg.position[0], msg.position[1])

def main():
    rclpy.init()
    node = GraphicalVisualizer()

    # Spin ROS 2 in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    # Set up Matplotlib
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-5, 15)
    ax.set_ylim(-5, 15)
    ax.set_title("Swarm Flight Visualization")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(True)

    drone_plots = {}
    goal_plot, = ax.plot([], [], 'rx', markersize=10, label='Goal')

    colors = {1: 'blue', 2: 'green'}

    def init():
        goal_plot.set_data([], [])
        return goal_plot,

    def update(frame):
        with node.lock:
            # Update Drones
            for drone_id, pos in node.drone_positions.items():
                if drone_id not in drone_plots:
                    drone_plots[drone_id], = ax.plot([], [], 'o', color=colors.get(drone_id, 'black'), label=f'Drone {drone_id}')
                    ax.legend()
                drone_plots[drone_id].set_data([pos[0]], [pos[1]])

            # Update Goal
            if node.goal_position:
                goal_plot.set_data([node.goal_position[0]], [node.goal_position[1]])

        return list(drone_plots.values()) + [goal_plot]

    ani = FuncAnimation(fig, update, frames=None, init_func=init, blit=True, interval=100)

    plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
