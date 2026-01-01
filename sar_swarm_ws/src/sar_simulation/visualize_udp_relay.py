import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
import socket
import json
import time

class UDPRelay(Node):
    def __init__(self, host='10.10.1.1', port=5005):
        super().__init__('visualize_udp_relay')
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.drone_positions = {}
        self.goal_position = None

        # Subscriptions
        self.create_subscription(VehicleOdometry, '/drone_1/fmu/out/vehicle_odometry',
                                 lambda msg: self.odom_callback(1, msg), 10)
        self.create_subscription(VehicleOdometry, '/drone_2/fmu/out/vehicle_odometry',
                                 lambda msg: self.odom_callback(2, msg), 10)
        self.create_subscription(VehicleOdometry, '/drone_3/fmu/out/vehicle_odometry',
                                 lambda msg: self.odom_callback(3, msg), 10)
        self.create_subscription(VehicleOdometry, '/drone_4/fmu/out/vehicle_odometry',
                                 lambda msg: self.odom_callback(4, msg), 10)
        self.create_subscription(VehicleOdometry, '/swarm/goal', self.goal_callback, 10)

        # Timer to send data to host
        self.create_timer(0.1, self.send_data)
        self.get_logger().info(f"UDP Relay started, sending to {host}:{port}")

    def odom_callback(self, drone_id, msg):
        self.drone_positions[drone_id] = [float(msg.position[0]), float(msg.position[1])]

    def goal_callback(self, msg):
        self.goal_position = [float(msg.position[0]), float(msg.position[1])]

    def send_data(self):
        data = {
            "drones": self.drone_positions,
            "goal": self.goal_position
        }
        try:
            self.sock.sendto(json.dumps(data).encode(), (self.host, self.port))
        except Exception as e:
            self.get_logger().error(f"Failed to send UDP: {e}")

def main():
    rclpy.init()
    node = UDPRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
