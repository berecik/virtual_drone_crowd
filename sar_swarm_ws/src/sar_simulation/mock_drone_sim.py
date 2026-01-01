import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry, TrajectorySetpoint
import numpy as np

class MockDroneSim(Node):
    def __init__(self):
        super().__init__('mock_drone_sim')
        self.declare_parameter('drone_ids', [1, 2, 3, 4])
        drone_ids = self.get_parameter('drone_ids').value

        self.drones = {}
        for d_id in drone_ids:
            self.drones[d_id] = {
                'pos': np.array([0.0, 0.0, 0.0], dtype=np.float32),
                'vel': np.array([0.0, 0.0, 0.0], dtype=np.float32),
                'target_pos': None,
                'pub': self.create_publisher(VehicleOdometry, f'/drone_{d_id}/fmu/out/vehicle_odometry', 10),
            }
            # Use a lambda with default argument to capture d_id
            self.create_subscription(
                TrajectorySetpoint,
                f'/drone_{d_id}/fmu/in/trajectory_setpoint',
                lambda msg, id=d_id: self.setpoint_callback(msg, id),
                10
            )

        self.timer = self.create_timer(0.1, self.update_physics)
        self.get_logger().info(f"Mock Drone Sim started for drones: {drone_ids}")

    def setpoint_callback(self, msg, d_id):
        # We only care about position for this simple task
        if not np.isnan(msg.position).any():
            self.drones[d_id]['target_pos'] = np.array(msg.position, dtype=np.float32)

    def update_physics(self):
        dt = 0.1
        for d_id, data in self.drones.items():
            if data['target_pos'] is not None:
                # Simple P control towards target_pos
                diff = data['target_pos'] - data['pos']
                dist = np.linalg.norm(diff)
                if dist > 0.01:
                    dir = diff / dist
                    speed = min(dist, 2.0) # max 2m/s
                    data['pos'] += dir * speed * dt

            # Publish odometry
            odom = VehicleOdometry()
            odom.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            odom.position = data['pos'].tolist()
            data['pub'].publish(odom)

def main():
    rclpy.init()
    node = MockDroneSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
