import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry, TrajectorySetpoint
import numpy as np
import sys

class SwarmController(Node):
    def __init__(self, drone_id, target_id=None):
        super().__init__(f'swarm_controller_{drone_id}')
        self.drone_id = drone_id
        self.target_id = target_id

        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.leader_pos = np.array([0.0, 0.0, 0.0])

        # Default offsets for drones
        offsets = {
            2: np.array([2.0, 0.0, 0.0]),
            3: np.array([0.0, 2.0, 0.0]),
            4: np.array([2.0, 2.0, 0.0])
        }
        self.offset = offsets.get(drone_id, np.array([0.0, 0.0, 0.0])) if target_id else np.array([0.0, 0.0, 0.0])

        # Publishers
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            f'/drone_{drone_id}/fmu/in/trajectory_setpoint',
            10
        )

        # Subscriptions
        self.create_subscription(
            VehicleOdometry,
            f'/drone_{drone_id}/fmu/out/vehicle_odometry',
            self.odom_callback,
            10
        )

        if self.target_id:
            self.create_subscription(
                VehicleOdometry,
                f'/drone_{target_id}/fmu/out/vehicle_odometry',
                self.leader_odom_callback,
                10
            )
        else:
            # If no target, follow a global goal
            self.create_subscription(
                VehicleOdometry,
                '/swarm/goal',
                self.leader_odom_callback,
                10
            )

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(f"Swarm Controller for drone {drone_id} started. Target: {target_id}")

    def odom_callback(self, msg):
        self.current_pos = np.array(msg.position)

    def leader_odom_callback(self, msg):
        self.leader_pos = np.array(msg.position)

    def timer_callback(self):
        target = self.leader_pos + self.offset

        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = target.astype(np.float32).tolist()
        msg.velocity = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.jerk = [float('nan')] * 3
        msg.yaw = 0.0
        msg.yawspeed = float('nan')

        self.setpoint_pub.publish(msg)

def main():
    rclpy.init()

    drone_id = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    target_id = int(sys.argv[2]) if len(sys.argv) > 2 else None

    node = SwarmController(drone_id, target_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
