import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
import numpy as np
import time

class SwarmTest(Node):
    def __init__(self):
        super().__init__('swarm_test')
        self.drone_1_pos = None
        self.drone_2_pos = None

        self.goal_pub = self.create_publisher(VehicleOdometry, '/swarm/goal', 10)

        self.create_subscription(VehicleOdometry, '/drone_1/fmu/out/vehicle_odometry', self.drone_1_cb, 10)
        self.create_subscription(VehicleOdometry, '/drone_2/fmu/out/vehicle_odometry', self.drone_2_cb, 10)

        self.get_logger().info("Swarm Test Node started")

    def drone_1_cb(self, msg):
        self.drone_1_pos = np.array(msg.position)

    def drone_2_cb(self, msg):
        self.drone_2_pos = np.array(msg.position)

def main():
    rclpy.init()
    node = SwarmTest()

    # 1. Send Goal
    goal = VehicleOdometry()
    goal.position = [10.0, 10.0, 0.0]

    print("Sending goal: [10, 10, 0]")
    for _ in range(5):
        node.goal_pub.publish(goal)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    # 2. Wait for drones to move
    start_time = time.time()
    success = False
    while time.time() - start_time < 20: # 20 seconds timeout
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.drone_1_pos is not None and node.drone_2_pos is not None:
            dist_to_goal = np.linalg.norm(node.drone_1_pos - np.array([10.0, 10.0, 0.0]))
            rel_dist = np.linalg.norm(node.drone_1_pos - node.drone_2_pos)

            print(f"Drone 1 pos: {node.drone_1_pos}, Dist to goal: {dist_to_goal:.2f}, Rel dist: {rel_dist:.2f}")

            # Check if both reached (approx) and maintained offset (approx 2.0)
            if dist_to_goal < 1.0 and abs(rel_dist - 2.0) < 0.5:
                success = True
                print("Drones reached goal together!")
                break
        else:
            print("Waiting for drone positions...")

    if success:
        print("TEST PASSED")
    else:
        print("TEST FAILED")

    node.destroy_node()
    rclpy.shutdown()
    if not success:
        exit(1)

if __name__ == '__main__':
    main()
