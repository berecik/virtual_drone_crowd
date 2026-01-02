# Testing Strategy for `sar_swarm_control`

This document describes the testing approach for the Rust ROS 2 node that manages the PX4 Offboard Control Loop.

## 1. Unit Testing (Core Logic)

Core mathematical and utility functions are separated into `src/utils.rs` and `src/boids.rs` to allow testing without a full ROS 2 environment.

### Key Tests (Detailed Catalog):

#### A. Coordinate Systems & Math
- **`test_enu_to_ned_conversion`**:
    - **Purpose**: Validates the translation between ROS 2 ENU (East-North-Up) and PX4 NED (North-East-Down) frames.
    - **Input**: `[10.0, 5.0, 2.0]` (ENU).
    - **Expected Outcome**: `[5.0, 10.0, -2.0]` (NED). Correctly handles axis swapping and Z-axis inversion.
- **`test_timestamp_generation`**:
    - **Purpose**: Ensures that the `get_clock_microseconds` helper provides monotonically increasing timestamps in microseconds, as required by PX4's uORB middleware.
    - **Verification**: Asserts that two sequential calls with a delay return increasing values with appropriate deltas.

#### B. Swarm Intelligence (Boids Algorithm)
- **`test_separation_force`**:
    - **Purpose**: Verifies that drones generate a repulsive force when too close to neighbors (< 1.0m).
    - **Input**: Two Boids separated by 0.1m.
    - **Expected Outcome**: A force vector pointing directly away from the neighbor.
- **`test_alignment_force`**:
    - **Purpose**: Checks if a drone attempts to match the velocity of its neighbors within the sensing radius (5.0m).
    - **Input**: One stationary Boid and one neighbor moving at `[1.0, 1.0, 0.0]`.
    - **Expected Outcome**: A force vector contributing to velocity matching.
- **`test_cohesion_force`**:
    - **Purpose**: Ensures drones are attracted to the center of mass of their neighbors.
    - **Input**: Neighbors within the 5.0m radius.
    - **Expected Outcome**: A force vector pointing towards the average neighbor position.
- **`test_boids_edge_cases`**:
    - **Purpose**: Stress tests the flocking logic against boundary conditions.
    - **Scenarios**: Neighbors at exactly 5.0m (excluded), identical positions (zero force to avoid division by zero), and neighbors just inside/outside the radius.

#### C. Communication & Middleware
- **`test_boid_serialization`**:
    - **Purpose**: Validates the `bincode` encoding/decoding of the `Boid` struct.
    - **Requirement**: Essential for reliable Zenoh-based state sharing between drones.
    - **Outcome**: Decoded struct must exactly match the original.
- **`test_handshake_logic`**:
    - **Purpose**: Verifies the internal state machine for the PX4 Offboard handshake (Heartbeat -> Mode Switch -> Arming).
    - **Verification**: Simulates 20 cycles and confirms that the mode switch and arming commands are triggered exactly at cycle 10.
- **`test_lifecycle_transitions`**:
    - **Purpose**: Verifies the `OffboardControlNode` transitions (Unconfigured -> Inactive -> Active).
    - **Verification**: Ensures Publishers/Subscribers are created/destroyed correctly during transitions.
- **`test_qos_profiles`**:
    - **Purpose**: Validates that telemetry and command topics use the mandated specialized QoS profiles (BestEffort for telemetry, Reliable for commands).

### Running Unit Tests:
Since the project depends on `rclrs`, a sourced ROS 2 environment (Jazzy) is required even for `cargo test`.
```bash
# Inside a ROS 2 environment
cd sar_swarm_ws/src/sar_swarm_control
cargo test
```

## 2. Integration Testing (Node Logic)

The `OffboardControlNode` in `src/offboard_control_node.rs` implements a state machine for the Offboard handshake:
1. **Unconfigured -> Inactive:** Initialization and Publisher/Subscriber setup.
2. **Inactive -> Active:** Enables the 10Hz heartbeat loop.
3. **Active Control:** Stream `TrajectorySetpoint` (NED) and `OffboardControlMode` (Heartbeat).
4. **Safety Verification:** Node waits for a valid GPS fix (simulated or real) before enabling offboard mode.

### Verification (Manual/SITL):
Full integration testing should be performed using the PX4 SITL (Software-in-the-Loop) environment.
1. Start PX4 SITL with Gazebo.
2. Start the XRCE-DDS Agent:
   ```bash
   MicroXRCEAgent udp4 -p 8888
   ```
3. Run the swarm node:
   ```bash
   ros2 run sar_swarm_control swarm_node
   ```
4. Observe the drone in Gazebo switching to Offboard mode, arming, and hovering at 5m.

## 3. Mocking Strategy

The `px4_msgs_mock` package is used during development to provide the necessary message definitions without requiring the full `px4_msgs` ROS package to be compiled, which accelerates the development cycle.
