# TESTING.md - Swarm System Test Status

This document tracks the high-level testing status and provides detailed explanations of the verification suite across the Virtual Drone Crowd project.

## 🧪 Current Status (2026-01-14)

| Module | Unit Tests | Integration Tests | SITL / Hardware | Status |
| :--- | :---: | :---: | :---: | :--- |
| `sar_swarm_control` (Rust) | ✅ Pass (17)* | ⏳ Pending | ✅ Pass (Sim) | Boids & Mission FSM Verified. |
| `sar_perception` (Python) | ✅ Pass (13) | ⏳ Pending | ✅ Pass (Sim) | 3D Localization & Lawnmower Verified |
| `heavy_lift_core` (Rust) | ✅ Pass (1) | ⏳ Pending | ⏳ Pending | Extraction State Machine Verified |
| **Swarm Simulation** | - | ✅ Pass (3) | ✅ Pass (Sim) | Mock Drone Flight Logic Verified |

\* *Note: Rust tests for `sar_swarm_control` require a sourced ROS 2 environment for compilation due to `rclrs` dependency.*

## 📂 Detailed Test Catalog

### 1. `sar_swarm_control` (Rust Core)

The following tests verify the mission-critical flight logic and swarm coordination:

#### A. Coordinate Systems & Math
- **`test_enu_to_ned_conversion`**:
    - **Purpose**: Validates the translation between ROS 2 ENU (East-North-Up) and PX4 NED (North-East-Down) frames.
- **`test_calculate_target_pos`**:
    - **Purpose**: Verifies leader-offset translation for swarm formation.
- **`test_timestamp_generation`**:
    - **Purpose**: Ensures monotonically increasing timestamps in microseconds.

#### B. Swarm Intelligence & Search
- **`test_separation_force`**, **`test_alignment_force`**, **`test_cohesion_force`**: Core flocking behaviors.
- **`test_boids_edge_cases`**: Neighbors at exactly 5.0m, identical positions, etc.
- **`test_boids_many_neighbors`**: Stress test with 100 neighbors.
- **`test_boids_empty_neighbors`**: Verification of zero-neighbor behavior.
- **`test_lawnmower_pattern`**:
    - **Purpose**: Verifies the generation of waypoints for a lawnmower search pattern.
    - **Verification**: Checks waypoint count and specific coordinates for alternating passes.

#### C. Communication & Middleware
- **`test_boid_serialization`**: Validates `bincode` encoding/decoding.
- **`test_process_raw_data`**:
    - **Purpose**: Verifies inter-drone message processing logic without a live Zenoh session.
    - **Verification**: Tests valid messages, messages from self (ignored), and malformed payloads.
- **`test_handshake_logic`**: Verifies PX4 Offboard handshake state machine.

### 2. `sar_perception` (Python AI)
The following tests verify the AI-driven human detection and 3D localization logic:

#### A. Detector Logic (`test_detector.py`)
- **`test_get_depth_at`**:
    - **Purpose**: Verifies robust depth sampling with window averaging and NaN/zero filtering.
- **`test_deproject`**:
    - **Purpose**: Validates the Pinhole Camera Model math for pixel-to-3D conversion.
- **`test_transform_and_publish_success`**:
    - **Purpose**: Ensures detection points are correctly transformed to the `map` frame and published.
- **`test_transform_and_publish_tf_error`**:
    - **Purpose**: Verifies graceful handling of TF2 lookup exceptions.

#### B. Search Planning (`test_search_planner.py`)
- **`test_generate_lawnmower_path`**:
    - **Purpose**: Validates the Boustrophedon waypoint generation logic.
- **`test_pose_callback_advances_waypoint`**:
    - **Purpose**: Ensures the mission advances to the next waypoint upon reaching the current target.
- **`test_pose_callback_ignores_far_pose`**:
    - **Purpose**: Verifies that waypoints are NOT advanced when the drone is too far.
- **`test_publish_target_at_end`**:
    - **Purpose**: Checks behavior when the search mission is complete.

### 3. `heavy_lift_core` (Rust Heavy Lift)
- **`test_state_transitions`**:
    - **Purpose**: Verifies the extraction sequence state machine (IDLE -> EN_ROUTE -> DESCENDING -> LIFTING -> RETURN -> IDLE).
    - **Verification**: Validates correctness of the `transition()` function and `next_state()` logic.

### 4. `sar_simulation` (Python Swarm Sim)
- **`test_swarm_flight`**:
    - **Purpose**: Validates the end-to-end swarm flight logic in the mock simulator.
    - **Verification**: Ensures drones reach targets and maintain boid constraints.

## 📂 Detailed Documentation

- [sar_swarm_control Testing Guide](sar_swarm_ws/src/sar_swarm_control/TESTING.md)

## 🛠 Testing Protocol

Autonomous agents and developers must follow the protocol defined in [AGENTS.md](AGENTS.md#2-testing-task-do-tests).

1. **Expand Coverage**: Add tests for new features and edge cases.
2. **Detailed Explanation**: When adding or updating tests, you **MUST** update this file (`TESTING.md`) with a detailed entry in the Test Catalog (Purpose, Input, Expected Outcome).
3. **Verify**: Run the full test suite (or `tests_standalone` if environment constrained).
4. **Document**: Always update this file (`TESTING.md`) and module-specific `TESTING.md` files after changes.
5. **Fix**: Ensure all tests pass before submission.
