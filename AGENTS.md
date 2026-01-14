# AGENTS.md - Technical Context & Development Guide

This document provides a comprehensive technical overview and context for autonomous agents (AI) and developers working on the **Virtual Drone Crowd** project. It serves as a "brain dump" of architectural decisions, system constraints, and development patterns.

---

## 👥 Authors & Context

- **Author:** beret ([beret@hipisi.org.pl](mailto:beret@hipisi.org.pl))
- **Company:** Marysia Software Limited ([ceo@marysia.app](mailto:ceo@marysia.app))
- **Website:** [https://marysia.app](https://marysia.app)
- **Primary Domain:** `app.marysia.drone`

---

## 🚁 Project Essence
**Virtual Drone Crowd** (DAS-SAR) is a dual-phase Search and Rescue (SAR) system:
1.  **Phase 1 (Scout Swarm):** Agile, man-portable drones (Holybro X500 V2) for autonomous area search and human detection.
2.  **Phase 2 (Heavy Lift):** A **Distributed Lift System (DLS)** using a minimum of **6 heavy-lift agents** (coaxial X8) to evacuate human casualties (100kg+ payload).

**Core Philosophy:** Decentralization, Determinism (via Rust), and Fail-Operational Redundancy (6 agents for 6-DOF payload control).

---

## 🏗 System Architecture

### 1. The Autonomous Agent (Individual Drone)
Each drone is an independent ROS 2 entity.
-   **Low-Level (Firmware):** PX4 Autopilot on Pixhawk 6C/X. Handles real-time stabilization.
-   **High-Level (Compute):** NVIDIA Jetson Orin Nano/AGX. Runs the ROS 2 workspace.
-   **Middleware:** `zenoh-bridge-ros2dds` bridges local ROS 2 topics to the global swarm mesh.

### 2. Software Stack & Language Choice
-   **Rust (`rclrs`):** Used for `sar_swarm_control` and `heavy_lift_core`. **Why?** Memory safety, zero-cost abstractions, and predictable performance. Essential for safety-critical coordination and the Distributed Control Allocation (DCA) layer.
-   **Python (`rclpy`):** Used for `sar_perception` and `sar_simulation`. **Why?** AI ecosystem (PyTorch, YOLOv8/11).
-   **Zenoh:** Used for Inter-Drone (swarm) and Ground-to-Swarm comms. **Why?** Low-latency mesh networking; avoids DDS discovery overhead in wireless environments.

---

## 🛠 Repository & Workspace Structure

-   `/sar_swarm_ws/src/sar_swarm_control`: The Rust swarm logic (Boids, FSM, formation).
-   `/sar_swarm_ws/src/heavy_lift_core`: Phase 2 core logic (DCA, Admittance Control, 6-agent redundancy).
-   `/sar_swarm_ws/src/sar_perception`: Python nodes for vision and 3D localization.
-   `/sar_swarm_ws/src/sar_simulation`: Mock simulators and test runners.
-   `/docs`: SORA analysis, Technical Architecture, and Project Plans.

---

## 🧩 Key Integration Points (How to Develop)

### Adding a New Swarm Behavior
1.  **Modify `sar_swarm_control` (Rust):** Implement the logic in a new module.
2.  **State Machine:** Add new states to the FSM (e.g., `APPROACH_TARGET`).
3.  **PX4 Interface:** Use `px4_msgs::msg::TrajectorySetpoint` (ENU -> NED conversion required).

### Phase 2: Distributed Lift System (DLS)
-   **6-Agent Minimum:** Phase 2 operations **require** 6 agents to maintain 6-DOF control of the slung payload.
-   **Admittance Control:** Drones must "admit" tether forces to prevent rigid position fighting.
-   **Emergency Detach:** Safety logic must handle immediate tether release in case of critical failure.

### Inter-Drone Communication
-   **Mechanism:** Publish to a local ROS 2 topic mapped to Zenoh.
-   **Data types:** `VehicleOdometry` or custom lightweight messages. Avoid raw video/LIDAR over the mesh.

### AI Detection Pipeline
-   Detections must be transformed from Image Coordinates -> Camera Coordinates (using Depth Map) -> Drone Body Frame -> World Frame (using Odometry).
-   Publish global coordinates to `/perception/human_found`.

---

## 🚦 Development & Testing Workflow

### 1. Maintenance Task ("do maintenance")
When an agent receives the "do maintenance" command, it must follow this iterative protocol:
1.  **Run All Tests:**
    -   **Rust:** Run `cargo test` in `sar_swarm_ws/src/sar_swarm_control`.
    -   **Python:** Run `pytest` or equivalent in `sar_perception` and `sar_simulation`.
    -   **Simulation:** Execute `test_swarm_flight.py` to verify flight logic.
2.  **Fix Issues:** Analyze any failures (compilation errors, test regressions, or linter warnings) and apply fixes.
3.  **Iterate:** Repeat steps 1 and 2 until all tests pass and no issues remain.
4.  **Update Documentation:**
    -   **AGENTS.md:** Ensure this guide reflects the latest architectural changes or maintenance procedures.
    -   **ROADMAP.md:** Update milestones and current status based on completed tasks.
        -   **Status vocabulary:** Use `[ ]` (to do), `[/]` (in progress), or `[x]` (done/cover by tests).
        -   **Format:** Start each task line with the status checkbox (e.g., `- [x] Task description`) or update the `Status` column in tables.
        -   **Test Coverage:** Update the `(Test Coverage: X%)` next to Phase headers. Calculate this as the percentage of tasks within that phase marked as `(cover by tests)`.
        -   **Subtask Coverage:** For complex tasks with multiple subtasks, describe the specific coverage for each subtask if it is possible and makes sense (e.g., `- [x] (cover by tests) **Task Name**: Description (Coverage: Unit tests for X and Y)`).
    -   **Project Docs:** Update `README.md` and ensure translations are synchronized to keep them synchronized with the codebase.
5.  **Log Maintenance:** Record the date and summary of changes in the project's history or a dedicated `MAINTENANCE.log`.

### 2. Testing Task ("do tests")
When an agent receives the "do tests" command, it must prioritize coverage and robustness:
1.  **Expand Test Coverage:**
    -   **Unit Tests:** Identify untested functions (e.g., in `boids.rs`, `utils.rs`, `communication.rs`) and write comprehensive unit tests.
    -   **Integration Tests:** Create or update tests in `sar_swarm_ws/src/sar_swarm_control/tests/` to verify multi-module interactions, such as the PX4 handshake logic or Zenoh communication.
    -   **Edge Cases:** Add tests for negative altitudes, disconnected peers, and malformed messages.
2.  **Execute & Verify:**
    -   Run `cargo test` and ensure coverage is as high as possible.
    -   In environments where `rclrs` cannot compile, use a `tests_standalone` approach to verify logic.
3.  **Fix & Repeat:**
    -   Fix all discovered issues immediately.
    -   Repeat the process until coverage is satisfactory and no tests fail.
3.  **Update Documentation:**
    -   **TESTING.md:** Update the global test status and provide a **detailed explanation** for all tests (Purpose, Input, Expected Outcome).
    -   **Module Testing Docs:** Update specific files like `sar_swarm_ws/src/sar_swarm_control/TESTING.md` with detailed test descriptions.
    -   **Project Docs:** Ensure documentation reflects any changes in system behavior discovered during testing.

*Last Maintenance: 2026-01-14 21:15 - Updated Roadmap to reflect Phase 1 completion and Phase 2 "Active Development" status. Synchronized with SORA Safety Case (6-agent requirement) and Technical Architecture (DLS/Admittance Control).*

### 3. Simulation First
Always validate logic in simulation.
-   `mock_drone_sim.py`: Fast, lightweight for logic/protocol testing.
-   `SITL (PX4/Gazebo)`: (Planned) For physics and control tuning.

### 3. Dockerized Environment
The `Dockerfile` in the root contains all dependencies (Rust, ROS 2, Python libs).
-   Use `docker-compose up` to spin up a multi-drone test environment.

### 4. Safety First
-   **Heartbeat:** All high-level nodes should monitor the link to the Pixhawk.
-   **Failsafes:** Logic must default to `HOVER` or `RTL` (Return to Launch) if the perception or control node crashes.

---

## 📈 Roadmap Context for Agents
If you are tasked with moving the project forward, prioritize:
1.  **Raft Consensus Implementation:** Moving from simple Leader-Follower to a robust consensus for task allocation.
2.  **Zenoh-based Telemetry:** Finalizing the bridge configuration to ensure reliable GCS feedback.
3.  **Admittance Control (Phase 2):** Implementing force-feedback logic for tethered flight.

---

## 📝 Coding Standards
-   **Rust:** Follow `clippy` and `rustfmt`. Use `Arc<Mutex<T>>` for shared state in ROS 2 nodes.
-   **Python:** PEP 8 compliance. Use type hints.
-   **Documentation:** All new modules must be added to the relevant `README.md` and referenced in the main documentation.
-   **Languages:** Maintain translations in `README_PL.md`, `README_UA.md`, `README_HE.md`, etc.

---
*Generated for the Virtual Drone Crowd Project. Use this context to maintain architectural integrity.*
