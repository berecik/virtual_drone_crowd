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
**Virtual Drone Crowd** is a dual-phase Search and Rescue (SAR) system:
1.  **Phase 1 (Scout Swarm):** Agile, man-portable drones for autonomous area search and human detection.
2.  **Phase 2 (Heavy Lift):** Large-scale platforms for physical human evacuation using distributed or centralized lift.

**Core Philosophy:** Decentralization, Determinism (via Rust), and Communication Efficiency (via Zenoh).

---

## 🏗 System Architecture

### 1. The Autonomous Agent (Individual Drone)
Each drone is an independent ROS 2 entity.
-   **Low-Level (Firmware):** PX4 Autopilot on Pixhawk 6C/X. Handles real-time stabilization.
-   **High-Level (Compute):** NVIDIA Jetson Orin. Runs the ROS 2 workspace.
-   **Middleware:** `zenoh-bridge-ros2dds` bridges local ROS 2 topics to the global swarm mesh.

### 2. Software Stack & Language Choice
-   **Rust (`rclrs`):** Used for `sar_swarm_control`. **Why?** Memory safety, zero-cost abstractions, and predictable performance without GC pauses. Essential for safety-critical swarm coordination and flight logic.
-   **Python (`rclpy`):** Used for `sar_perception`. **Why?** Deep learning ecosystem (PyTorch, TensorRT, YOLOv8). AI model inference is the primary use case.
-   **Zenoh:** Used for Inter-Drone (swarm) and Ground-to-Swarm comms. **Why?** Outperforms DDS in lossy wireless environments; avoids "discovery storms."

---

## 🛠 Repository & Workspace Structure

-   `/sar_swarm_ws/src/sar_swarm_control`: The Rust core. Look here for FSM, formation logic, and PX4 setpoint publishing.
-   `/sar_swarm_ws/src/sar_perception`: Python nodes for vision. Look here for detection-to-3D coordinate projection.
-   `/sar_swarm_ws/src/sar_simulation`: Mock simulators and test runners.
-   `/docker`: Infrastructure configurations.

---

## 🧩 Key Integration Points (How to Develop)

### Adding a New Swarm Behavior
1.  **Modify `sar_swarm_control` (Rust):** Implement the logic in a new module or within the main loop.
2.  **State Machine:** Add new states to the FSM (e.g., `APPROACH_TARGET`, `WAIT_FOR_EXTRACTION`).
3.  **PX4 Interface:** Use `px4_msgs::msg::TrajectorySetpoint` to command the drone. Ensure `position` and `yaw` are set correctly.

### Inter-Drone Communication
-   Do **not** use standard ROS 2 discovery over WiFi for large swarms.
-   **Mechanism:** Publish to a local ROS 2 topic. Ensure it is added to the `zenoh-bridge` allow-list. Other drones will "see" this topic as if it were local.
-   **Data types:** Prefer `VehicleOdometry` or custom lightweight messages. Avoid sending raw sensor streams (LIDAR/Video) over the mesh unless requested.

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
    -   **Project Docs:** Update `README.md`, `SYSTEM_DOCUMENTATION.md`, and `WORKING_PLAN.md` to keep them synchronized with the codebase.
5.  **Log Maintenance:** Record the date and summary of changes in the project's history or a dedicated `MAINTENANCE.log`.

*Last Maintenance: 2026-01-02 - Performed routine maintenance, created MAINTENANCE.log, synchronized ROADMAP.md and WORKING_PLAN.md. Verified Phase 1 completion and Phase 2 initialization.*

### 2. Simulation First
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
-   **Documentation:** All new modules must be added to the relevant `README.md` and referenced in `SYSTEM_DOCUMENTATION.md`.

---
*Generated for the Virtual Drone Crowd Project. Use this context to maintain architectural integrity.*
