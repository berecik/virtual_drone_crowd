# Project Roadmap: Virtual Drone Crowd

**Maintained by:** beret@hipisi.org.pl & Marysia Software Limited (https://marysia.app)

## 🗓 Timeline Overview

| Milestone | Target Date | Status | Description |
| :--- | :--- | :--- | :--- |
| **M1: Swarm Foundation** | Month 3 | [x] | Basic swarm formation and Zenoh comms in simulation. |
| **M2: Perception PoC** | Month 6 | [x] | Autonomous human detection and 3D localization logic. |
| **M3: Scout Hardware Ready** | Month 9 | [/] | Flight tests of 4x X500 drones with Jetson Orin Nano. |
| **M4: Heavy Lift Prototype** | Month 14 | [/] | First lift of 100kg+ load using 6-agent DLS. |
| **M5: Integrated SAR Demo** | Month 18 | [ ] | End-to-end "Search & Rescue" mission (Scout + Heavy Lift). |
| **M6: Certification & Deployment** | Month 24 | [ ] | SORA approval and operational readiness (SAIL V/VI). |

## 📍 Phase 1: Scouting Swarm (PoC) - [Completed/Maintaining] (Test Coverage: 100%)

### 1. Development Environment & Core Control
- [x] **Infrastructure:** Finalize Docker-based development environment with ROS 2 Jazzy and Rust (rclrs).
- [x] **Swarm Node (Rust):** Implement a robust `sar_swarm_control` node in Rust.
    - [x] (cover by tests) Boids algorithm (Separation, Alignment, Cohesion). (Coverage: Unit tests for all three forces)
    - [x] (cover by tests) Zenoh-based inter-drone state sharing. (Coverage: Serialization and message processing)
    - [x] **XRCE-DDS Integration (Offboard control loop, heartbeat, setpoints).** (Coverage: Lifecycle state transitions, 10Hz heartbeat, GPS fix logic, ENU->NED verified via standalone tests)
- [x] (cover by tests) **Coordinate & QoS Optimization:** Implement ENU->NED transforms and specialized QoS for Pixhawk 6C (BestEffort Telemetry, Reliable Commands). (Coverage: Math and transform logic, DDS-principled QoS configuration, Unit tests for ENU->NED)

### 2. Perception & AI
- [x] (cover by tests) **Detector Node:** Develop `sar_perception` node using Python and YOLOv8/11. (Coverage: Mock-based unit tests for depth logic and 3D projection)
- [x] (cover by tests) **Localization:** Implement 3D point estimation of detections using OAK-D depth data and TF2 transforms. (Coverage: Deprojection math and TF2 error handling)
- [x] (cover by tests) **Search Algorithm:** Implement a lawnmower or area-partitioning search pattern. (Coverage: Unit tests for waypoint generation)

### 3. Simulation & Validation
- [x] **SITL Setup:** Integrate PX4 Software-in-the-Loop with Gazebo Harmonic (OAK-D Config DONE).
- [x] (cover by tests) **Core Control Validation:** Verified Offboard handshake and hover setpoint logic. (Coverage: State machine and handshake sequence)
- [x] (cover by tests) **Logic Testing:** Validate autonomous "Target Found" -> "Orbit Target" sequence in simulation.
- [ ] **Hardware-in-the-loop (HITL):** Test Jetson Orin Nano communication with Pixhawk 6C via XRCE-DDS Agent.

## 🏗 Phase 2: Heavy-Lift & Extraction - [Active Development] (Test Coverage: 15%)

### 4. Heavy-Lift Platform (6-Agent DLS)
- [x] (cover by tests) **Architecture Skeleton:** Initialize `heavy_lift_core` with action definitions and state machine. (Coverage: IDLE -> EN_ROUTE -> LIFTING state transitions)
- [ ] **Redundant Configuration:** Implement 6-agent fail-operational logic (controllability with 5/6 agents).
- [ ] **Hardware Procurement:** Order T-Motor U15 II / Hobbywing X9 Plus propulsion and coaxial X8 frames.
- [ ] **Active Winch System:** Design and implement active tether damping and winch control logic in Rust.
- [ ] **Rigging:** Design spreader bar for NATO stretcher and integrate emergency release hooks.

### 5. Advanced Control & Safety
- [ ] **Admittance Control:** Implement mass-spring-damper model for swarm-payload coupling to handle tether tension.
- [ ] **Distributed Control Allocation (DCA):** Solve QP problem for optimal tether tension and resultant wrench.
- [ ] **SORA Compliance:** Implement "Emergency Detach" and "Failsafe RTL" for high-risk human extraction (SAIL V/VI).
- [ ] **MPC Solver:** Integrate ACADO/OSQP MPC solvers for non-linear trajectory optimization.

### 6. Ground Station & UX
- [ ] **GCS Dashboard (React):** Build a 3D tactical map showing real-time drone positions and video feeds.
- [ ] **Field App (Flutter):** Develop a rugged tablet interface for rescuers to "request pickup" or "mark target".

## 🚀 Phase 3: Integration & Certification (Test Coverage: 0%)

### 7. Full System Integration
- [ ] **Multi-Drone Collaboration:** 4 scouts providing visual coverage and localization for 6 heavy-lifters.
- [ ] **End-to-End Mission:** Perform full autonomous mission: Search -> Detect -> Call Heavy Lift -> Extract -> RTB.

### 8. Regulatory Approval
- [ ] **SORA Finalization:** Finalize Specific Operations Risk Assessment for ULC/EASA.
- [ ] **Certified Testing:** Perform flight tests with dummy loads and parachute system validation.

---
