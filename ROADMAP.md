# Project Roadmap: Virtual Drone Crowd

**Maintained by:** beret@hipisi.org.pl & Marysia Software Limited (https://marysia.app)

## 🗓 Timeline Overview

| Milestone | Target Date | Status | Description |
| :--- | :--- | :--- | :--- |
| **M1: Swarm Foundation** | Month 3 | [x] | Basic swarm formation and Zenoh comms in simulation. |
| **M2: Perception PoC** | Month 6 | [ ] | Autonomous human detection and 3D localization. |
| **M3: Scout Hardware Ready** | Month 8 | [ ] | Flight tests of 4x X500 drones with Jetson Orin Nano. |
| **M4: Heavy Lift Prototype** | Month 12 | [ ] | First lift of 80kg dummy load using Octocopter. |
| **M5: Integrated SAR Demo** | Month 15 | [ ] | End-to-end "Search & Rescue" mission demonstration. |
| **M6: Certification & Deployment** | Month 18 | [ ] | SORA approval and operational readiness for emergency services. |

## 📍 Phase 1: Scouting Swarm (PoC) - [Current Focus] (Test Coverage: 85%)

### 1. Development Environment & Core Control
- [x] **Infrastructure:** Finalize Docker-based development environment with ROS 2 Jazzy and Rust (rclrs).
- [x] **Swarm Node (Rust):** Implement a robust `sar_swarm_control` node in Rust.
    - [x] (cover by tests) Boids algorithm (Separation, Alignment, Cohesion). (Coverage: Unit tests for all three forces)
    - [x] (cover by tests) Zenoh-based inter-drone state sharing. (Coverage: Serialization and message processing)
    - [x] **XRCE-DDS Integration (Offboard control loop, heartbeat, setpoints).** (Coverage: Lifecycle state transitions and heartbeat logic)
- [x] (cover by tests) **Coordinate & QoS Optimization:** Implement ENU->NED transforms and specialized QoS for Pixhawk 6C (BestEffort Telemetry, Reliable Commands). (Coverage: Math and transform logic, QoS configuration)

### 2. Perception & AI
- [x] (cover by tests) **Detector Node:** Develop `sar_perception` node using Python and YOLOv8. (Coverage: Mock-based unit tests for depth logic)
- [ ] **Localization:** Implement 3D point estimation of detections using OAK-D depth data and TF2 transforms.
- [x] (cover by tests) **Search Algorithm:** Implement a lawnmower or area-partitioning search pattern. (Coverage: Unit tests for waypoint generation)

### 3. Simulation & Validation
- [x] **SITL Setup:** Integrate PX4 Software-in-the-Loop with Gazebo Ignition (OAK-D Config DONE).
- [x] (cover by tests) **Core Control Validation:** Verified Offboard handshake and hover setpoint logic. (Coverage: State machine and handshake sequence)
- [ ] **Logic Testing:** Validate autonomous "Target Found" -> "Orbit Target" sequence in simulation.
- [ ] **Hardware-in-the-loop (HITL):** Test Jetson Orin Nano communication with Pixhawk 6C via XRCE-DDS Agent.

## 🏗 Phase 2: Heavy-Lift & Extraction - [Started] (Test Coverage: 7%)

### 4. Heavy-Lift Platform Build
- [x] (cover by tests) **Architecture Skeleton:** Initialize `heavy_lift_core` with action definitions and state machine. (Coverage: State transition logic)
- [ ] **Hardware Procurement:** Order Gaia 160MP frame, T-Motor U13/U15 propulsion, and 12S batteries.
- [ ] **Power System:** Design and build high-current PDB and BEC systems for Jetson AGX Orin.
- [ ] **Rigging:** Design spreader bar for NATO stretcher and integrate electronic release hooks.

### 5. Advanced Control Algorithms
- [ ] **Admittance Control:** Implement force-feedback-based position adjustment in Rust.
- [ ] **Load Damping:** Develop algorithms to minimize payload oscillation during transport.
- [ ] **Redundancy Logic:** Implement "Motor Failure" recovery modes for X8 coaxial configuration.

### 6. Ground Station & UX
- [ ] **GCS Dashboard (React):** Build a 3D tactical map showing real-time drone positions and video feeds.
- [ ] **Field App (Flutter):** Develop a rugged tablet interface for rescuers to "request pickup" or "mark target".

## 🚀 Phase 3: Integration & Certification (Test Coverage: 0%)

### 7. Full System Integration
- [ ] **Multi-Drone Extraction:** Test 4 scouts providing visual coverage for 1 heavy-lifter.
- [ ] **End-to-End Mission:** Perform full autonomous mission: Search -> Detect -> Call Heavy Lift -> Extract -> RTB.

### 8. Regulatory Approval
- [ ] **Documentation:** Finalize SORA (Specific Operations Risk Assessment) for ULC/EASA.
- [ ] **Testing:** Perform certified flight tests with dummy loads and parachute system validation.
