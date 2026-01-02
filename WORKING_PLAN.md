# Working Plan: Virtual Drone Crowd

**Contact:** beret@hipisi.org.pl | Marysia Software Limited (https://marysia.app)

This plan outlines the specific technical tasks required to move from the current codebase to a fully functional SAR swarm.

## Phase 1: Scouting Swarm (PoC)

### 1. Development Environment & Core Control
- [x] **Infrastructure:** Finalize Docker-based development environment with ROS 2 Humble and Rust (rclrs).
- [x] **Swarm Node (Rust):** Implement a robust `sar_swarm_control` node in Rust. [COMPLETED]
    - [x] Boids algorithm (Separation, Alignment, Cohesion).
    - [x] Zenoh-based inter-drone state sharing.
    - [x] ROS 2 integration (MAVROS topics).
- [ ] **MAVLink Integration:** Connect ROS 2 `TrajectorySetpoint` to PX4 via XRCE-DDS or MAVROS.

### 2. Perception & AI
- [ ] **Detector Node:** Develop `sar_perception` node using Python and YOLOv8.
- [ ] **Localization:** Implement 3D point estimation of detections using OAK-D depth data and TF2 transforms.
- [ ] **Search Algorithm:** Implement a lawnmower or area-partitioning search pattern.

### 3. Simulation & Validation
- [ ] **SITL Setup:** Integrate PX4 Software-in-the-Loop with Gazebo Ignition.
- [ ] **Logic Testing:** Validate autonomous "Target Found" -> "Orbit Target" sequence in simulation.
- [ ] **Hardware-in-the-loop (HITL):** Test Jetson Orin Nano communication with Pixhawk 6C.

## Phase 2: Heavy-Lift & Extraction [Phase 2 Implementation Started]

### 4. Heavy-Lift Platform Build
- [x] **Architecture Skeleton:** Initialize `heavy_lift_core` with action definitions and state machine. [DONE]
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

## Phase 3: Integration & Certification

### 7. Full System Integration
- [ ] **Multi-Drone Extraction:** Test 4 scouts providing visual coverage for 1 heavy-lifter.
- [ ] **End-to-End Mission:** Perform full autonomous mission: Search -> Detect -> Call Heavy Lift -> Extract -> RTB.

### 8. Regulatory Approval
- [ ] **Documentation:** Finalize SORA (Specific Operations Risk Assessment) for ULC/EASA.
- [ ] **Testing:** Perform certified flight tests with dummy loads and parachute system validation.
