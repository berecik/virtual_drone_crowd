# System Documentation: Virtual Drone Crowd

**Authors:** beret@hipisi.org.pl & Marysia Software Limited (ceo@marysia.app, https://marysia.app)  
**Domain:** app.marysia.drone

## 1. System Architecture

The system is designed as a decentralized swarm of autonomous agents. It follows a multi-layered architecture to ensure safety, performance, and flexibility.

### 1.1 Hardware Layers
- **Flight Control (Low-Level):** Pixhawk 6C/6X running PX4 Autopilot. Handles IMU fusion, stabilization, and motor output.
- **Companion Compute (High-Level):** NVIDIA Jetson Orin Nano (Phase 1) or AGX Orin (Phase 2). Runs ROS 2, AI vision, and swarm coordination logic.
- **Vision Subsystem:** Luxonis OAK-D Pro (active IR depth sensing) or Intel RealSense D435i.

### 1.2 Software Stack
- **Operating System:** Ubuntu 24.04 LTS with ROS 2 Jazzy Jalisco.
- **Communication Middleware:** Eclipse Zenoh and uXRCE-DDS. Zenoh handles inter-drone swarm networking, while uXRCE-DDS provides the bridge to the PX4 Autopilot.
- **Node Architecture:** All critical control nodes are implemented as `LifecycleNode` (in Rust/rclrs) to manage safety states and hardware access.
- **Swarm Logic (Rust):** Implemented in Rust for memory safety and deterministic execution. Handles leader election (Raft), formation control, and offboard flight control loops.
- **Perception Layer (Python):** Uses YOLOv8 (TensorRT optimized) for real-time victim detection.

## 2. Communication Model

The system utilizes a hybrid communication model:
1. **Intra-Drone (ROS 2):** Nodes within a single drone communicate via standard ROS 2 topics/services.
2. **Inter-Drone (Zenoh):** The `zenoh-bridge-ros2dds` bridges selected topics to a global swarm mesh network.
3. **Ground Station (Zenoh-JS/Flutter):** Web-based and mobile dashboards connect to the swarm via Zenoh routers.

### Quality of Service (QoS)
Based on DDS principles to mitigate wireless saturation:
- **Telemetry (`/fmu/out/vehicle_global_position`):** `BestEffort`, `Volatile`. Prioritizes low-latency for high-frequency updates where lost packets are superseded by fresh data.
- **Commands (`/fmu/in/trajectory_setpoint`):** `Reliable`, `TransientLocal`. Ensures critical control targets are delivered and allows late-joining bridge components to receive the last setpoint.
- **Video/LIDAR:** `BestEffort`, `Volatile` (standard for high-bandwidth streams).

## 3. Subsystems

### 3.1 `sar_swarm_control` (Rust)
The core "brain" of each drone.
- **Offboard Control Node:** Implements `LifecycleNode` for safe PX4 offboard control.
    - **Heartbeat:** 10Hz proof-of-life signal to Pixhawk 6C.
    - **Transforms:** Handles ROS 2 (ENU) to PX4 (NED) coordinate conversions.
    - **Safety:** Monitors GPS fix (VehicleGlobalPosition) before enabling offboard mode.
- **State Machine:** Manages mission phases (Idle, Search, Follow, Extract).
- **Admittance Controller:** (Planned for Phase 2) Adjusts drone position based on tether tension.
- **Safety Monitor:** Heartbeat checking and autonomous RTL (Return to Launch) on link loss.

### 3.2 `sar_perception` (Python)
- **Object Detection:** Identifies human signatures in RGB and Thermal streams.
- **Coordinate Transformation:** Projects 2D detections into 3D world coordinates using depth maps and drone odometry.

### 3.3 `sar_simulation`
- **Mock Sim:** Lightweight UDP-based simulation for logic testing.
- **Gazebo/SITL:** (Planned) High-fidelity physics simulation for propulsion and tether dynamics.

## 4. Operational Phases

### Phase 1: Reconnaissance (PoC)
- **Objective:** Autonomous area coverage and victim localization.
- **Platform:** Holybro X500 V2.
- **Outcome:** Centimeter-accurate localization of a manikin in a search sector.

### Phase 2: Heavy-Lift Evacuation
- **Objective:** Physical transport of a 100kg load (human + stretcher).
- **Platform:** Custom Octocopter (X8) / Foxtech Gaia 160MP.
- **Mechanism:** Single heavy-lifter or distributed lift (swarms tethered to one payload) using Admittance Control.

## 5. Regulatory Compliance

### EASA / ULC (Poland)
- **Category:** Specific (requires Operational Authorization) or Certified.
- **SORA:** All operations are based on SORA (Specific Operations Risk Assessment).
- **SAIL Level:** Expected SAIL III/IV for Phase 1, SAIL V/VI for Phase 2.
- **Safety Systems:** Independent FTS (Flight Termination System) and ballistic parachutes for heavy-lift units.
