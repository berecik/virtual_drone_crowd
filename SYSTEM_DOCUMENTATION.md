# System Documentation: Virtual Drone Crowd

**Authors:** beret@hipisi.org.pl & Marysia Software Limited (ceo@marysia.app, https://marysia.app)  
**Domain:** app.marysia.drone

## 1. Executive Summary

The Virtual Drone Crowd project addresses the logistical challenges of Search and Rescue (SAR) by introducing a **Distributed Lift System (DLS)**. By decoupling lift capacity from individual drone size, a swarm of man-portable drones can coordinate to transport heavy payloads (100kg+), enabling rapid deployment in remote or urban-canyon environments inaccessible to traditional heavy-lift aircraft.

## 2. Theoretical Framework

### 2.1 Physics of Coupled Slung Load Systems
The engineering foundation rests on cooperative aerial manipulation. Agents are physically coupled through the payload via flexible tethers. The system state is the resultant vector of tension forces applied by $N$ agents. This allows for "geometric control," where the payload's orientation can be manipulated independently of its trajectory—critical for maneuvering through complex environments.

### 2.2 Control Strategies: Admittance vs. Impedance
Standard position controllers are too "rigid" for coupled systems, where small GNSS drifts can cause drones to "fight" each other.
*   **Impedance Control:** High stiffness, resists external forces to maintain position.
*   **Admittance Control:** Low stiffness, "admits" external forces and complies with them.
The system uses **Admittance Control**, modeling each drone as a virtual mass-spring-damper system. This mimics biological coordination (e.g., ants moving food), where agents adjust based on felt forces.

### 2.3 Optimization: Model Predictive Control (MPC)
To achieve stable flight, each agent runs a non-linear MPC solver (ACADO/OSQP) on its companion computer at 50Hz+.
**MPC Constraints:**
1.  **Actuator Limits:** Motors cannot exceed max thrust.
2.  **Cable Tension:** $T > 0$ (Cables must never go slack to avoid snap loads).
3.  **Collision Avoidance:** Minimum separation distance maintenance.

## 3. Hardware Architecture

### 3.1 Phase 1: Scaled Prototype ("Micro" Swarm)
*   **Airframe:** Holybro X500 V2 (Carbon Fiber for rigidity).
*   **Computational Core:** NVIDIA Jetson Orin Nano (8GB). Handles real-time MPC, VIO, and Zenoh networking.
*   **Flight Controller:** Holybro Pixhawk 6C (PX4). Handles low-level attitude stabilization.
*   **Sensors:** 
    *   **GNSS:** CubePilot Here 4 (Multiband RTK for cm-level precision).
    *   **Vision:** Luxonis OAK-D Pro or Intel RealSense D435i (Visual Inertial Odometry for GPS-denied environments).
*   **Power:** Matek BEC 12S-PRO (Regulated power to protect Jetson from motor voltage spikes).

### 3.2 Phase 2: Full-Scale Heavy Lift
*   **Payload Target:** 115kg (Human + stretcher + rigging + safety margin).
*   **Propulsion:** T-Motor U15 II KV80 (~36kg thrust per motor).
*   **Configuration:** Octocopter (X8) Coaxial for critical redundancy.

## 4. Software Stack

### 4.1 Framework: ROS 2 Humble
Utilizes decentralized Peer-to-Peer communication and Lifecycle Management to ensure all safety checks are active before arming.

### 4.2 Core Logic: Rust
Mandatory for safety-critical control loops. Rust's memory ownership model eliminates memory safety bugs and race conditions without runtime overhead.
*   **MAVSDK-Rust:** Safe interface for PX4 offboard control.

### 4.3 Communication: Eclipse Zenoh
Replaces standard DDS for inter-drone communication.
*   **Discovery Efficiency:** Reduces discovery overhead by up to 99%.
*   **Reliability:** Optimized for high-performance communication over unreliable wireless links (Mesh Networking).
*   **Bridging:** `zenoh-bridge-ros2dds` bridges local ROS 2 topics to the global swarm network.

## 5. Marine SAR Application

*   **Environmental Hardening:** IP67 rating and salt spray corrosion protection.
*   **Dynamic Recovery:** Heave compensation for landing on pitching ship decks using VIO and AprilTag tracking.
*   **Search Patterns:** Parallel sweep and creeping line patterns optimized for leeway drift in maritime environments.

## 6. Safety Protocols and Regulatory Compliance (SORA)

The project follows the **Specific Operations Risk Assessment (SORA)** methodology (EASA/ULC).
*   **Redundancy:** Decentralized swarm logic (no single point of failure), X8 motor configuration, and triple-redundant IMUs.
*   **Ground/Air Risk Mitigation:** Operations over controlled SAR zones and low-altitude flight (<120m).
*   **Human-Swarm Interaction:** High-level strategic commands (e.g., "Search Sector A") via React-based GCS or Flutter-based tactical tablets.
