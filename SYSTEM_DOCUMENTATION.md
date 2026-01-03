# System Documentation: Distributed Aerial SAR System (DAS-SAR)

**Authors:** beret@hipisi.org.pl & Marysia Software Limited (ceo@marysia.app, https://marysia.app)  
**Domain:** app.marysia.drone  
**Version:** 1.1 (2026-01-03)

## 1. Executive Summary

The Distributed Aerial SAR (DAS-SAR) project, also known as **Virtual Drone Crowd**, addresses the logistical challenges of Search and Rescue (SAR) by introducing a **Distributed Lift System (DLS)**. By decoupling lift capacity from individual drone size, a swarm of man-portable drones can coordinate to transport heavy payloads (100kg+), enabling rapid deployment in remote or urban-canyon environments inaccessible to traditional heavy-lift aircraft.

## 2. Theoretical Framework

### 2.1 Physics of Coupled Slung Load Systems
The engineering foundation rests on cooperative aerial manipulation. Agents are physically coupled through the payload via flexible tethers. The system state is the resultant vector of tension forces applied by $N$ agents. This allows for "geometric control," where the payload's orientation can be manipulated independently of its trajectory—critical for maneuvering a stretcher through complex environments (e.g., forest canopy).

### 2.2 Control Strategies: Admittance vs. Impedance
Standard position controllers are too "rigid" for coupled systems, where small GNSS drifts can cause drones to "fight" each other, leading to motor saturation or structural failure.
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
Risk-reduction campaign using a 3-4 quadcopter swarm for 2-5kg payload validation.

*   **Airframe:** **Holybro X500 V2**. Carbon Fiber arms for high rigidity (crucial to avoid IMU noise in control loops).
*   **Computational Core:** **NVIDIA Jetson Orin Nano (8GB)**. Handles real-time MPC (50Hz+), VIO, and Zenoh networking (40 TOPS AI performance).
*   **Flight Controller:** **Holybro Pixhawk 6C**. STM32H7 processor, triple-redundant IMUs, vibration isolation.
*   **Sensors:** 
    *   **GNSS:** **CubePilot Here 4**. Multiband RTK for centimeter-level precision (DroneCAN).
    *   **Vision:** **Intel RealSense D435i** or **Luxonis OAK-D Pro**. Visual Inertial Odometry (VIO) for GPS-denied environments.
*   **Power:** **Matek BEC 12S-PRO**. 9-55V input, 5A output. Protects Jetson from voltage spikes during motor braking.

**Phase 1 Bill of Materials (Estimated Unit Cost: €2,490)**
| Component | Specification | Cost |
| :--- | :--- | :--- |
| Airframe | Holybro X500 V2 ARF | €380 |
| Flight Controller | Pixhawk 6C | €215 |
| GNSS | Here 4 RTK | €285 |
| Companion PC | Jetson Orin Nano 8GB | €450 |
| Depth Camera | RealSense D435i | €415 |
| Telemetry | SiYi MK15 | €550 |
| Misc | BEC, Batteries, Mounts | €195 |

### 3.2 Phase 2: Full-Scale Heavy Lift
Targeting 115kg total lift (80kg Human + 10kg Stretcher + 5kg Rigging + 20% margin).

*   **Propulsion:** **T-Motor U15 II KV80** (~36kg thrust/motor) or **Hobbywing XRotor X9 Plus** (26.5kg thrust/axis).
*   **Configuration:** **Octocopter (X8) Coaxial**. Critical for redundancy; allows safe landing if one motor fails.
*   **Airframe:** **Foxtech Gaia 160MP** (modified) or custom CNC carbon fiber (40mm tubes).
*   **Power:** **LiPo (Tattu Pro 12S 22000mAh)**. Required for high discharge rates (25C+) to handle stabilization torque. Li-Ion is not recommended due to voltage sag under load.
*   **Rigging:** Dyneema (UHMWPE) tethers with integrated shock cords and servo-actuated emergency release hooks.

## 4. Software Stack

### 4.1 Framework: ROS 2 Humble
Utilizes decentralized Peer-to-Peer communication. **Lifecycle Management** ensures all safety checks (GPS fix, sensor health) are active before arming.

### 4.2 Core Logic: Rust
Mandatory for safety-critical control loops to eliminate memory safety bugs and race conditions.
*   **MAVSDK-Rust:** Safe interface for PX4 offboard control.
*   **ros2_rust:** Client libraries for ROS 2 nodes.

### 4.3 Communication: Eclipse Zenoh
Optimized for mesh networking over unreliable wireless links.
*   **Discovery Efficiency:** Reduces discovery overhead by up to 99% compared to DDS by avoiding "discovery storms" typical of multicast-based protocols.
*   **WAN Support:** Native routing for long-range Ground Control Station (GCS) connectivity, even through NAT.
*   **Bridging:** `zenoh-bridge-ros2dds` for local-to-global swarm topic bridging, allowing drones to see each other as remote nodes while maintaining efficient transport.

### 4.4 Consensus & Coordination
*   **Raft Consensus:** Implemented in Rust to ensure reliable state agreement across the swarm (e.g., "Target Confirmed", "Leader Election").
*   **Area Partitioning:** Dynamic Voronoi tessellation or sector bidding based on battery levels and proximity.
*   **Boids Model:** Emergent behaviors for transit (Separation, Alignment, Cohesion).

## 5. User Interfaces & Interaction

### 5.1 React GCS (Command Center)
Stationary Ground Control Station for tactical overview.
*   **3D Visualization:** Real-time drone tracking and mission paths using CesiumJS/Leaflet.
*   **Swarm Health:** Telemetry grid, battery status, and signal strength (RSSI) monitoring.
*   **Video Feed:** Low-latency multi-stream grid for situational awareness.

### 5.2 Flutter Tactical Terminals
Mobile interface for field rescuers on rugged tablets.
*   **Flutter Rust Bridge:** High-performance UI using Rust for the underlying Zenoh communication layer.
*   **"Follow Me" Mode:** Tactical coordination where drones track the rescuer's position.

## 6. Payload Interface & Patient Safety

*   **Stretcher:** Standard **NATO Litter (Stanag 2040)**.
*   **Spreader Bar:** Custom bar to clear patient's body and maintain lift geometry.
*   **Emergency Protocol:** Immediate tether release for failed agents; remaining agents execute emergency controlled descent.

## 6. Marine SAR Application

*   **Environmental Hardening:** IP67 rating and salt spray corrosion protection (T-Motor industrial coatings).
*   **Dynamic Recovery:** Heave compensation for landing on pitching ship decks using VIO and AprilTag/Moving Base tracking.
*   **Search Patterns:** Parallel sweep and creeping line patterns optimized for leeway drift (maritime search efficiency).

## 7. Safety Protocols and Regulatory Compliance (SORA)

The project follows the **Specific Operations Risk Assessment (SORA)** methodology (EASA/ULC).
*   **SAIL Level:** Targeted **SAIL V/VI** (High risk, borders on "Certified").
*   **Redundancy:** X8 motor configuration, triple-redundant IMUs, independent Flight Termination System (FTS).
*   **Software Assurance:** Rust codebase designed towards DO-178C standards.
*   **Human-Swarm Interaction:** Strategic commands (e.g., "Evacuate Target") via **React** GCS or **Flutter** tactical tablets (using `flutter_rust_bridge`).

### 7.1 Risk Matrix & Mitigation
| Risk | Probability | Impact | Mitigation Strategy |
| :--- | :--- | :--- | :--- |
| **Comms Loss** | Medium | High | Autonomous RTH procedures in PX4; Zenoh Mesh relaying. |
| **Propulsion Failure** | Low | Critical | X8 Coaxial redundancy; Ballistic parachute for Heavy Lift units. |
| **Software Bug** | High (Dev) | Medium | SITL validation before flight; Rust for memory safety. |
| **Regulatory Denial** | Medium | High | Early engagement with ULC/EASA; Professional SORA consultancy. |

## 8. Financial Summary & Roadmap

*   **Total Budget:** ~€270,000 (including R&D, Hardware, and Regulatory Consultancy).
*   **Timeline:** 18 months from Phase 1 PoC to Phase 2 Certification.
*   **Current Status:** Phase 1 software validated in SITL; hardware procurement initiated.
