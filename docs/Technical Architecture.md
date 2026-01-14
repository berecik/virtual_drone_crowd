# Technical Architecture: Distributed Aerial Search and Rescue (DAS-SAR)

[Languages]: [EN](Technical%20Architecture.md) | [PL](Technical%20Architecture_PL.md) | [UA](Technical%20Architecture_UA.md)

## 1. System Overview
The DAS-SAR system utilizes a **Distributed Lift System (DLS)** to transport heavy payloads, such as human casualties, using a swarm of coordinated drones. This architecture decouples total lift capacity from the size of individual agents, enabling the use of man-portable drones for heavy-lift missions.

## 2. Swarm Topology

### 2.1 Phase 1: Scaled Prototype
*   **Fleet Size:** 4 Units (Quadcopters).
*   **Objective:** Validation of control laws and cooperative state estimation.

### 2.2 Phase 2: Full-Scale Swarm
*   **Fleet Size:** 6 Units (Coaxial Octocopters/X8) + 1 Reserve Agent.
*   **Platform:** Custom heavy-lift frames optimized for distributed transport.
*   **Configuration:** Tethered star topology, where each agent is connected to the payload via an active winch system.

### 2.3 Physics of 6-Agent Redundancy
The Phase 2 swarm is explicitly defined as a minimum of **6 agents** to ensure **fail-operational** capability. 
*   **6-DOF Control:** A tethered payload requires a minimum of 6 linearly independent tension vectors to achieve full controllability over its 6 degrees of freedom (3D translation and 3D rotation).
*   **Safe Redundancy:** By utilizing 6 agents in a geometric configuration that avoids co-planarity of tethers, the system maintains full 6-DOF authority over the payload. In the event of a single agent failure, the remaining agents can redistribute their thrust vectors. While the loss of one agent reduces the total lift capacity, the "6-agent minimum" ensures that the swarm does not lose the ability to stabilize the payload's orientation or position, preventing catastrophic oscillations or loss of control during emergency maneuvers.

## 3. Distributed Control Allocation

The core of the DAS-SAR intelligence is the **Distributed Control Allocation (DCA)** layer, which translates desired payload motion into individual agent setpoints.

### 3.1 Thrust Vector Calculation
The swarm controller (running a consensus-based algorithm across the Jetson Orin modules) calculates the required **Resultant Wrench** ($\mathbf{W}_{req}$) needed to move the payload along its trajectory:
$$\mathbf{W}_{req} = [F_x, F_y, F_z, \tau_x, \tau_y, \tau_z]^T$$

### 3.2 Allocation Logic
The DCA algorithm distributes this wrench among the $N$ active agents:
1.  **Optimization:** It solves a constrained quadratic programming (QP) problem to find the optimal tension $T_i$ for each tether.
2.  **Constraints:**
    *   $T_{min} \leq T_i \leq T_{max}$ (Ensuring tethers never go slack and motors stay within limits).
    *   Geometric constraints based on the current relative positions of drones to the payload.
3.  **Redundancy Management:** If an agent reports a failure or low battery, the DCA instantly re-calculates the allocation matrix for $N-1$ agents, increasing the load on remaining units to maintain the required $\mathbf{W}_{req}$.

## 4. Hardware Specifications

### 4.1 Phase 2 Propulsion (The "Macro" Swarm)
To handle the 100kg+ payload requirement with high safety margins, Phase 2 utilizes industrial-grade propulsion systems.
*   **Motors:** **T-Motor U15 II** or **Hobbywing XRotor X9 Plus**.
*   **Configuration:** Coaxial X8 setup for internal motor redundancy.
*   **Thrust-to-Weight Ratio:** Targeted 2:1 ratio at nominal load to ensure recovery capability during agent failure.

### 4.2 Active Winch Mechanisms
Each Phase 2 agent is equipped with an **Active Winch Mechanism** for tether length control.
*   **Dynamic Damping:** The winch actively adjusts tether length to damp payload oscillations and compensate for gust-induced displacements.
*   **Variable Geometry:** Allows the swarm to expand or contract its radius mid-flight to navigate through narrow openings (e.g., between buildings or trees).
*   **Emergency Detach:** Integrated high-speed cable cutter and servo-release for immediate separation of failing agents.

## 5. Software Stack
*   **Middleware:** Eclipse Zenoh (Peer-to-Peer mesh networking).
*   **Control Logic:** Rust (Safety-critical flight loops).
*   **Autopilot:** PX4 running on Pixhawk 6C / Cube Orange.
*   **State Estimation:** Visual-Inertial Odometry (VIO) fused with RTK-GNSS.
