# Safety Case: Specific Operations Risk Assessment (SORA)

[Languages]: [EN](Safety_Case_SORA.md) | [PL](Safety_Case_SORA_PL.md) | [UA](Safety_Case_SORA_UA.md)

**Role:** Aviation Safety Specialist  
**Project:** DAS-SAR: Distributed Aerial Search and Rescue Swarm  
**Version:** 1.0  
**Date:** 2026-01-14

## 1. Introduction
This document outlines the safety case for the Phase 2 DAS-SAR mission: the autonomous aerial evacuation of a human casualty using a distributed lift swarm. Given the high-risk nature of carrying a human payload, this assessment follows the EASA SORA (Specific Operations Risk Assessment) methodology to determine the Ground Risk Class (GRC) and Air Risk Class (ARC), aiming for a SAIL (Specific Assurance and Integrity Level) V or VI classification.

## 2. Risk Analysis: Human Payload Extraction
Carrying a human (80kg - 100kg) via a distributed drone swarm introduces several critical risk factors that differ from standard cargo operations:

*   **Injury Severity:** A crash involving a human payload is inherently catastrophic. Unlike inert cargo, the payload is sensitive to accelerations, vibrations, and orientation.
*   **System Complexity:** The use of multiple physical tethers increases the risk of entanglement or "death spirals" if one agent behaves erratically.
*   **Dynamic Instability:** A human on a stretcher is a non-rigid, shifting mass. Movement of the patient can induce unexpected CG (Center of Gravity) shifts and pendulum oscillations in the swarm.
*   **Environmental Factors:** SAR missions often occur in "Urban Canyons" or mountainous terrain where GPS multipath and unpredictable wind gusts are prevalent.

## 3. Mitigation: 6-Drone Redundant Configuration
The primary technical mitigation against catastrophic failure is the shift from a 4-drone prototype to a **6-drone fail-operational configuration**.

### 3.1 Thrust Redundancy and GRC Reduction
The swarm is designed to survive the total loss of any single agent (1-of-6 failure) mid-flight without dropping the payload.

*   **Nominal State:** Each of the 6 drones carries ~17% of the total load (approx. 19kg of a 115kg total MTOM).
*   **Failure State:** If a motor burns out, an ESC fails, or a battery reaches a critical state, the swarm's control logic (running in Rust for safety-critical reliability) redistributes the load. The remaining 5 drones increase their thrust to carry ~24kg each.
*   **Safety Factor:** Each heavy-lift drone (utilizing T-Motor U15-II or Hobbywing X9 propulsion) is rated for up to 40kg of thrust. This provides a 2:1 thrust-to-weight ratio even in a failure state, ensuring the swarm can maintain altitude and perform a controlled emergency landing.
*   **GRC Impact:** This high level of redundancy significantly lowers the Ground Risk Class (GRC) by reducing the probability of an uncontrolled impact. The system transitions from a "Fail-Safe" (crashing safely) to a "Fail-Operational" (continuing flight) paradigm.

## 4. Containment: Emergency Detach Protocol
To prevent a single failing drone from jeopardizing the entire swarm, an "Emergency Detach" protocol is implemented via active tether management.

### 4.1 Protocol Execution
In the event of a critical failure (e.g., structural failure, fire, or total loss of control) that cannot be compensated for by thrust redistribution:

1.  **Slack Generation:** The failing drone (or the swarm logic commanding its winch) attempts to create momentary slack in its tether.
2.  **Pyrotechnic/Mechanical Release:** A servo-actuated or pyrotechnic quick-release hook on the drone side or the spreader-bar side is triggered.
3.  **Autonomous Egress:** The failing drone detaches to prevent dragging the other 5 drones down or inducing a "tug-of-war" that could destabilize the stretcher.
4.  **Swarm Re-stabilization:** The remaining 5 drones immediately enter a high-gain stabilization mode to compensate for the sudden shift in the force vector.

## 5. Specific Assurance and Integrity Levels (SAIL)
Based on the SORA V2.5 guidelines, the DAS-SAR operation is targeted at **SAIL V/VI**.

*   **Software Assurance:** The use of **Rust** for the flight control loop and distributed coordination ensures memory safety and prevents common concurrency bugs that cause system freezes in C++ based systems.
*   **Independent FTS:** Each drone is equipped with an independent Flight Termination System (FTS) that can be triggered by the Ground Control Station (GCS) or an onboard "heartbeat" monitor, separate from the primary PX4 autopilot.
*   **Containment:** The Emergency Detach protocol serves as a critical containment barrier to limit the effect of an agent failure to a single-unit loss rather than a total swarm loss.
