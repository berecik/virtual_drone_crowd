# Project Roadmap: Virtual Drone Crowd

## 🗓 Timeline Overview

| Milestone | Target Date | Status | Description |
| :--- | :--- | :--- | :--- |
| **M1: Swarm Foundation** | Month 3 | 🟡 In Progress | Basic swarm formation and Zenoh comms in simulation. |
| **M2: Perception PoC** | Month 6 | ⚪ Not Started | Autonomous human detection and 3D localization. |
| **M3: Scout Hardware Ready** | Month 8 | ⚪ Not Started | Flight tests of 4x X500 drones with Jetson Orin Nano. |
| **M4: Heavy Lift Prototype** | Month 12 | ⚪ Not Started | First lift of 80kg dummy load using Octocopter. |
| **M5: Integrated SAR Demo** | Month 15 | ⚪ Not Started | End-to-end "Search & Rescue" mission demonstration. |
| **M6: Certification & Deployment** | Month 18 | ⚪ Not Started | SORA approval and operational readiness for emergency services. |

## 📍 Current Status (Month 1-2)
- ✅ Initial technical study and system architecture defined.
- ✅ Dockerized ROS 2 workspace established.
- ✅ Basic swarm control node (Rust) and simulation (Python) implemented.
- 🟡 Implementing Zenoh-based decentralized communication.

## 🚀 Future Milestones

### Q1-Q2 2026: The Scouting Brain
- Finalize Rust-based swarm coordination logic.
- Integrate YOLOv8 on Jetson for victim detection.
- Complete 3-drone formation flight in high-fidelity simulation.

### Q3-Q4 2026: The Physical Lift
- Assembly of the "Heavy Lifter" platform (Foxtech Gaia 160MP).
- Development of the Admittance Control algorithm for payload stabilization.
- Indoor tethered flight tests.

### 2027: Mission Readiness
- Integration of the Ground Control Station (React) and Mobile Tactical App (Flutter).
- SORA certification process with ULC (Poland).
- Field exercises with SAR teams (GOPR/TOPR).
