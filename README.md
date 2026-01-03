# Virtual Drone Crowd: Distributed Aerial SAR System

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Language](https://img.shields.io/badge/Language-Rust-orange.svg)](https://www.rust-lang.org/)

## 📖 Overview

**Virtual Drone Crowd** is a research and development framework focused on the simulation and deployment of **Distributed Lift Systems (DLS)** for Search and Rescue (SAR) operations. The project implements a decentralized swarm architecture where multiple autonomous drones coordinate to lift and transport heavy payloads (e.g., human casualties) via flexible tethers.

This repository bridges the gap between simulation and reality, providing the source code for swarm control logic (Rust), simulation environments (Gazebo/PX4), and high-performance communication middleware based on **Eclipse Zenoh** and **ROS 2**.

### 🇵🇱 Przegląd Projektu (Polish)
Projekt DAS-SAR ma na celu zrewolucjonizowanie operacji poszukiwawczo-ratowniczych (SAR) poprzez wdrożenie kooperacyjnego roju autonomicznych dronów zdolnych do ewakuacji ciężkich ładunków. Dzięki wykorzystaniu **modelu Boids**, algorytmu **Raft Consensus** oraz sieci **Zenoh**, system zapewnia bezpieczną koordynację i transport osób w trudno dostępnych terenach (góry, morze, miasta).

## 🚀 Key Features

*   **Distributed Admittance Control:** Implements a mass-spring-damper model for each drone, allowing the swarm to stabilize slung loads without rigid position fighting.
*   **Swarm Intelligence:** Emergent flocking behaviors based on the **Boids Model** (Separation, Alignment, Cohesion) and dynamic area partitioning.
*   **Consensus & Coordination:** Uses the **Raft Consensus** algorithm for reliable state agreement and leader election across the swarm.
*   **Mesh Networking (Zenoh):** Utilizes **Eclipse Zenoh** for high-performance, low-latency communication, reducing discovery overhead by 99% in WiFi-congested environments.
*   **Safety-Critical Core:** Control loops implemented in **Rust** to guarantee memory safety and real-time performance.
*   **Tactical Interfaces:** Multi-platform interaction via **React GCS** (3D visualization) and **Flutter Tactical Terminals** (mobile field coordination).
*   **Marine & Terrain SAR:** Specialized support for maritime drift search (heave compensation) and urban canyon evacuation.

## 🏗 Project Structure

```text
.
├── docker/                 # Zenoh configuration and docker setups
├── docs/                   # Technical studies and project documentation
├── sar_swarm_ws/           # ROS 2 Workspace
│   └── src/
│       ├── px4_msgs/       # PX4-ROS 2 message definitions
│       ├── sar_perception/ # AI/Vision nodes (Detection & Localization)
│       ├── sar_simulation/ # Swarm simulation and test scripts
│       └── sar_swarm_control/ # Core swarm control logic (Rust)
├── Dockerfile              # Development environment container
├── docker-compose.yml      # Multi-container orchestration
├── README.md               # This file
├── SYSTEM_DOCUMENTATION.md # Detailed technical architecture
└── ROADMAP.md              # Project timeline and milestones
└── TESTING.md              # Testing procedures
```

## 🛠 Tech Stack

| Component | Technology |
| :--- | :--- |
| **Core Framework** | ROS 2 (Humble) |
| **Swarm Control** | Rust (rclrs / MAVSDK-Rust), Raft Consensus |
| **Edge AI/Vision** | Python (PyTorch/YOLOv8), Luxonis OAK-D / RealSense |
| **Middleware** | Eclipse Zenoh (Mesh Networking) |
| **User Interfaces** | React (GCS), Flutter (Tactical Terminal) |
| **Hardware (Ph 1)** | NVIDIA Jetson Orin Nano, Pixhawk 6C, Holybro X500 V2 |
| **Hardware (Ph 2)** | T-Motor U15 II, Hobbywing X9 Plus, Gaia 160MP |
| **Optimization** | ACADO/OSQP MPC Solvers |

## 🚦 Getting Started

### Prerequisites
- Docker & Docker Compose
- Ubuntu 22.04 LTS (recommended for local development)
- ROS 2 Humble
- Rust Toolchain

### Running Simulation
The project includes a Docker-based simulation for testing the Rust-based Boids swarm behavior.

1. **Build the Docker environment:**
   ```bash
   docker-compose build
   ```

2. **Launch the swarm simulation:**
   ```bash
   docker-compose up
   ```
   This will start 3 SITL drones and 3 swarm nodes running the Rust Boids algorithm.

3. **Visualize on host:**
   ```bash
   python3 visualize_on_host.py
   ```

## 🧪 Development

### Running Unit Tests
To verify the Boids logic:
```bash
cd sar_swarm_ws/src/sar_swarm_control
cargo test
```

## 📖 Documentation

For detailed information, please refer to:
- [System Documentation (English)](SYSTEM_DOCUMENTATION.md) - Deep dive into architecture and control theory.
- [Dokumentacja Systemu (Polski)](SYSTEM_DOCUMENTATION_PL.md) - Pełna dokumentacja techniczna i sprzętowa.
- [Roadmap](ROADMAP.md) - Project timeline and milestones.
- [Testing](TESTING.md) - Detailed testing instructions.
- [Original Technical Study (Polish)](docs/Projekt%20Dronów%20SAR_%20Ewakuacja%20Człowieka.md)

## 👥 Authors & Contact

- **beret** - [beret@hipisi.org.pl](mailto:beret@hipisi.org.pl)
- **Marysia Software Limited** - [ceo@marysia.app](mailto:ceo@marysia.app)
- **Website:** [https://marysia.app](https://marysia.app)

---

## ⚖️ Legal & Safety
Operations involving heavy-lift swarms and human transport are subject to EASA Specific/Certified category regulations and require SORA (Specific Operations Risk Assessment). See [System Documentation](SYSTEM_DOCUMENTATION.md#6-safety-protocols-and-regulatory-compliance-sora) for details.
