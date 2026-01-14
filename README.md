# DAS-SAR: Distributed Aerial Search and Rescue Swarm

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble%2FJazzy-blue.svg)](https://docs.ros.org/en/humble/)
[![Language](https://img.shields.io/badge/Language-Rust-orange.svg)](https://www.rust-lang.org/)
[![Language](https://img.shields.io/badge/Language-Python-blue.svg)](https://www.python.org/)
[![Middleware](https://img.shields.io/badge/Middleware-Zenoh-green.svg)](https://zenoh.io/)

## 📖 Overview

**DAS-SAR** (Distributed Aerial Search and Rescue) represents a paradigm shift from traditional single-point-of-failure aerial evacuation (e.g., helicopters) to a **Fail-Operational Distributed Lift System (DLS)**. Instead of relying on one massive aircraft, we utilize a swarm of autonomous heavy-lift drones tethered to a single payload.

This system is designed not just for searching and reconnaissance, but for **redundant heavy lifting**. By decoupling lift capacity from individual airframe size, the DAS-SAR swarm can extract human casualties (100kg+ payloads) from environments inaccessible to conventional aircraft, such as dense forests, deep canyons, or urban disaster zones.

---

## 📐 System Visualization

> **[PLACEHOLDER: Diagram showing a 6-agent swarm tethered to a single stretcher payload, illustrating the geometric authority and tether geometry.]**

---

## 🚀 Key Features

*   **Fail-Operational Distributed Lift:** The swarm survives the total loss of an agent mid-mission without dropping the payload.
*   **6-Agent Redundancy:** Phase 2 utilizes a minimum of 6 heavy-lift drones (coaxial octocopters). This provides the necessary geometric authority to maintain 6-DOF control even in a failure state.
*   **Dynamic Agent Replacement:** Supports "Hot-Swap" logic where reserve drones can swap out with depleted or failing agents mid-mission, ensuring continuous 24/7 operation.
*   **Distributed Admittance Control:** Implements mass-spring-damper models to stabilize slung loads and manage internal tether tension without rigid position fighting.
*   **Mesh Networking (Zenoh):** Leveraging **Eclipse Zenoh** for ultra-low latency swarm coordination, significantly outperforming standard DDS in congested or wide-area environments.

## 🏗 Project Structure

```text
.
├── docker/                 # Zenoh configuration and docker setups
├── docs/                   # Project plans, Technical studies, and SORA analysis
├── sar_swarm_ws/           # ROS 2 Workspace
│   └── src/
│       ├── heavy_lift_core/# Core swarm lift logic (Rust)
│       ├── px4_msgs/       # PX4-ROS 2 message definitions
│       ├── sar_perception/ # AI/Vision nodes (Detection & Localization - Python)
│       ├── sar_simulation/ # Swarm simulation and test scripts
│       └── sar_swarm_control/ # Distributed control algorithms (Rust)
├── Dockerfile              # Development environment container
├── docker-compose.yml      # Multi-container orchestration
├── README.md               # This file
├── README_PL.md            # Documentation in Polish
├── README_UA.md            # Documentation in Ukrainian
├── README_HE.md            # Documentation in Hebrew
├── docs/Technical Architecture.md # Detailed technical architecture
├── ROADMAP.md              # Project timeline and milestones
└── AGENTS.md               # Technical Context & Development Guide
```

## 🛠 Tech Stack

| Component | Technology |
| :--- | :--- |
| **Safety-Critical Control** | **Rust** (rclrs, MAVSDK-Rust) |
| **AI & Computer Vision** | **Python** (PyTorch, YOLOv8/11) |
| **Middleware** | **Eclipse Zenoh** & **ROS 2** (Humble/Jazzy) |
| **Simulation** | Gazebo Harmonic / PX4 SITL |
| **Hardware (Ph 1)** | NVIDIA Jetson Orin Nano, Pixhawk 6C, Holybro X500 V2 |
| **Hardware (Ph 2)** | T-Motor U15 II / Hobbywing X9 Plus (Coaxial X8) |

## 🚦 Getting Started

### Prerequisites
- Docker & Docker Compose
- Ubuntu 22.04 LTS (recommended)
- ROS 2 Humble/Jazzy
- Rust Toolchain

### Running Simulation
The project includes a Docker-based simulation for testing the Rust-based swarm behavior.

1. **Build the Docker environment:**
   ```bash
   docker-compose build
   ```

2. **Launch the swarm simulation:**
   ```bash
   docker-compose up
   ```

3. **Visualize on host:**
   ```bash
   python3 visualize_on_host.py
   ```

## 🧪 Development

### Running Unit Tests
To verify the core control logic:
```bash
cd sar_swarm_ws/src/sar_swarm_control
cargo test
```

## 📖 Documentation

For detailed information, please refer to:
- [Project Plan v2.0](docs/DAS-SAR%20Project%20Plan%20v2.0_%20Distributed%20Heavy-Lift%20Swarm.md) - Strategy and Phase 2 specs.
- [Technical Architecture](docs/Technical%20Architecture.md) - Deep dive into control theory and system design.
- [SORA Safety Case](docs/Safety_Case_SORA.md) - Risk analysis for human extraction.
- [Roadmap](ROADMAP.md) - Development timeline.

## 👥 Authors & Contact

- **beret** - [beret@hipisi.org.pl](mailto:beret@hipisi.org.pl)
- **Marysia Software Limited** - [ceo@marysia.app](mailto:ceo@marysia.app)
- **Website:** [https://marysia.app](https://marysia.app)

---

## ⚖️ Legal & Safety
Operations involving heavy-lift swarms and human transport are subject to EASA Specific/Certified category regulations. All operations must follow the [SORA](docs/Safety_Case_SORA.md) protocols defined in the documentation.
