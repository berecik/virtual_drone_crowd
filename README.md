# Virtual Drone Crowd: Autonomous SAR Swarm & Evacuation System

This project aims to develop a distributed autonomous drone swarm system for Search and Rescue (SAR) operations, specifically focusing on two phases: rapid reconnaissance (Phase 1) and physical human evacuation (Phase 2).

## 🚀 Overview

The system leverages a "Swarm Intelligence + Heavy Lift" paradigm. Small, agile drones (Scouts) perform autonomous area search and victim localization using AI, while heavy-lift platforms (Extractors) handle the physical transport of casualties.

### Key Features
- **Decentralized Control:** Built on ROS 2 and Rust for deterministic, safety-critical swarm coordination.
- **Edge AI:** On-board human detection using NVIDIA Jetson and depth-sensing cameras (OAK-D / RealSense).
- **Efficient Communication:** Utilizes Eclipse Zenoh for low-latency, low-bandwidth swarm networking.
- **Hybrid Architecture:** Combines high-performance Rust for control loops with Python for AI and rapid prototyping.

## 🏗 Project Structure

```text
.
├── docker/                 # Zenoh configuration and docker setups
├── docs/                   # Original technical studies and project plans
├── sar_swarm_ws/           # ROS 2 Workspace
│   └── src/
│       ├── px4_msgs/       # PX4-ROS 2 message definitions
│       ├── sar_perception/ # AI/Vision nodes (Detection & Localization)
│       ├── sar_simulation/ # Python-based swarm simulation and test scripts
│       └── sar_swarm_control/ # Core swarm control logic (Rust)
├── Dockerfile              # Development environment container
├── docker-compose.yml      # Multi-container orchestration (Drone nodes + Zenoh)
├── README.md               # This file
├── SYSTEM_DOCUMENTATION.md # Detailed technical architecture
├── WORKING_PLAN.md         # Phase-by-phase implementation tasks
└── ROADMAP.md              # Project timeline and milestones
```

## 🛠 Tech Stack

| Component | Technology |
| :--- | :--- |
| **Core Framework** | ROS 2 (Humble/Jazzy) |
| **Swarm Control** | Rust (rclrs) |
| **Edge AI/Vision** | Python (PyTorch/YOLOv8) |
| **Middleware** | Eclipse Zenoh |
| **Flight Stack** | PX4 Autopilot |
| **Hardware** | NVIDIA Jetson Orin, Pixhawk 6, OAK-D Pro |
| **Frontend** | React (GCS), Flutter (Tactical Mobile) |

## 🚦 Getting Started

### Prerequisites
- Docker & Docker Compose
- Python 3.10+ (for local visualization)

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
   (Ensure you have the required Python dependencies installed on your host).

## 🧪 Development

### Running Unit Tests
To verify the Boids logic:
```bash
cd sar_swarm_ws/src/sar_swarm_control
cargo test
```

## 📖 Documentation

For detailed information, please refer to:
- [System Documentation](SYSTEM_DOCUMENTATION.md)
- [Working Plan](WORKING_PLAN.md)
- [Roadmap](ROADMAP.md)
- [Original Technical Study (Polish)](docs/Projekt%20Dronów%20SAR_%20Ewakuacja%20Człowieka.md)

## 👥 Authors & Contact

- **beret** - [beret@hipisi.org.pl](mailto:beret@hipisi.org.pl)
- **Marysia Software Limited** - [ceo@marysia.app](mailto:ceo@marysia.app)
- **Website:** [https://marysia.app](https://marysia.app)

---

## ⚖️ Legal & Safety
Operations involving heavy-lift swarms and human transport are subject to EASA Specific/Certified category regulations and require SORA (Specific Operations Risk Assessment). See [System Documentation](SYSTEM_DOCUMENTATION.md#regulatory-compliance) for details.
