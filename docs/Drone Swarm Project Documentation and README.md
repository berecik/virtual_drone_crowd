# **Strategic Implementation Framework for Distributed Aerial Search and Rescue Systems: Architecture, Control Dynamics, and Operational Deployment**

## **Executive Summary**

The contemporary landscape of Search and Rescue (SAR) operations is undergoing a profound transformation, precipitated by the convergence of high-performance edge computing, novel propulsion technologies, and advanced autonomous control strategies. Historically, the domain of aerial evacuation has been monopolized by human-crewed rotary-wing aircraft. While effective, helicopters are encumbered by significant capital expenditures, high operational costs per flight hour, and severe operational constraints regarding terrain accessibility and meteorological conditions.1 Furthermore, the deployment of such assets in high-risk environments often endangers the aircrew, creating a paradoxical scenario where rescuers are exposed to the same hazards as the victims.

In parallel, the proliferation of Unmanned Aerial Vehicles (UAVs) has introduced a new capability layer to SAR, primarily in the realms of reconnaissance, thermal detection, and situational awareness. However, a critical logistical gap remains: the physical extraction of a casualty. Current heavy-lift drone solutions capable of lifting a human ($80\\text{kg}+$) typically rely on massive, single-airframe designs with a Maximum Take-Off Mass (MTOM) exceeding $300\\text{kg}$. These platforms suffer from the same transportability and deployment issues as manned helicopters, requiring trailers or flatbed trucks to reach the staging area.1

This report presents a comprehensive technical analysis of a **Distributed Lift System (DLS)**, a paradigm-shifting architecture that utilizes a swarm of smaller, man-portable drones physically tethered to a single payload. By decoupling lift capacity from the size of individual agents, this system allows a coordinated group of four drones, each capable of lifting $30\\text{kg}$, to transport a $100\\text{kg}$ payload.1 This modularity facilitates backpack deployment by ground teams into remote or urban-canyon environments inaccessible to traditional aircraft.

The analysis that follows dissects the theoretical foundations of cooperative aerial manipulation, the specific hardware architecture required for both prototype and full-scale implementation, and the safety-critical software stack built upon **ROS 2**, **Rust**, and **Eclipse Zenoh**. Furthermore, it provides a detailed examination of maritime SAR applications, regulatory compliance via the Specific Operations Risk Assessment (SORA) methodology, and bilingual project documentation for international deployment.

## ---

**1\. Operational Context and Problem Definition**

### **1.1 The Limitations of Legacy SAR Paradigms**

The operational efficacy of SAR missions is often measured against the "Golden Hour"—the critical window following a traumatic injury during which there is the highest likelihood that prompt medical treatment will prevent death. Traditional methods, relying on ground search teams, tracking dogs, and manned helicopters, encounter significant barriers that erode this window. Ground teams are limited by human physiology and terrain roughness, while helicopters are constrained by landing zone availability and weather minimums.1

The integration of small UAVs has digitized the "battlefield for life," acting as flying binoculars that extend the sensor range of the rescue team. However, the current operational model is predominantly "one pilot – one drone." This paradigm generates significant cognitive load for operators, who must manually navigate the aircraft, interpret video feeds, and manage battery levels simultaneously. Under the stress of a rescue operation, this cognitive saturation frequently leads to pilot error or missed detection of the victim.1

### **1.2 The Heavy-Lift Challenge and Distributed Solutions**

While swarm-based reconnaissance—where a single operator commands a fleet of drones to "search sector A"—is maturing, the capability to physically evacuate a casualty remains the "holy grail" of robotic SAR. The payload involved is substantial: an average human male ($80\\text{kg}$), combined with a stretcher ($10\\text{kg}$), medical monitoring equipment ($5\\text{kg}$), and necessary rigging, necessitates a total lift capacity of approximately $115\\text{kg}$.1

Scientific literature and engineering feasibility studies generally categorize heavy-lift solutions into two distinct approaches:

1. **Unified Heavy Transport:** Utilization of a single, large-scale multicopter or helicopter. While mechanically simpler in terms of control theory (a single rigid body), this creates a Single Point of Failure (SPOF). If the propulsion system fails, the payload is lost. Furthermore, the logistical footprint of a drone with a 3-meter rotor span negates the rapid-response advantage of UAVs.1  
2. **Cooperative Payload Transport (Distributed Lift):** Utilization of multiple smaller drones connected to the payload via tethers. This introduces nonlinear dynamic complexities. The payload is not rigidly attached; it is suspended by flexible cables, creating a chaotic pendulum system if not actively damped. However, it offers superior redundancy; if one agent fails, the remaining agents can potentially compensate to lower the payload safely.1

The DLS architecture analyzed in this report adopts the second strategy. It mitigates the inherent instability of slung loads through advanced **Admittance Control** and **Model Predictive Control (MPC)**, distributing the lift requirements across a swarm of $N$ quadrotors.1

## ---

**2\. Theoretical Framework: Cooperative Aerial Manipulation**

### **2.1 Physics of Coupled Slung Load Systems**

The engineering foundation of the DLS rests on the principles of cooperative aerial manipulation. This differs fundamentally from formation flight. In formation flight, agents maintain relative positions using GNSS or vision without physical interaction. In a distributed lift system, the agents are physically coupled through the payload. The state of the payload—its position $P$, velocity $v$, and orientation $\\theta$—is the resultant vector of the tension forces applied by $N$ agents via flexible tethers.1

Research from institutions such as TU Delft and ETH Zurich has established the governing equations for such systems. In a single-drone slung load scenario, the drone must bank (roll/pitch) to accelerate the payload, inevitably inducing a swing (pendulum effect). In a multi-drone system, the internal forces—tension between drones transmitted through the payload—can be manipulated to orient the payload (yaw/pitch/roll) independently of its trajectory.1 This capability is termed "geometric control" and is a critical advantage for maneuvering a stretcher through complex environments, such as lowering a victim through a gap in a forest canopy or navigating between buildings in an urban canyon.

The dynamic model describes the system as a centralized mass connected by massless cables to $N$ quadrotors. The control objective is bifurcated:

1. **Trajectory Tracking:** Moving the payload center of mass along a desired 4D trajectory $P(t)$ (x, y, z, yaw).  
2. **Internal Tension Management:** Ensuring cables remain taut (avoiding slack/snap loads) while minimizing unnecessary internal stress (agents pulling against each other) that wastes battery energy and reduces flight time.1

### **2.2 Control Strategies: Admittance vs. Impedance**

A core engineering challenge identified in the project documentation is the "rigid" nature of traditional PID position controllers used in standard autopilots like PX4 or ArduPilot.1 Standard commercial drones are designed to hold a specific GNSS coordinate with high stiffness.

If four drones attempt to hold a rigid position attached to a shared payload, a GNSS drift of just $10\\text{cm}$ in opposite directions can cause them to fight each other. This "fighting" results in integral windup in the PID controllers, leading to motor saturation, overheating, and potentially tearing the payload apart or causing a catastrophic collision.

The proposed solution utilizes **Admittance Control**. In this control scheme, the drone does not strictly enforce a position. Instead, it is modeled as a virtual mass-spring-damper system.

* **Impedance Control:** The drone resists external forces to maintain position (High Stiffness).  
* **Admittance Control:** The drone "admits" external forces and complies with them (Low Stiffness).

If the tether cable pulls the drone—due to wind acting on the payload or the movement of another drone—the drone detects this force (via load cells or estimator observers) and moves slightly in the direction of the pull, rather than fighting it with maximum thrust.1 This mimics the biological coordination seen in ants moving heavy food items, where individuals adjust their path based on the forces they feel from the group.1

### **2.3 Optimization: Model Predictive Control (MPC)**

To achieve stable flight, each agent runs a non-linear **Model Predictive Control (MPC)** solver on its companion computer. The MPC solver looks ahead into a finite time horizon (e.g., 2 seconds) and calculates the optimal sequence of thrust vectors to minimize the error between the payload's actual position and the target trajectory.

Crucially, the MPC optimization includes hard constraints:

1. **Actuator Limits:** Motors cannot exceed max thrust.  
2. **Cable Tension:** $T \> 0$ (The cable must never go slack).  
3. **Collision Avoidance:** Agents must maintain a minimum separation distance.

Solvers such as **ACADO** or **OSQP** are utilized to solve this optimization problem at rates exceeding $50\\text{Hz}$ on the onboard NVIDIA Jetson computers.1

## ---

**3\. Hardware Architecture and Specification**

The implementation roadmap is divided into two distinct phases to manage technical risk and capital expenditure. Phase 1 focuses on a scaled prototype for software validation, while Phase 2 represents the full-scale operational system.

### **3.1 Phase 1: Scaled Prototype ("Micro" Swarm)**

The objective of Phase 1 is to build a swarm of 3–4 quadcopters capable of lifting a combined $2\\text{--}5\\text{kg}$ dummy payload. This scale enables safe, low-cost iteration of the software stack and sensor-fusion algorithms before committing to high-risk, heavy-lift hardware.1

#### **3.1.1 Airframe: Holybro X500 V2**

The **Holybro X500 V2** is selected as the optimal development platform. It is a $500\\text{mm}$ wheelbase quadcopter featuring high-strength carbon fiber arms and fiber-reinforced nylon connectors.1

* **Structural Integrity:** The rigidity of carbon fiber is crucial for cooperative control; frame flex introduces high-frequency noise into the Inertial Measurement Unit (IMU), which can destabilize the sensitive distributed control loops required for admittance control.1  
* **Modularity:** The X500 V2 includes a dedicated platform board with predrilled mounting holes for companion computers and a rail-mounting system for depth cameras, eliminating the need for complex custom fabrication during the prototyping phase.1

#### **3.1.2 Computational Core: NVIDIA Jetson Orin Nano**

Standard flight controllers (like the Pixhawk) lack the processing power for real-time non-linear optimization and computer vision processing. The swarm logic resides on a powerful companion computer: the **NVIDIA Jetson Orin Nano (8GB)**.1

* **AI Performance:** The Orin Nano delivers up to 40 TOPS (Trillions of Operations Per Second) of AI performance. This compute headroom is utilized for:  
  * **Visual Inertial Odometry (VIO):** Processing stereo depth data to estimate position in GPS-denied environments.  
  * **MPC Solvers:** Running the distributed lift optimization algorithms at $50\\text{Hz}+$.  
  * **Mesh Networking:** Handling high-throughput traffic via the Eclipse Zenoh middleware.  
* **Integration:** To reduce weight and bulk, the system utilizes third-party carrier boards such as the **Seeed Studio reComputer J401** or **Waveshare JETSON-IO-BASE-A**, which offer the necessary interfaces (CSI, M.2 Key E, Gigabit Ethernet) in a form factor suitable for drone integration.1

#### **3.1.3 Flight Controller: Holybro Pixhawk 6C**

While the Jetson handles high-level logic, low-level attitude stabilization is handled by the **Holybro Pixhawk 6C**.1

* **Processor:** Based on the STM32H7 microcontroller running at $480\\text{MHz}$.  
* **Redundancy:** Features triple-redundant IMUs. If one sensor fails or drifts, the flight controller automatically switches to a backup, ensuring flight stability.  
* **Vibration Isolation:** The sensors are mechanically isolated from frame vibrations, crucial for accurate state estimation.  
* **Communication:** Connects to the Jetson via a high-speed UART (MAVLink) connection.1

#### **3.1.4 Perception and Navigation Sensors**

Accurate relative positioning is the cornerstone of swarm stability.

* **GNSS/RTK:** The **CubePilot Here 4** provides centimeter-level Real-Time Kinematic (RTK) precision using multi-band GNSS. In a swarm where drones fly within $1\\text{--}2\\text{m}$ of each other, standard GPS accuracy ($2\\text{--}3\\text{m}$) would result in collisions. The Here 4 operates as a DroneCAN peripheral, ensuring robust data transmission.1  
* **Visual Perception:** The **Intel RealSense D435i** depth camera with an integrated IMU is the industry standard for Visual Inertial Odometry (VIO).1 This allows the drone to hold position optically if the RTK link is jammed or degraded—a critical redundancy for SAR operations in canyons, forests, or near large structures. Alternatively, the **Luxonis OAK-D Pro** is highlighted for its active IR laser projector, allowing depth perception in low-texture environments like snow fields or grassy plains where passive stereo cameras often fail.1

#### **3.1.5 Power Management**

A critical, often overlooked detail in drone integration is powering the companion computer. The Orin Nano requires a regulated DC input. Connecting it directly to the 4S flight battery ($14.8\\text{V}\\text{--}16.8\\text{V}$) poses a risk of damage from voltage spikes caused by regenerative braking from the motors (Active Braking). The solution utilizes the **Matek Systems BEC 12S-PRO**, which accepts $9\\text{--}55\\text{V}$ input and outputs a clean, selectable $5\\text{V}/12\\text{V}$ at $5\\text{A}$.1 This provides ample power headroom for the Jetson (peaking at $15\\text{--}25\\text{W}$) and USB peripherals like the RealSense camera.

**Table 1: Phase 1 Component Summary**

| Subsystem | Component | Key Specification | Justification |
| :---- | :---- | :---- | :---- |
| **Compute** | NVIDIA Jetson Orin Nano | 40 TOPS AI, 8GB RAM | Real-time MPC & VIO processing |
| **Flight Control** | Holybro Pixhawk 6C | STM32H7, Triple IMU | Low-level stability & redundancy |
| **Airframe** | Holybro X500 V2 | Carbon Fiber, 500mm | Rigidity for control precision |
| **Vision** | Intel RealSense D435i | Stereo Depth \+ IMU | GPS-denied navigation (VIO) |
| **GNSS** | CubePilot Here 4 | Multiband RTK | Centimeter-level positioning |
| **Power** | Matek BEC 12S-PRO | 9-55V In, 5A Out | Protection from voltage spikes |

### **3.2 Phase 2: Full-Scale Heavy Lift**

Phase 2 scales the system to lift a human ($80\\text{kg}$) plus stretcher ($10\\text{kg}$), rigging ($5\\text{kg}$), and a safety margin ($20\\%$), totaling approximately $115\\text{kg}$.1 This requires drones in the $50\\text{--}60\\text{kg}$ Maximum Take-Off Mass (MTOM) class.

#### **3.2.1 Propulsion System**

To achieve the necessary lift, the project specifies industrial-grade propulsion:

* **Primary Candidate:** **T-Motor U15 II KV80**, capable of $\\sim 36\\text{kg}$ thrust per motor at $100\\text{V}$.1  
* **Alternative:** **Hobbywing XRotor X9 Plus**, providing $26.5\\text{kg}$ thrust per axis.  
* **Configuration:** An **Octocopter (X8)** coaxial configuration is preferred over a quadcopter. In an X8 setup, two motors are mounted coaxially on each of the four arms. This provides critical redundancy; if one motor or ESC fails, the coaxial partner can compensate, allowing the drone to land safely rather than tumbling out of the sky—a mandatory safety feature for operations involving human life.1

## ---

**4\. Software Architecture: The Modern Safety-Critical Stack**

The operational success of the swarm is entirely dependent on the robustness of its software stack. Traditional robotics stacks (C++/Python on ROS 1\) often suffer from latency jitter, memory safety issues, and communication bottlenecks. This project leverages a modern, safety-critical stack comprising **Rust**, **ROS 2**, and **Eclipse Zenoh**.1

### **4.1 Core Framework: ROS 2 Humble**

**ROS 2 (Robot Operating System 2\)** is chosen for its real-time capabilities and decentralized architecture. Unlike ROS 1, which relied on a central roscore (a single point of failure), ROS 2 allows fully decentralized Peer-to-Peer communication using the Data Distribution Service (DDS) standard.1

* **Quality of Service (QoS):** ROS 2 enables precise definition of data transmission reliability.  
  * **Reliable:** Used for telemetry and control commands to guarantee delivery.  
  * **Best Effort:** Used for high-bandwidth video streams and LiDAR data, where dropping a frame is preferable to introducing latency.1  
* **Lifecycle Management:** ROS 2 introduces Lifecycle Nodes, allowing the system to strictly control the state of drivers (Unconfigured, Inactive, Active, Finalized). This ensures that a drone does not arm its motors until all sensor drivers and safety checks are confirmed active.1

### **4.2 The Paradigm Shift to Rust**

For safety-critical control loops, the project mandates the use of **Rust**. C++ allows for memory management errors (buffer overflows, dangling pointers) and race conditions that can lead to segmentation faults mid-flight. In a heavy-lift scenario, a software crash translates to a $100\\text{kg}$ object falling from the sky.

* **Memory Safety:** Rust’s memory ownership model guarantees memory safety at compile time, eliminating entire classes of bugs without the runtime overhead of a Garbage Collector (as found in Java or Python).1  
* **Implementation:** The stack utilizes ros2\_rust client libraries to write control nodes.  
* **MAVSDK-Rust:** The interface between the high-level ROS 2 logic and the low-level Flight Controller (PX4) is handled by the Rust wrapper for MAVSDK. This allows for the safe and reliable transmission of offboard setpoints (attitude/thrust).1

### **4.3 Communication Middleware: Eclipse Zenoh**

Standard ROS 2 uses DDS as its middleware. However, DDS is primarily designed for stable, wired networks. In wireless swarm scenarios, the discovery traffic mechanism in DDS (multicasting) generates "discovery storms" that can saturate the limited bandwidth of WiFi or LTE links, causing latency spikes and packet loss.1

* **Solution:** The project implements **Eclipse Zenoh**. Zenoh reduces discovery overhead by up to 99% and is optimized for high-performance, low-latency communication over unreliable networks.2  
* **Bridging:** The zenoh-bridge-ros2dds bridges internal ROS 2 topics on each drone to the external swarm network. For example, Drone A can publish its tension vector to /drone\_A/tension, which Drone B subscribes to immediately via Zenoh, transparently across the mesh network.3 This capability is essential for the "Force Consensus" algorithm where drones must agree on the forces applied to the payload.

### **4.4 Distributed Control Algorithm**

The control loop running on the Jetson implements a distributed collaborative transport algorithm:

1. **State Estimation:** Each drone estimates the payload position relative to itself using the tether angle. This angle is measured either visually via the RealSense camera looking down at the tether or via load cells on the tether mount.1  
2. **Force Consensus:** Drones exchange their measured tether forces via Zenoh.  
3. **Optimization:** A local MPC solver calculates the required thrust vector to minimize the difference between the actual payload position and the target trajectory. Crucially, the solver enforces the constraint $T \> 0$ to ensure the tether never goes slack, which would cause a dangerous "snap" load when it re-tensions.  
4. **Actuation:** The computed thrust vector is converted to roll/pitch/yaw/throttle commands and sent to the Pixhawk via MAVLink.

## ---

**5\. Marine SAR Application: Adapting for Maritime Environments**

The user query highlights the specific application of Marine SAR operations. Adapting the DLS for maritime environments introduces specific challenges regarding environment, search patterns, and recovery dynamics.4

### **5.1 Environmental Hardening**

Maritime environments are hostile to electronics.

* **Corrosion Protection:** Phase 2 drones must be IP-rated (IP67). Components like the T-Motor U15 must be treated for corrosion resistance against salt spray.  
* **Wind Resistance:** Sea environments experience higher wind speeds and gusts. The Admittance Control parameters must be tuned to be "stiffer," resisting wind gusts while still remaining compliant enough to handle tether forces.

### **5.2 Dynamic Landing and Recovery**

Landing a drone on a pitching ship deck is one of the most difficult maneuvers in aviation.

* **Heave Compensation:** The system must account for the vertical motion (heave) of the ship deck. The RealSense D435i VIO capabilities must be tuned to track moving landing pads (e.g., AprilTags) on the vessel deck.  
* **Tethered Recovery:** The tether system can be utilized for recovery. Once the payload (or a guide line) is secured to the ship, the winch system can actively pull the swarm down to the deck, mechanically synchronizing the landing with the ship's motion.

### **5.3 Simulation and Search Patterns**

Simulation plays a crucial role in validating maritime operations before deployment. Using **Gazebo** and the **PX4 SITL** (Software In The Loop), developers can simulate specific maritime scenarios:

* **Drifting Models:** Simulating a drowning crew member or life raft drifting with ocean currents and wind (Leeway drift).  
* **Search Efficiency:** Testing different swarm formations (e.g., parallel sweep vs. creeping line) to maximize the probability of detection.4  
* **Visual Simulation:** Rendering sea states, whitecaps, and glare to train the computer vision models (YOLO/OAK-D) for detecting heads or life jackets in the water. The system creates a "Virtual Drone Crowd" to simulate crowd counting or individual detection in these complex visual environments.4

## ---

**6\. Safety Protocols and Regulatory Compliance (SORA)**

Operating a swarm of heavy-lift drones requires strict adherence to aviation safety standards. The project is grounded in the **Specific Operations Risk Assessment (SORA)** methodology used by the European Union Aviation Safety Agency (EASA) and the Polish Civil Aviation Authority (ULC).1

### **6.1 SORA Methodology**

SORA assesses the risk of drone operations in the "Specific" category.

* **Ground Risk Class (GRC):** This quantifies the risk to people on the ground. It is mitigated by operating over controlled areas (SAR zones) and using the distributed tether system as a fail-safe. If one drone fails, the others can support the load, preventing a crash.  
* **Air Risk Class (ARC):** This quantifies the risk of collision with manned aircraft. It is mitigated by operating in segregated airspace or at very low altitudes ($\<120\\text{m}$) typical of SAR operations.

### **6.2 System Redundancy**

* **Decentralization:** The removal of a central control node means the swarm functions as a collective. If the "leader" drone (logically assigned) fails, the swarm automatically reconfigures, and another node takes over the distribution of logic.1  
* **Hardware Redundancy:** The X8 motor configuration ensures flight capability after a motor loss. The Pixhawk 6C features triple-redundant IMUs.  
* **Software Isolation:** Python modules (used for non-critical AI perception) are strictly isolated from the Rust-based control loops. A garbage collection pause in Python will not freeze the flight control loop, ensuring stability is maintained even if the vision system lags.1

### **6.3 Human-Swarm Interaction**

To reduce the cognitive load on the operator, the system moves away from direct teleoperation (joystick control).

* **High-Level Commands:** The operator issues strategic commands like "Search Sector A" or "Evacuate Target at Coordinates X,Y."  
* **Interfaces:**  
  * **GCS (Ground Control Station):** A **React**\-based web application visualizes the swarm status, battery levels, and live video on 3D terrain maps (CesiumJS).1  
  * **Tactical Terminals:** Field rescuers use ruggedized tablets running **Flutter** apps. These apps connect to the swarm via Zenoh (using flutter\_rust\_bridge) to receive telemetry and video without the overhead of a full ROS stack on the tablet.1

## ---

**7\. Project Description Rewrite (Bilingual)**

As requested, the following section rewrites the project description based on the provided documents (virtual\_drone\_crowd, Drone Swarm Evacuation Project Plan) in both Polish and English. This description serves as a comprehensive summary suitable for stakeholders or a repository README.

### **English: Distributed Aerial Swarm for Search and Rescue (DAS-SAR)**

Project Overview  
The DAS-SAR project aims to revolutionize Search and Rescue (SAR) operations by deploying a cooperative swarm of autonomous drones capable of heavy-lift evacuation. Unlike traditional approaches that rely on single, massive airframes which are logistically difficult to transport, this project utilizes a Distributed Lift System (DLS). By tethering multiple smaller, man-portable drones to a single payload (e.g., a stretcher), the system decouples lift capacity from agent size, enabling rapid deployment in inaccessible terrains such as mountains, urban canyons, or maritime environments.  
**Key Objectives**

1. **Autonomous Coordination:** Implementation of decentralized swarm logic where drones coordinate thrust and position to stabilize a suspended payload using **Admittance Control**. This prevents collision and instability caused by GPS drift.  
2. **Scalability:** A modular architecture allowing the swarm size to scale based on the payload weight (e.g., 4 drones for $100\\text{kg}$, 6 drones for $150\\text{kg}$).  
3. **Safety & Redundancy:** Utilization of **Rust** and **ROS 2** for a safety-critical, real-time control stack that eliminates single points of failure. The system relies on **Eclipse Zenoh** for robust mesh networking in communication-denied environments.  
4. **Marine & Terrestrial Application:** Adaptable for mountain rescue and maritime search operations using visual-inertial navigation to operate without reliable GNSS.

**Technical Stack**

* **Hardware:** Holybro X500 V2 (Prototype) / Heavy-Lift X8 Coaxial, NVIDIA Jetson Orin Nano, Pixhawk 6C, RealSense D435i.  
* **Software:** ROS 2 Humble, Eclipse Zenoh (Mesh Networking), Rust (Control Loops), PX4 Autopilot, ACADO/OSQP MPC Solvers.

### ---

**Polski: Rozproszony System Roju Dronów do Poszukiwań i Ratownictwa (DAS-SAR)**

Przegląd Projektu  
Projekt DAS-SAR ma na celu zrewolucjonizowanie operacji poszukiwawczo-ratowniczych (SAR) poprzez wdrożenie kooperacyjnego roju autonomicznych dronów zdolnych do ewakuacji ciężkich ładunków. W przeciwieństwie do tradycyjnych podejść polegających na pojedynczych, masywnych statkach powietrznych, które są trudne w transporcie, ten projekt wykorzystuje Rozproszony System Nośny (DLS). Poprzez fizyczne połączenie wielu mniejszych, przenośnych dronów z jednym ładunkiem (np. noszami), system uniezależnia udźwig od wielkości pojedynczej jednostki, umożliwiając szybkie wdrożenie w trudno dostępnym terenie, takim jak góry, kaniony miejskie czy środowisko morskie.  
**Kluczowe Cele**

1. **Autonomiczna Koordynacja:** Wdrożenie zdecentralizowanej logiki roju, w której drony koordynują ciąg i pozycję w celu stabilizacji podwieszonego ładunku przy użyciu **Sterowania Admitancyjnego (Admittance Control)**. Zapobiega to kolizjom i niestabilności spowodowanej błędem GPS (dryftem).  
2. **Skalowalność:** Modułowa architektura pozwalająca na dostosowanie wielkości roju do wagi ładunku (np. 4 drony dla $100\\text{kg}$, 6 dronów dla $150\\text{kg}$).  
3. **Bezpieczeństwo i Redundancja:** Wykorzystanie języka **Rust** i systemu **ROS 2** w celu stworzenia krytycznego dla bezpieczeństwa stosu sterowania działającego w czasie rzeczywistym, eliminującego pojedyncze punkty awarii. System opiera się na **Eclipse Zenoh** w celu zapewnienia niezawodnej sieci mesh w środowiskach bez zasięgu.  
4. **Zastosowanie Morskie i Lądowe:** Możliwość adaptacji do ratownictwa górskiego oraz morskich operacji poszukiwawczych z wykorzystaniem nawigacji wizyjno-inercyjnej działającej bez niezawodnego sygnału GNSS.

**Stos Technologiczny**

* **Sprzęt:** Holybro X500 V2 (Prototyp) / Heavy-Lift X8 Coaxial, NVIDIA Jetson Orin Nano, Pixhawk 6C, RealSense D435i.  
* **Oprogramowanie:** ROS 2 Humble, Eclipse Zenoh (Sieci Mesh), Rust (Pętle Sterowania), PX4 Autopilot, Solvery MPC ACADO/OSQP.

## ---

**8\. GitHub Repository Documentation (README.md)**

The following text serves as the README.md for the GitHub repository virtual\_drone\_crowd, incorporating the technical depth and specific usage examples required.

# **Virtual Drone Crowd: Distributed Aerial SAR System**

(https://img.shields.io/badge/ROS\_2-Humble-blue.svg)\](https://docs.ros.org/en/humble/)  
(https://img.shields.io/badge/Language-Rust-orange.svg)\](https://www.rust-lang.org/)

## **📖 Overview**

**Virtual Drone Crowd** is a research and development framework focused on the simulation and deployment of **Distributed Lift Systems (DLS)** for Search and Rescue (SAR) operations. The project implements a decentralized swarm architecture where multiple autonomous drones coordinate to lift and transport heavy payloads (e.g., human casualties) via flexible tethers.

This repository bridges the gap between simulation and reality, providing the source code for swarm control logic (Rust), simulation environments (Gazebo/PX4), and high-performance communication middleware based on **Eclipse Zenoh** and **ROS 2**.

## **🚀 Key Features**

* **Distributed Admittance Control:** Implements a mass-spring-damper model for each drone, allowing the swarm to stabilize slung loads without rigid position fighting or integral windup.  
* **Mesh Networking (Zenoh):** Utilizes **Eclipse Zenoh** for high-performance, low-latency communication. This enables the system to operate in GPS-denied and WiFi-congested environments by reducing discovery traffic by up to 99%.  
* **Safety-Critical Core:** Control loops are implemented in **Rust** to guarantee memory safety (no segfaults) and real-time performance.  
* **Marine & Terrain SAR:** Specialized simulation scenarios for maritime drift search and urban canyon evacuation.  
* **Scalable Architecture:** Supports heterogeneous swarms (e.g., a mix of scouting micro-drones and heavy-lift transport units).

## **🛠️ System Architecture**

### **Hardware Stack (Phase 1 Prototype)**

* **Compute:** NVIDIA Jetson Orin Nano (8GB) \- Handling MPC and VIO.  
* **Flight Control:** Holybro Pixhawk 6C (PX4 Autopilot) \- Handling attitude control.  
* **Sensors:** Intel RealSense D435i (Visual Inertial Odometry), CubePilot Here 4 (RTK GNSS).  
* **Frame:** Holybro X500 V2.

### **Software Stack**

* **Middleware:** ROS 2 Humble \+ zenoh-bridge-ros2dds  
* **Simulation:** Gazebo Garden \+ PX4 SITL (Software In The Loop)  
* **Optimization:** ACADO/OSQP MPC Solvers running at 50Hz.

## **📦 Installation**

### **Prerequisites**

* Ubuntu 22.04 LTS  
* ROS 2 Humble Hawksbill  
* Rust Toolchain (rustup)  
* Docker & NVIDIA Container Toolkit (required for simulation)

### **Build Instructions**

1. \*\*Clone the repository:\*\*bash  
   git clone  
   cd virtual\_drone\_crowd  
2. **Install system dependencies:**  
3. **Build Rust Control Nodes:**  
4. **Build ROS 2 packages:**

## **🖥️ Usage Examples**

### **1\. Launching a Marine SAR Simulation**

This scenario simulates a swarm searching for a drifting target (e.g., a life raft) in a maritime environment. It utilizes a "creeping line" search pattern.

### **2\. Testing Distributed Lift (Admittance Control)**

This scenario simulates the physical extraction of a payload. It validates the force consensus algorithm where drones share tension data to lift synchronously.

## **📂 Project Structure**

* src/control\_core: **Rust**\-based MPC solvers and admittance controllers. Contains the safety-critical loop.  
* src/zenoh\_bridge: Configuration files for the **Zenoh** mesh networking bridge.  
* simulation/worlds: Gazebo worlds including **Marine** (ocean waves), **Urban** (buildings), and **Forest**.  
* simulation/models: URDF/SDF models for X500 prototype and Phase 2 X8 Coaxial drones.  
* docs: Technical reports, SORA risk assessments, and architecture diagrams.

## **🤝 Contributing**

We welcome contributions, especially in the areas of MPC solver optimization and maritime simulation fidelity. Please read CONTRIBUTING.md for details on our code of conduct and the process for submitting pull requests.

## **📜 License**

This project is licensed under the Apache 2.0 License \- see the LICENSE file for details.

#### **Works cited**

1. Drone Swarm Evacuation Project Plan.pdf  
2. Integrating ROS2 with Eclipse zenoh, accessed on January 2, 2026, [https://zenoh.io/blog/2021-04-28-ros2-integration/](https://zenoh.io/blog/2021-04-28-ros2-integration/)  
3. eclipse-zenoh/zenoh-plugin-ros2dds: A Zenoh plug-in for ROS2 with a DDS RMW. See https://discourse.ros.org/t/ros-2-alternative-middleware-report/ for the advantages of using this plugin over other DDS RMW implementations. \- GitHub, accessed on January 2, 2026, [https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds)  
4. Virtual Simulation of the Maritime Search Operation for Drowning Crew \- ResearchGate, accessed on January 2, 2026, [https://www.researchgate.net/publication/337302697\_Virtual\_Simulation\_of\_the\_Maritime\_Search\_Operation\_for\_Drowning\_Crew](https://www.researchgate.net/publication/337302697_Virtual_Simulation_of_the_Maritime_Search_Operation_for_Drowning_Crew)