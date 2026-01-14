# **Strategic Implementation Plan for Distributed Aerial Search and Rescue Systems: From Prototype to Heavy-Lift Human Evacuation**

[Languages]: [EN](Drone%20Swarm%20Evacuation%20Project%20Plan.md) | [PL](Drone%20Swarm%20Evacuation%20Project%20Plan_PL.md) | [UA](Drone%20Swarm%20Evacuation%20Project%20Plan_UA.md)

## **1\. Introduction and Operational Context**

The domain of Search and Rescue (SAR) is currently undergoing a paradigm shift, driven by the rapid miniaturisation of avionics and the maturation of autonomous control strategies. Traditional aerial evacuation relies almost exclusively on human-crewed rotary-wing aircraft—helicopters—which are capital-intensive, operationally expensive, and severely constrained by terrain and weather. While small Unmanned Aerial Vehicles (UAVs) have been successfully integrated for reconnaissance and thermal detection, the capability to extract a casualty physically remains a logistical gap. A single heavy-lift drone capable of carrying a human (80 kg+) requires a massive airframe (300 kg+ MTOM), presenting transportability issues similar to those of human-crewed aircraft.

The solution proposed in this report is a **Distributed Lift System (DLS)**: a swarm of smaller, man-portable drones physically tethered to a single payload (the casualty on a stretcher). This approach decouples lift capacity from individual agent size. Four drones, each capable of lifting 30kg, can theoretically transport a 100kg payload when coordinated effectively. This modularity allows the system to be backpack-deployed by ground teams and assembled on-site in remote or urban-canyon environments where helicopters cannot land.

However, the transition from a single rigid-body vehicle to a multi-agent coupled system introduces nonlinear dynamic complexities. The payload is not rigidly attached; it is suspended by flexible cables, creating a chaotic pendulum system if not actively damped. Furthermore, the failure of a single agent must not result in catastrophic loss of the payload.

This report outlines a two-phase engineering roadmap to realise this capability. **Phase 1** focuses on a scaled prototype using the Holybro X500 V2 platform and NVIDIA Jetson Orin Nano compute modules to validate the distributed control algorithms. **Phase 2** translates these validated architectures into a full-scale heavy-lift system utilising industrial propulsion (T-Motor/Hobbywing) and complying with EASA Specific Category regulations.

## **2\. Theoretical Framework: Cooperative Aerial Manipulation**

The engineering foundation of this project rests on the principles of cooperative aerial manipulation. Unlike formation flight, where agents maintain relative positions without physical interaction, a distributed lift system involves significant physical coupling. The state of the payload (position, velocity, orientation) is the resultant vector of the tension forces applied by $N$ agents.

### **2.1 The Physics of Slung Load Swarms**

Research from TU Delft and ETH Zurich has established the governing equations for such systems.1 In a single-drone slung load, the drone must bank to accelerate the payload, inducing a swing. In a multi-drone system, internal forces (tension between drones via the payload) can be controlled to orient the payload (yaw/pitch/roll) independently of its trajectory. This is a critical advantage for manoeuvring a stretcher through a forest canopy.3

The dynamic model can be described as a centralised mass connected by massless cables to $N$ quadrotors. The control objective is twofold:

1. **Trajectory Tracking:** Moving the payload centre of mass along a desired path $P(t)$.  
2. **Internal Tension Management:** Ensuring cables remain taut (avoiding slack/snap loads) while minimising unnecessary internal stress that wastes energy.4

### **2.2 Control Strategies: Admittance vs. Impedance**

A core challenge is the "rigid" nature of traditional PID position controllers used in standard autopilots (PX4/ArduPilot). If four drones try to hold a rigid position using GNSS, GPS drift of just 10cm can cause them to collide, tearing the payload apart or burning out their motors.

The proposed solution utilises **Admittance Control**. In this scheme, the drone does not strictly enforce a position. Instead, it acts as a mass-spring-damper system. If the cable pulls the drone (due to wind on the payload or another drone's movement), the drone "admits" this force and moves slightly, rather than fighting it with maximum thrust. This mimics the biological coordination seen in ants moving heavy food items.6 The NVIDIA Jetson compute module on each drone will run a non-linear Model Predictive Control (MPC) solver that calculates the optimal thrust vector to stabilise the payload while respecting these admittance constraints, sending high-level attitude commands to the flight controller.3

## **3\. Phase 1: Scaled Prototype (The "Micro" Swarm)**

Phase 1 is a risk-reduction campaign. The objective is to build a swarm of 3-4 quadcopters capable of lifting a 2-5kg dummy payload. This scale enables safe, low-cost iteration of the software stack (Rust/ROS 2\) and sensor-fusion algorithms (VIO \+ RTK) before committing to high-risk, heavy-lift hardware.

### **3.1 Airframe Selection: Holybro X500 V2**

The **Holybro X500 V2** is selected as the optimal development platform. It is a 500mm wheelbase carbon fibre quadcopter explicitly designed for research and development.

* **Structural Integrity:** The X500 V2 features high-strength carbon fibre arms with fibre-reinforced nylon connectors.7 This rigidity is crucial for precise control; frame flex introduces noise into the IMU, destabilising the sensitive distributed control loops.  
* **Modularity:** The frame includes a dedicated platform board with predrilled mounting holes for companion computers (Raspberry Pi/Jetson Nano) and a rail-mounting system for depth cameras.8 This eliminates the need for complex custom fabrication in the early stages.  
* **Maintainability:** As a kit, parts are easily replaceable. The "Almost Ready to Fly" (ARF) version comes with pre-installed motors and ESCs, significantly reducing assembly time.7

### **3.2 Computational Architecture: NVIDIA Jetson Orin Nano**

Standard flight controllers (Pixhawk) lack the processing power for real-time non-linear optimisation and vision processing. The swarm logic will reside on a companion computer: the **NVIDIA Jetson Orin Nano (8GB)**.

* **Performance:** The Orin Nano delivers up to 40 TOPS (Trillions of Operations Per Second) of AI performance.10 This is sufficient to run:  
  * **Visual Inertial Odometry (VIO):** Processing RealSense depth data to estimate position without GPS (essential for robustness).  
  * **MPC Solvers:** Running the ACADO or OSQP solvers for the distributed lift algorithm at 50Hz+.  
  * **Zenoh Bridge:** Handling high-throughput mesh networking traffic.  
* **Carrier Board:** The standard developer kit is bulky. For the drone, we recommend a third-party carrier board like the **Seeed Studio reComputer J401** or the **Waveshare JETSON-IO-BASE-A**. These offer the same interfaces (CSI, M.2 Key E, Gigabit Ethernet) in a smaller, lighter form factor suitable for integration into the X500 deck.11

#### **3.2.1 Power Distribution Challenges**

A critical integration detail often overlooked is powering the Jetson. The Orin Nano requires a regulated DC input (5V-12V, depending on the carrier).

* **Risk:** Connecting the Jetson directly to a 4S flight battery (14.8V-16.8V) is risky if the carrier board's regulation is insufficient to handle voltage spikes from motor braking (regenerative EMF). .13  
* **Solution:** A high-quality **Battery Eliminator Circuit (BEC)** is mandatory. The **Matek Systems BEC 12S-PRO** is recommended. It accepts 9-55V input and outputs a clean, selectable 5V/12V at 5A.14 This provides ample headroom for the Jetson (which peaks around 15-25W) and peripherals like the RealSense camera 16

### **3.3 Sensor Suite & Avionics**

* **Flight Controller:** **Holybro Pixhawk 6C**. Based on the STM32H7 processor, it offers triple-redundant IMUs and vibration isolation. It communicates with the Jetson via UART (MAVLink high-speed link).17  
* **GNSS/RTK:** **CubePilot Here 4**. Standard GPS accuracy (2-3m) is insufficient for swarm formation flight, where drones may be only 1-2 meters apart. The Here four provides RTK (Real-Time Kinematic) precision (centimetre-level) and acts as a droneCAN peripheral, freeing up serial ports.19  
* **Visual Perception:** **Intel RealSense D435i**. This depth camera with an integrated IMU is the industry standard for VIO. It will be mounted on the X500's rail system using a 3D-printed downward-facing, forward-facing mount (STLs are available on Thingiverse/Printables).21 The D435i allows the drone to hold position optically if the RTK link is jammed or degraded.

### **3.4 Phase 1 Bill of Materials and Costs**

The following table shows the estimated unit cost for the Phase 1 prototype. A minimum of 3 drones is required for stability testing (triangular configuration), with four being optimal for redundancy.

| Component | Specification | Unit Cost (€) | Qty | Total (€) | Source |
| :---- | :---- | :---- | :---- | :---- | :---- |
| **Airframe** | Holybro X500 V2 ARF Kit (w/ Motors/ESCs) | €380 | 1 | €380 | 23 |
| **Flight Controller** | Holybro Pixhawk 6C (Plastic Case) | €215 | 1 | €215 | 18 |
| **GNSS Module** | CubePilot Here 4 Multiband RTK | €285 | 1 | €285 | 20 |
| **Companion Computer** | NVIDIA Jetson Orin Nano 8GB Module \+ Carrier | €450 | 1 | €450 | 12 |
| **Depth Camera** | Intel RealSense D435i | €415 | 1 | €415 | 21 |
| **Telemetry Radio** | SiYi MK15 Air Unit (Video \+ Data \+ RC) | €550 | 1 | €550 | 24 |
| **Power Regulator** | Matek BEC 12S-PRO (5V/12V 5A) | €15 | 1 | €15 | 15 |
| **Flight Battery** | 4S 5000mAh 60C LiPo (Gens Ace/Tattu) | €65 | 2 | €130 | 7 |
| **Mounting Hardware** | 3D Printed Mounts (PETG/ABS), Cables | €50 | 1 | €50 | 7 |
| **Unit Total** |  |  |  | **€2,490** |  |
| **Swarm Total (4 Units)** |  |  |  | **€9,960** |  |

*Note: Costs are approximate based on European retailers (MyBotShop, DronePartsCenter, etc.) and exclude shipping/taxes.*

### **3.5 Phase 1 Integration Strategy**

1. **Mechanical Assembly:** Assemble the X500 frame. Install the Pixhawk 6C on the vibration-dampened deck.  
2. **Jetson Mounting:** Print a custom carrier board mount that clips onto the X500.7's 10mm rail system. Ensure the RealSense camera is rigidly mounted to the frame, not the flight controller, to avoid sensor aliasing.  
3. **Power Wiring:** Solder the Matek BEC input to the PDB pads. Connect the BEC output (12V or 5V, depending on carrier) to the Jetson's DC barrel jack or GPIO power pins. *Critical:* Add a capacitor to the input side of the BEC to smooth voltage spikes from the ESCs.  
4. **Software Setup:** Flash PX4 Autopilot to the Pixhawk. Configure the Jetson with Ubuntu 20.04/22.04 and install the ROS 2 Humble environment 25

## **4\. Software Architecture: Rust, ROS 2, and Zenoh**

The success of the swarm depends entirely on the software stack. Traditional stacks (C++/Python on ROS 1\) suffer from latency, memory safety issues, and communication bottlenecks. This project will leverage a modern, safety-critical stack.

### **4.1 Core Framework: ROS 2 Humble**

ROS 2 (Robot Operating System 2\) is chosen for its real-time capabilities and decentralised architecture (no ROS Master).

* **Communication Middleware:** While DDS is the default, we will utilise **Eclipse Zenoh**. Zenoh is designed for high-performance, low-latency communication over unreliable networks (such as WiFi mesh networks in the field). It outperforms DDS in swarm scenarios by reducing discovery traffic overhead.27  
* **Bridge:** The zenoh-bridge-ros2dds will bridge the internal ROS 2 topics of each drone to the external swarm network, allowing Drone A to publish its tension vector /drone\_A/tension, which Drone B can subscribe to immediately.27

### **4.2 Control Logic: The Shift to Rust**

For the safety-critical control loops, **Rust** is the language of choice. C++ allows for memory leaks and race conditions that can lead to segmentation faults mid-flight—catastrophic in a heavy-lift scenario. Rust's memory ownership model guarantees memory safety at compile time.29

* **Implementation:** We will use ros2\_rust client libraries to write the control nodes.31  
* **MAVSDK-Rust:** The interface between the high-level ROS 2 logic and the low-level Flight Controller (PX4) will be handled by the Rust wrapper for MAVSDK. This allows the safe and reliable transmission of offboard setpoints (attitude/thrust).32

### **4.3 Control Algorithm: Distributed Admittance**

The control loop running on the Jetson will implement the TU Delft collaborative transport algorithm.3

1. **State Estimation:** Each drone estimates the payload position relative to itself using the tether angle (measured via load cells or visually via the RealSense looking down at the tether).  
2. **Force Consensus:** Drones exchange their measured tether forces via Zenoh.  
3. **Optimisation:** A local MPC solver calculates the required thrust vector to minimise the difference between the *actual* payload position and the *target* trajectory, subject to the constraint that cable tension must remain positive ($T \> 0$).4  
4. **Actuation:** The computed thrust vector is converted to roll/pitch/yaw/throttle commands and sent to the Pixhawk via MAVLink.

## **5\. Phase 2: Full-Scale Heavy Lift (\>25kg Payload)**

Phase 2 scales the system to lift a human. This is not a linear upgrade; the physics of heavy-lift multicopters leads to exponential increases in power requirements and structural stresses.

### **5.1 The "Heavy Lifter" Specification**

To lift a human (80kg) \+ stretcher (10kg) \+ rigging (5kg) \+ margin (20%), the total payload capacity must be \~115kg. A swarm of 4 drones means each drone must carry \~30 kilograms of payload *plus* its own weight (\~20kg). This puts the Maximum Take-Off Mass (MTOM) of each drone in the 50-60kg class.

#### **5.1.1 Propulsion System**

We require industrial motors capable of 30-40kg thrust *per axis* (in an X4 quad) or 15-20kg per axis (in an X8 Coaxial).

* **Primary Candidate: T-Motor U15 II KV80.**  
  * **Thrust:** Max thrust \~36kg per motor at 100V. Ideally run at 50-60% throttle for efficiency.34  
  * **Configuration:** A Quadcopter using 4x U15 motors is risky (no redundancy). An **Octocopter (X8)** configuration using 8x **T-Motor U13 II** or **Hobbywing X9 Plus** is superior.  
* **Alternative: Hobbywing XRotor X9 Plus.**  
  * **Thrust:** Max 26.5kg per axis.36  
  * **Integration:** These come as integrated units (Motor \+ ESC \+ Prop), significantly simplifying the build and offering IPX7 waterproofing (critical for SAR).36 A Coaxial X8 setup with Hobbywing X9S would provide \~200kg total lift, safely carrying a 100kg payload with 50% redundancy.

#### **5.1.2 Airframe Structure**

Standard hobby frames will snap under these loads.

* **Frame:** **Foxtech Gaia 160MP** (modified) or a custom CNC-cut carbon fibre frame. The arms must be 40 mm+ in diameter carbon tubes.38  
* **Custom Fabrication:** European suppliers like **Refitech** or **CNCDrone** can cut custom centre plates from a 4-5mm quasi-isotropic carbon fibre plate to accommodate the spreader bar attachment point at the precise Centre of Gravity (CoG). .39

### **5.2 Power Systems: The Energy Density Dilemma**

Lifting 100kg requires immense power.

* **Voltage:** The system must operate at 12S (44.4V) or 14S (51.8V) to keep current within manageable limits.  
* **Battery Chemistry:**  
  * *Li-Ion (e.g., 18650/21700 packs):* High energy density (250Wh/kg), great for endurance. However, they exhibit voltage sag under high-current bursts (low C-rating). 41  
  * *LiPo (Lithium Polymer):* Lower density, but massive discharge capability (25C+).  
* **Recommendation:** For the *lift* phase, **LiPo** is safer. The swarm needs instant torque to stabilise a swinging patient. We recommend **Tattu Pro 12S 22000mAh** Smart Batteries. These are industry standards for agricultural drones and handle 25C continuous discharge 43

### **5.3 Phase 2 Bill of Materials (Per Drone \- Swarm of 4\)**

| Component | Specification | Unit Cost (€) | Qty | Total (€) | Source |
| :---- | :---- | :---- | :---- | :---- | :---- |
| **Propulsion System** | Hobbywing XRotor X9 Plus (Motor+ESC+Prop) | €350 | 8 (X8 Config) | €2,800 | 45 |
| **Frame** | Custom Heavy Lift (Gaia 160MP Class) | €2,200 | 1 | €2,200 | 38 |
| **Flight Controller** | Cube Orange+ (Triple Redundant) \+ Carrier | €450 | 1 | €450 | 20 |
| **Compute Module** | NVIDIA Jetson AGX Orin (Industrial Grade) | €1,500 | 1 | €1,500 | 11 |
| **Sensors** | 2x RealSense D435i \+ Ouster OS1 LiDAR | €3,500 | 1 | €3,500 | 21 |
| **Batteries** | Tattu Pro 12S 22000mAh Smart Battery | €750 | 4 | €3,000 | 46 |
| **Comms** | Herelink Blue (Long Range) | €1,800 | 1 | €1,800 | 47 |
| **Rigging** | Auto-release Hook \+ Dyneema Tether | €600 | 1 | €600 | 48 |
| **Unit Total** |  |  |  | **€15,850** |  |
| **Swarm Total** | **(4 Units \+ Spares)** |  |  | **\~€70,000** |  |

## **6\. Payload Interface and Patient Safety**

The interface between the swarm and the patient is critical. We cannot simply tie a rope to a person.

### **6.1 The Stretcher System**

* **NATO Litter:** The standard NATO litter (Stanag 2040\) is the baseline. It is rigid, widely available, and has standardised attachment points.49  
* **Spreader Bar:** A custom spreader bar is required to connect the four drone tethers to the four corners of the litter. This ensures the tethers remain clear of the patient's body and provides a stable lift geometry 51  
* **Patient Harness:** A rescue basket (e.g., Stokes litter) or a specialised evacuation harness (like the Petzl Thales) is secured to the litter to prevent the patient from falling out during turbulence.52

### **6.2 Tether Management**

* **Material:** Dyneema (UHMWPE) rope is essential due to its high strength-to-weight ratio and low stretch.  
* **Damping:** An elastic element (shock cord) must be integrated in series with the rigid tether to damp high-frequency vibrations from the drones, preventing them from being transmitted to the patient.  
* **Emergency Release:** Each drone must have a servo-actuated release hook. If a drone suffers a critical failure that makes it unflyable, it must detach immediately to prevent dragging the swarm down. The remaining three drones must then enter an emergency descent 54

## **7\. Regulatory Pathway: EASA SORA Compliance**

Operating a \>25kg swarm to carry a human is one of the most strictly regulated activities in European aviation. It falls under the **Specific Category** and borders on **Certified**.

### **7.1 Specific Operations Risk Assessment (SORA)**

Under EU Regulation 2019/947, the operator must submit a SORA.55

* **Ground Risk Class (GRC):** Even in a sparsely populated area, the risk is high due to the payload.  
* **Air Risk Class (ARC):** Likely ARC-b or ARC-c, depending on airspace.  
* **SAIL Level:** The combination will likely result in **SAIL V or VI** (Specific Assurance and Integrity Level). This is the highest level before "Certified."

### **7.2 Compliance Requirements for SAIL V/VI**

To achieve authorisation:

1. **Design Verification:** EASA may require a "Design Verification Report" (DVR) for the containment system. The drones need independent Flight Termination Systems (FTS) separate from the autopilot 57  
2. **Software Assurance:** The software (Rust control stack) must be developed to standards approaching DO-178C (software safety in airborne systems). This validates the choice of Rust for its memory safety guarantees.30  
3. **Operational Procedures:** A robust ConOps (Concept of Operations), including geofencing, emergency buffers, and remote pilot competency, is mandatory 55

## **8\. Financial Analysis and Timeline**

### **8.1 Total Project Cost Estimation**

* **Phase 1 Hardware:** €10,000  
* **Phase 2 Hardware:** €70,000  
* **Ground Station & Infrastructure:** €15,000 (Generators, rugged laptops, RTK base station).  
* **Development Labour (12 months):** €150,000 (2 FTE Engineers: 1 Robotics/Software, 1 Mechanical/Pilot).  
* **Regulatory/Certification Consultancy:** €25,000.  
* **Total Estimated Budget:** **\~€270,000**

### **8.2 Roadmap**

* **Month 1-3:** Procurement and assembly of Phase 1 X500 drones. Set up the ROS 2/Rust environment.  
* **Month 4-6:** Indoor testing of Phase 1 swarm with dummy payload (Vicon/OptiTrack environment). Tuning of admittance controllers.  
* **Months 7-9:** Outdoor testing of Phase 1\. Procurement of Phase 2 heavy-lift parts.Months **10-12:** Assembly of heavy lifters. Tether integration. Single-drone heavy lift tests.  
* **Month 13+:** Full swarm heavy lift tests (sandbags \-\> mannequin \-\> human). SORA submission.

## **9\. Conclusion**

The development of a drone swarm for human evacuation is technically feasible using current off-the-shelf components (Jetson Orin, T-Motor propulsion) and advanced distributed control algorithms (TU Delft). The shift to **Rust and ROS 2** provides the necessary software reliability for such a safety-critical application.

While **Phase 1** is a relatively low-cost entry point to validate the logic, **Phase 2** requires significant capital and regulatory rigour. The primary hurdle is not lifting the weight—industrial motors can easily do that—but proving to EASA that the distributed control system is robust enough to trust with a human life. By following the SORA methodology and utilising redundant X8 configurations, this project offers a revolutionary capability for SAR teams: a backpack-deployable aerial ambulance.

## **10\. Data Tables**

### **10.1 Comparison of Middleware for Swarm Control**

| Feature | ROS 2 (DDS) | ROS 2 (Zenoh) | Relevance to Project |
| :---- | :---- | :---- | :---- |
| **Discovery** | Multicast (Flooding) | Peer-to-Peer / Gossip | Zenoh reduces network saturation in WiFi swarms 27 |
| **Overhead** | High (RTPS packets) | Low (Wire-level efficient) | Zenoh allows more bandwidth for VIO/Depth data. |
| **WAN Support** | Complex (VPN required) | Native (Routing) | Zenoh allows the ground station to be miles away easily 28 |
| **Reliability** | Configurable (quality of service) | Configurable (quality of service) | Both support reliable streams; Zenoh is faster on setup. |

### **10.2 Battery Chemistry Trade-off for Phase 2**

| Metric | LiPo (Lithium Polymer) | Li-Ion (LiNiMnCo) | Verdict for SAR Swarm |
| :---- | :---- | :---- | :---- |
| **Energy Density** | 150-180 Wh/kg | 200-250 Wh/kg | Li-Ion is better for long flight times.41 |
| **Discharge Rate** | 25C \- 100C | 3C \- 10C | LiPo is required for heavy liftstabilisationn torque 58 |
| **Voltage Sag** | Low | High under load | LiPo provides consistent power during the lift. |
| **Safety** | Volatile if punctured | Safer (Metal cans) | LiPo requires an armoured casing (Tattu Pro series). |
| **Selection** | **Tattu Pro 12S** | **Not Recommended** | **Use LiPo for Lift Phase safety.** |

#### **Works cited**

1. TU Delft algorithm to enable drones to work together to transport heavy payloads, accessed on December 10, 2025, [https://www.therobotreport.com/tu-delft-algorithm-lets-drones-work-together-transport-heavy-payloads/](https://www.therobotreport.com/tu-delft-algorithm-lets-drones-work-together-transport-heavy-payloads/)  
2. .TU Delft \- an algorithm that lets drones lift heavy loads together and manipulate the load mid-air \- YouTube, accessed on December 10, 2025, [https://www.youtube.com/watch?v=tMvNCUuXai8](https://www.youtube.com/watch?v=tMvNCUuXai8)  
3. New algorithm lets autonomous drones work together to transport heavy, changing payloads \- TU Delft, accessed on December 10, 2025, [https://www.tudelft.nl/en/2025/me/news/new-algorithm-lets-autonomous-drones-work-together-to-transport-heavy-changing-payloads](https://www.tudelft.nl/en/2025/me/news/new-algorithm-lets-autonomous-drones-work-together-to-transport-heavy-changing-payloads)  
4. A Robust Neural Control Design for Multi-drone Slung Payload Manipulation with Control Contraction Metrics \- arXiv, accessed on December 10, 2025, [https://arxiv.org/pdf/2510.01489](https://arxiv.org/pdf/2510.01489)  
5. Control Framework for a UAV Slung-Payload Transportation System \- IEEE Xplore, accessed on December 10, 2025, [https://ieeexplore.ieee.org/iel7/7782633/9828546/10149091.pdf](https://ieeexplore.ieee.org/iel7/7782633/9828546/10149091.pdf)  
6. Swarm of Drones in a Simulation Environment—Efficiency and Adaptation \- MDPI, accessed on December 10, 2025, [https://www.mdpi.com/2076-3417/14/9/3703](https://www.mdpi.com/2076-3417/14/9/3703)  
7. X500 V2 Kits – Holybro Store, accessed on December 10, 2025, [https://holybro.com/products/x500-v2-kits](https://holybro.com/products/x500-v2-kits)  
8. Platform for Quadcopter Research (Holybro x500?) : r/diydrones \- Reddit, accessed on December 10, 2025, [https://www.reddit.com/r/diydrones/comments/1hny1iz/platform\_for\_quadcopter\_research\_holybro\_x500/](https://www.reddit.com/r/diydrones/comments/1hny1iz/platform_for_quadcopter_research_holybro_x500/)  
9. PX4 Development Kit \- X500 v2 – Holybro Store, accessed on December 10, 2025, [https://holybro.com/products/px4-development-kit-x500-v2](https://holybro.com/products/px4-development-kit-x500-v2)  
10. Jetson Orin Nano Super Developer Kit \- NVIDIA, accessed on December 10, 2025, [https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)  
11. Top 5 Carrier boards for Jetson \- ThinkRobotics.com, accessed on December 10, 2025, [https://thinkrobotics.com/blogs/learn/top-5-carrier-boards-for-jetson](https://thinkrobotics.com/blogs/learn/top-5-carrier-boards-for-jetson)  
12. NVIDIA Jetson Orin Nano Super Developer Kit \- Antratek Electronics, accessed on December 10, 2025, [https://www.antratek.com/nvidia-jetson-orin-nano-super-developer-kit](https://www.antratek.com/nvidia-jetson-orin-nano-super-developer-kit)  
13. Powering up Nvidia Jetson Orin Nano on Drone: r/diydrones \- Reddit, accessed on December 10, 2025, [https://www.reddit.com/r/diydrones/comments/1d1xojt/powering\_up\_nvidia\_jetson\_orin\_nano\_on\_drone/](https://www.reddit.com/r/diydrones/comments/1d1xojt/powering_up_nvidia_jetson_orin_nano_on_drone/)  
14. PDB & BEC – MATEKSYS, accessed on December 10, 2025, [https://www.mateksys.com/?page\_id=2845](https://www.mateksys.com/?page_id=2845)  
15. BEC12S-PRO, 9-55V to 5V/8V/12V-5A \- MATEKSYS, accessed on December 10, 2025, [https://www.mateksys.com/?portfolio=bec12s-pro](https://www.mateksys.com/?portfolio=bec12s-pro)  
16. Power Requirements Evaluation of Embedded Devices for Real-Time Video Line Detection, accessed on December 10, 2025, [https://www.mdpi.com/1996-1073/16/18/6677](https://www.mdpi.com/1996-1073/16/18/6677)  
17. Holybro Pixhawk 6C Arm® Cortex® H7 | MYBOTSHOP.DE, € 399,95, accessed on December 10, 2025, [https://www.mybotshop.de/Pixhawk-6C-Aluminum-Case](https://www.mybotshop.de/Pixhawk-6C-Aluminum-Case)  
18. FC Holybro Pixhawk 6C (plastic case) \&PM02-12S\&M10 GPS | on Hobbydrone.cz, accessed on December 10, 2025, [https://www.hobbydrone.cz/en/fc-holybro-pixhawk-6c-plastic-case-pm02-12s-m10-gps/](https://www.hobbydrone.cz/en/fc-holybro-pixhawk-6c-plastic-case-pm02-12s-m10-gps/)  
19. Hex/ProfiCNC-Here4 Multiband RTK GNSS \- Dronivo \- Your expert for d, 297,50 €, accessed on December 10, 2025, [https://www.dronivo.de/Hex/ProfiCNC-Here4-Multiband-RTK-GNSS\_1](https://www.dronivo.de/Hex/ProfiCNC-Here4-Multiband-RTK-GNSS_1)  
20. CubePilot HERE 4 Multiband RTK GNSS \- mybotshop, accessed on December 10, 2025, [https://www.mybotshop.de/CubePilot-HERE-4-Multiband-RTK-GNSS\_1](https://www.mybotshop.de/CubePilot-HERE-4-Multiband-RTK-GNSS_1)  
21. Intel® RealSense Depth Camera D435i | MYBOTSHOP.DE, € 415,95, accessed on December 10, 2025, [https://www.mybotshop.de/Intel-RealSense-Depth-Camera-D435i\_1](https://www.mybotshop.de/Intel-RealSense-Depth-Camera-D435i_1)  
22. Mounting assembly for Holybro X500 v2 by blackemblem \- Thingiverse, accessed on December 10, 2025, [https://www.thingiverse.com/thing:6137052](https://www.thingiverse.com/thing:6137052)  
23. Holybro X500 V2 ARF Kit | MYBOTSHOP.DE, € 379,95, accessed on December 10, 2025, [https://www.mybotshop.de/Holybro-X500-V2-ARF-Kit\_1](https://www.mybotshop.de/Holybro-X500-V2-ARF-Kit_1)  
24. HD system Remote Controller SIYI MK15 Handheld Ground Station Standard Combo, accessed on December 10, 2025, [https://www.hobbydrone.cz/en/hd-system-remote-controller-siyi-mk15-handheld-ground-station-standard-combo/](https://www.hobbydrone.cz/en/hd-system-remote-controller-siyi-mk15-handheld-ground-station-standard-combo/)  
25. ROS 2 — Dev documentation \- ArduPilot, accessed on December 10, 2025, [https://ardupilot.org/dev/docs/ros2.html](https://ardupilot.org/dev/docs/ros2.html)  
26. Software-in-the-Loop for Drone Simulation with Ardupilot | by Komsun T. \- Medium, accessed on December 10, 2025, [https://medium.com/@komxun/software-in-the-loop-simulation-with-ardupilot-43ecafc83571](https://medium.com/@komxun/software-in-the-loop-simulation-with-ardupilot-43ecafc83571)  
27. Optimising Data Distribution Service Discovery for Swarm Unmanned Aerial Vehicles Through Preloading and Network Awareness \- MDPI, accessed on December 10, 2025, [https://www.mdpi.com/2504-446X/9/8/564](https://www.mdpi.com/2504-446X/9/8/564)  
28. Actuate 2024 | Chris Lalancette | Zenoh and ROS 2: Not a Paradox \- YouTube, accessed on December 10, 2025, [https://www.youtube.com/watch?v=AS7l-iDQpdY](https://www.youtube.com/watch?v=AS7l-iDQpdY)  
29. Rust in Production: Building Drone Controllers with Fusion Engineering (Podcast Interview), accessed on December 10, 2025, [https://www.reddit.com/r/rust/comments/1e0w679/rust\_in\_production\_building\_drone\_controllers/](https://www.reddit.com/r/rust/comments/1e0w679/rust_in_production_building_drone_controllers/)  
30. Bringing Rust to Safety-Critical Systems in Space \- arXiv, accessed on December 10, 2025, [https://arxiv.org/html/2405.18135v1](https://arxiv.org/html/2405.18135v1)  
31. Awesome Robot Operating System 2 (ROS 2\) | Awesome ROS2 \- GitHub Pages, accessed on December 10, 2025, [https://fkromer.github.io/awesome-ros2/](https://fkromer.github.io/awesome-ros2/)  
32. MAVSDK (main / v3) | MAVSDK Guide, accessed on December 10, 2025, [https://mavsdk.mavlink.io/](https://mavsdk.mavlink.io/)  
33. GitHub \- mavlink/MAVSDK-Rust, accessed on December 10, 2025, [https://github.com/mavlink/MAVSDK-Rust](https://github.com/mavlink/MAVSDK-Rust)  
34. U15II KV80/KV100 Heavy Duty Motor for Industry \- UnmannedRC, accessed on December 10, 2025, [https://unmannedrc.com/products/u15ii-kv80-kv100-heavy-duty-motor-for-industry](https://unmannedrc.com/products/u15ii-kv80-kv100-heavy-duty-motor-for-industry)  
35. U15Ⅱ KV80 Heavy-lift UAV / Drone Motor \- 36kg Max. Thrust, accessed on December 10, 2025, [https://store.tmotor.com/product/u15-v2-motor-u-power-kv80.html](https://store.tmotor.com/product/u15-v2-motor-u-power-kv80.html)  
36. XRotor X9 Plus system \- HOBBYWING North America, accessed on December 10, 2025, [https://www.hobbywingdirect.com/products/xrotor-x9-plus](https://www.hobbywingdirect.com/products/xrotor-x9-plus).  
37. Hobbywing X9 Series Power System for Heavy Lift Drones \- FOXTECH Store, accessed on December 10, 2025, [https://store.foxtech.com/hobbywing-x9-series-power-system-for-heavy-lift-drones/](https://store.foxtech.com/hobbywing-x9-series-power-system-for-heavy-lift-drones/)  
38. GAIA 160MP-Heavy Lift Drone Frame \- Foxtech, accessed on December 10, 2025, [https://www.foxtechfpv.com/gaia-160-mp-heavy-lift-drone-frame.html](https://www.foxtechfpv.com/gaia-160-mp-heavy-lift-drone-frame.html)  
39. Carbon Drone Frame Manufacturer \- Refitech Composite Solutions, accessed on December 10, 2025, [https://www.refitech.eu/uav/](https://www.refitech.eu/uav/)  
40. CNC cutting service for frame in carbon fibre on EU: r/fpv \- Reddit, accessed on December 10, 2025, [https://www.reddit.com/r/fpv/comments/14fgcbm/cnc\_cutting\_service\_for\_frame\_in\_carbon\_fiber\_on/](https://www.reddit.com/r/fpv/comments/14fgcbm/cnc_cutting_service_for_frame_in_carbon_fiber_on/)  
41. Li-ion vs LiPo Batteries for Drones Which LLastLonger, accessed on December 10, 2025, [https://www.large-battery.com/blog/li-ion-vs-lipo-long-lasting-battery-drones/](https://www.large-battery.com/blog/li-ion-vs-lipo-long-lasting-battery-drones/)  
42. Li-Ion vs. LiPo Batteries for Long-Range Drone Flights: A 2025 Technical Guide, accessed on December 10, 2025, [https://www.xtbattery.com/news/li-ion-vs-lipo-batteries-for-long-range-drone-flights-a-2025-technical-guide/](https://www.xtbattery.com/news/li-ion-vs-lipo-batteries-for-long-range-drone-flights-a-2025-technical-guide/)  
43. Tattu Pro 22000mAh 44.4V 25C 12S1P Lipo Smart Battery Pack with AS150U-F Plug, accessed on December 10, 2025, [https://genstattu.com/tattu-pro-22000mah-44-4v-25c-12s-1p-lipo-smart-battery-pack-with-as150u-f-plug/](https://genstattu.com/tattu-pro-22000mah-44-4v-25c-12s-1p-lipo-smart-battery-pack-with-as150u-f-plug/)  
44. Tattu Pro 12S 22000mAh 45.6V 25C Lipo Smart Drone Battery \- Grepow, accessed on December 10, 2025, [https://www.grepow.com/uav-battery/tattu-pro-12s-22000mah-45-6v-25c-lipo-smart-drone-battery.html](https://www.grepow.com/uav-battery/tattu-pro-12s-22000mah-45-6v-25c-lipo-smart-drone-battery.html)  
45. Hobbywing \- X9 Plus Power System \- (1 Pair CW/CCW) \- Multirotor from 3DXR UK, accessed on December 10, 2025, [https://www.3dxr.co.uk/multirotor-c3/drone-arm-sets-c283/hobbywing-x9-plus-power-system-1-pair-cw-ccw-p4616](https://www.3dxr.co.uk/multirotor-c3/drone-arm-sets-c283/hobbywing-x9-plus-power-system-1-pair-cw-ccw-p4616)  
46. Tattu Plus 1.0 22000mAh 44.4V 25C 12S1P Lipo Battery Pack with AS150U plug, accessed on December 10, 2025, [https://gensace.de/products/tattu-plus-1-0-22000mah-44-4v-25c-12s1p-lipo-battery-pack-with-as150u-plug](https://gensace.de/products/tattu-plus-1-0-22000mah-44-4v-25c-12s1p-lipo-battery-pack-with-as150u-plug)  
47. CubePilot Herelink HD Video Transmission System (V1.1) \- mybotshop, accessed on December 10, 2025, [https://www.mybotshop.de/CubePilot-Herelink-HD-Video-Transmission-System-V11\_1](https://www.mybotshop.de/CubePilot-Herelink-HD-Video-Transmission-System-V11_1)  
48. Tether Attachment Kits (United States) | Guardian Fall Protection, accessed on December 10, 2025, [https://guardianfall.com/product/tether-attachment-kits/11401](https://guardianfall.com/product/tether-attachment-kits/11401)  
49. Rails with a surgical tool attachment turn a NATO litter into an operating table | TechLink, accessed on December 10, 2025, [https://techlinkcenter.org/technologies/rails-with-a-surgical-tool-attachment-turn-a-nato-litter-into-an-operating-table/74020857-3f7b-41b9-a802-8ee944be42ca](https://techlinkcenter.org/technologies/rails-with-a-surgical-tool-attachment-turn-a-nato-litter-into-an-operating-table/74020857-3f7b-41b9-a802-8ee944be42ca)  
50. NATO STANDARD AMedP-2.1 STRETCHERS, BEARING BRACKETS AND ATTACHMENT SUPPORTS, accessed on December 10, 2025, [https://www.coemed.org/files/stanags/03\_AMEDP/AMedP-2.1\_EDA\_V1\_E\_2040.pdf](https://www.coemed.org/files/stanags/03_AMEDP/AMedP-2.1_EDA_V1_E_2040.pdf)  
51. Spreader Bars \- Lifting Equipment Store USA, accessed on December 10, 2025, [https://liftingequipmentstore.us/collections/spreader-bars](https://liftingequipmentstore.us/collections/spreader-bars)  
52. Zenith Tethered Drone Use Cases, Applications, Key Features, accessed on December 10, 2025, [https://zenithaerotech.com/tethered-drone-use-cases-applications-key-features/](https://zenithaerotech.com/tethered-drone-use-cases-applications-key-features/)  
53. Petzl Thales Evacuation / Rescue Harness \- Climb On Equipment, accessed on December 10, 2025, [https://climbonequipment.com/en-us/products/petzl-thales-evacuation-rescue-harness](https://climbonequipment.com/en-us/products/petzl-thales-evacuation-rescue-harness)  
54. .This Drone Can Lift More Than You\! \- YouTube, accessed on December 10, 2025, [https://www.youtube.com/watch?v=m4VcbyXibt.A](https://www.youtube.com/watch?v=m4VcbyXibtA)  
55. Unlocking Efficiency: How SORA V2.5 Enhances EASA Drone Regulation for UAS Operators? \- UASolutions Sàrl, accessed on December 10, 2025, [https://www.uasolutions.ch/how-sora-v2-5-enhances-easa-drone-regulation/](https://www.uasolutions.ch/how-sora-v2-5-enhances-easa-drone-regulation/)  
56. Specific Operations Risk Assessment (SORA) \- EASA \- European Union, accessed on December 10, 2025, [https://www.easa.europa.eu/en/domains/drones-air-mobility/operating-drone/specific-category-civil-drones/specific-operations-risk-assessment-sora](https://www.easa.europa.eu/en/domains/drones-air-mobility/operating-drone/specific-category-civil-drones/specific-operations-risk-assessment-sora)  
57. No 002/2024 UAS Swarm Shows \- Transport Malta, accessed on December 10, 2025, [https://www.transport.gov.mt/Regulatory-Instrument-02-UAS-Swarm-Shows.pdf-f10120](https://www.transport.gov.mt/Regulatory-Instrument-02-UAS-Swarm-Shows.pdf-f10120)  
58. LiPo vs Lithium Ion Batteries for Unmanned & Robotics Applications, accessed on December 10, 2025, [https://www.unmannedsystemstechnology.com/feature/lipo-vs-lithium-ion-batteries-for-unmanned-robotics-applications/](https://www.unmannedsystemstechnology.com/feature/lipo-vs-lithium-ion-batteries-for-unmanned-robotics-applications/)
