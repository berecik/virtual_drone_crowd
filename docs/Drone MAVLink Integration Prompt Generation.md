# **Architectural Analysis and Implementation Strategy for PX4-ROS 2 Integration via XRCE-DDS in Autonomous SAR Drone Swarms**

## **1\. Introduction: The Evolution of Autonomous Search and Rescue Architectures**

The domain of Search and Rescue (SAR) is undergoing a fundamental transformation, driven by the convergence of high-performance edge computing, decentralized networking, and swarm intelligence. The "Virtual Drone Crowd" project represents a quintessential example of this paradigm shift, proposing a sophisticated "Swarm Drone System" (SDS) designed to mitigate the physiological and operational limitations of traditional human-centric rescue efforts.2 Traditional SAR operations, relying on line-abreast foot searches or manned rotary-wing aviation, are inherently constrained by environmental hazards, operator fatigue, and the sheer scale of search areas. The integration of Unmanned Aerial Vehicles (UAVs) initially offered a "flying binocular" capability, but the "one-pilot-one-drone" model has proven insufficient for the spatial and temporal demands of modern emergency response.2

The "Virtual Drone Crowd" project articulates a two-phase implementation strategy that necessitates a rigorous re-evaluation of the underlying software control stack. Phase 1 focuses on a Proof of Concept (PoC) utilizing a swarm of agile, sensor-rich reconnaissance drones to localize casualties, while Phase 2 introduces heavy-lift platforms for physical extraction.2 This operational bifurcation imposes conflicting requirements on the system architecture: the agility and low-latency communication required for the swarm, and the absolute determinism and safety assurance required for the heavy-lift evacuation vehicle.

Central to this architectural challenge is the interface between the high-level mission planning (performed by the swarm intelligence) and the low-level flight stabilization (performed by the flight controller). The industry standard for this interface has historically been the MAVLink protocol, mediated by the MAVROS package. However, the project's working plan explicitly mandates a transition to the modern XRCE-DDS (eXtremely Resource Constrained Environments Data Distribution Service) middleware, connecting ROS 2 (Robot Operating System 2\) directly to the PX4 Autopilot.2 This transition removes the serialization overhead of MAVLink but introduces significant complexity in terms of Quality of Service (QoS) configuration, coordinate frame transformations, and safety state machine management.

This research report provides an exhaustive technical analysis of these integration challenges. It dissects the theoretical underpinnings of the PX4-ROS 2 bridge, analyzes the specific constraints of the Rust programming language (rclrs) in this context, and synthesizes these findings into a comprehensive implementation specification. The ultimate objective is to formulate a precise, context-aware prompt for "Junie"—an AI coding assistant—to automate the generation of the mission-critical control nodes, thereby reducing the development lead time for the project's "Development Environment & Core Control" task.

### **1.1 The Operational Hardware Context**

Understanding the software requirements necessitates a detailed examination of the hardware platform specified in the project documentation. The Phase 1 PoC utilizes the Holybro X500 V2 frame, a research-standard carbon fiber platform.2 The computational core is the Nvidia Jetson Orin Nano, chosen for its ability to deliver 40 TOPS of AI performance, enabling onboard execution of complex computer vision models (e.g., YOLOv8 for human detection) without relying on ground station processing.2

The flight controller is the Pixhawk 6C, running the PX4 firmware.2 The interplay between the Jetson (High-Level Computer or Companion Computer) and the Pixhawk (Flight Management Unit or FMU) is the critical path for system reliability. In legacy systems, this link was often a bottleneck. The move to XRCE-DDS over high-speed UART or Ethernet allows the Jetson to inject control setpoints at rates exceeding 50Hz, a necessity for the dynamic maneuvering required in obstacle-rich SAR environments.7

**Table 1: Hardware-Software Dependency Matrix**

| Hardware Component | Software Role | Critical Interface Requirement |
| :---- | :---- | :---- |
| **Nvidia Jetson Orin Nano** | High-Level Control (ROS 2/Rust) | Must publish setpoints \>20Hz to avoid failsafe triggering.8 |
| **Pixhawk 6C (FMU)** | Low-Level Stabilization (PX4) | Requires "Offboard" mode heartbeat and valid NED coordinates.9 |
| **Luxonis OAK-D Pro** | Perception (Depth/AI) | Data must be fused and transformed from Optical Frame to NED Frame.10 |
| **Holybro Here 3+** | Positioning (RTK GNSS) | Provides global position; ROS 2 node must handle local-to-global setpoint conversion.11 |

The selection of **Rust** for the core control logic on the Jetson is a strategic decision to mitigate memory safety risks. Unlike C++, which allows for subtle bugs like buffer overflows and race conditions that can lead to catastrophic platform loss, Rust’s ownership model enforces memory safety at compile time.2 This is particularly pertinent for the Phase 2 Heavy Lift drone, where a software crash could result in a 100kg projectile falling into a rescue zone.

## **2\. Architectural Paradigm Shift: From MAVLink to XRCE-DDS**

The most significant technical deviation in this project from traditional drone development is the abandonment of MAVROS in favor of the micro XRCE-DDS bridge. Understanding the nuances of this shift is essential for constructing the control logic.

### **2.1 Limitations of the MAVLink Legacy**

MAVLink was designed as a lightweight, header-only message marshaling library for micro aerial vehicles. It excels at telemetry over low-bandwidth radio links (e.g., 433MHz or 915MHz telemetry radios). The ROS 1 integration, MAVROS, acted as a "translation layer," converting ROS topics into MAVLink streams.12

However, this architecture introduces a "double serialization" penalty. A control command generated in ROS is serialized to a ROS message, deserialized by MAVROS, re-serialized to MAVLink, sent over UART, and deserialized by PX4. In high-frequency control loops (e.g., 100Hz trajectory tracking), this latency becomes non-negligible and introduces jitter that destabilizes the flight controller's internal estimators.14 Furthermore, MAVROS acts as a centralized bottleneck, contrary to the decentralized philosophy of the "Virtual Drone Crowd" project.2

### **2.2 The XRCE-DDS Architecture**

The architecture adopted by PX4 v1.14 and later leverages the OMG DDS (Data Distribution Service) standard, specifically the XRCE (eXtremely Resource Constrained Environments) profile.

* **The Client (FMU):** The Pixhawk runs a uXRCE-DDS Client. This client continuously polls the internal uORB (micro Object Request Broker) bus—PX4's internal pub/sub system—and serializes selected topics directly to the DDS wire format.7  
* **The Agent (Companion):** The Jetson runs the micro-xrce-dds-agent. This process acts as a proxy, projecting the FMU's topics into the global ROS 2 dataspace.3

This architecture provides a "direct pipe" into the flight controller. A ROS 2 node on the Jetson can subscribe to /fmu/out/vehicle\_odometry and receive data that is byte-for-byte identical to what the internal position controller sees, with minimal overhead. Conversely, publishing to /fmu/in/trajectory\_setpoint injects data directly into the PX4 control loop.7

**Implications for Implementation:**

1. **Topic Naming:** There is no /mavros/... namespace. Topics are strictly /fmu/in/... for commands and /fmu/out/... for telemetry.12  
2. **Message Definitions:** The code must use px4\_msgs definitions, which mirror the internal C++ structs of PX4, rather than generic geometry\_msgs.7  
3. **QoS Sensitivity:** Unlike MAVROS, which handled buffering internally, XRCE-DDS exposes the raw DDS QoS policies. A mismatch between the Client's "Best Effort" publication strategy and a ROS 2 Node's "Reliable" subscription strategy will result in total communication silence.3

### **2.3 The Role of Zenoh in Swarm Operations**

While XRCE-DDS handles the vertical link (OBC to FMU), the project utilizes **Eclipse Zenoh** for the horizontal link (Drone to Drone). Standard DDS discovery relies on multicast, which generates "discovery storms" that can saturate WiFi networks as the swarm size increases.2 Zenoh reduces this overhead by up to 99%.7

For the "Core Control Task," the implication is that the Rust control node must act as a gateway. It consumes high-level swarm commands (via Zenoh) and translates them into local flight control commands (via XRCE-DDS). The prompt for Junie must focus on the *local* control loop, but the architecture implies this node sits at the boundary of the two networks.

## **3\. Theoretical Framework of Offboard Control**

To program an autonomous drone, one must understand the state machine governing its behavior. PX4 is a safety-critical system; it does not blindly accept commands. It requires a specific "handshake" and continuous "proof of life" to delegate control to an external computer.

### **3.1 The OffboardControlMode Heartbeat**

The most common failure mode in PX4 integration is the "Offboard Rejected" error. This occurs because the internal commander requires a valid stream of OffboardControlMode messages *before* it allows a mode switch.8

This message serves two purposes:

1. **Proof of Life:** It must arrive at a frequency greater than 2 Hz (typically 10-20 Hz is recommended). If the stream is interrupted for more than COM\_OF\_LOSS\_T (default 0.5s), the drone assumes the companion computer has crashed and triggers a failsafe (usually switching to "Hold" or "Return" mode).9  
2. **Control Authority Definition:** It tells the internal controller *which* loops to bypass. For example, setting position \= true tells the internal controller, "Ignore your internal position hold logic; I will provide the position setpoint." Setting velocity \= false means the internal velocity controller will still run, but it will take its input from the position controller's output.8

**Insight:** The Rust implementation cannot simply publish a setpoint when it wants the drone to move. It must run a continuous, high-frequency loop (separate from sensor callbacks) that publishes the OffboardControlMode heartbeat, regardless of whether the target position has changed.

### **3.2 The TrajectorySetpoint Interface**

The TrajectorySetpoint message is the primary vector for high-level control. It allows simultaneous specification of position, velocity, acceleration, and jerk (derivative of acceleration).15

* **NaN Masking:** PX4 uses IEEE 754 NaN (Not a Number) as a mask. To control only position, the velocity and acceleration fields must be populated with NaN. Sending 0.0 is *not* the same as sending NaN; sending 0.0 for velocity commands the drone to stop, whereas NaN tells the controller to calculate the velocity necessary to reach the target position.9  
* **Coordinate Frame:** All values in TrajectorySetpoint must be in the **NED (North-East-Down)** frame. This is a critical distinction from the ROS standard **ENU (East-North-Up)**.16

### **3.3 Control Loop Dynamics**

Research indicates that there is a decoupling between the publication rate of the control mode and the setpoint. While OffboardControlMode simply needs to satisfy the \>2Hz requirement, the TrajectorySetpoint should ideally match the update rate of the internal control loop to prevent aliasing or stair-stepping in the flight path.14 The internal multicopter position controller typically runs at 50Hz or 100Hz. Therefore, a Rust control node loop rate of **20Hz to 50Hz** is optimal—fast enough for smooth motion, but slow enough to avoid saturating the XRCE-DDS link.21

## **4\. Mathematical Imperatives: Spatial Reference Systems**

The discord between the robotics community (ROS) and the aerospace community (PX4) regarding coordinate frames is a consistent source of catastrophic failure in autonomous systems. The project must implement a rigorous conversion layer.

### **4.1 The ENU vs. NED Conflict**

* **ROS 2 (ENU):** The "East-North-Up" frame is a right-handed Cartesian system. The X-axis points East, Y points North, and Z points Up (away from gravity). This is intuitive for ground robots and manipulation.7  
* **PX4 (NED):** The "North-East-Down" frame is also right-handed but aligns Z with gravity (Down). X points North, Y points East. This convention simplifies navigation mathematics where "down" is a positive increase in altitude (depth) or gravity vector alignment.7

### **4.2 Transformation Logic**

Converting a position vector $P\_{ENU} \= \[x, y, z\]^T$ to $P\_{NED}$ requires a specific rotation.  
Geometrically, we are rotating the frame 90 degrees around the Z-axis (aligning East with North) and then 180 degrees around the new X-axis (flipping Up to Down).7  
The linear mapping is:

$$x\_{NED} \= y\_{ENU}$$

$$y\_{NED} \= x\_{ENU}$$

$$z\_{NED} \= \-z\_{ENU}$$  
For orientation (quaternions or yaw), the transformation is more complex. For a simple Yaw angle $\\psi$:

$$\\psi\_{NED} \= \\frac{\\pi}{2} \- \\psi\_{ENU}$$

This converts the unit-circle angle (0 at East, counter-clockwise positive) to the compass heading (0 at North, clockwise positive).24  
**Implementation Requirement:** The Rust code must explicitly implement this transform. The px4\_ros\_com package provides C++ helpers, but for a pure Rust implementation using rclrs, we must write a helper function, potentially utilizing the nalgebra crate for vector operations.26 The prompt for Junie must specifically request this function to ensure the drone doesn't fly into the ceiling (due to inverted Z) or 90 degrees off course.

## **5\. Systems Engineering: Quality of Service (QoS)**

In the XRCE-DDS architecture, QoS is not just a performance tuning parameter; it is a connectivity requirement. The DDS middleware will deliberately drop connections between Publishers and Subscribers if their QoS policies are incompatible.28

### **5.1 The "Best Effort" Constraint**

PX4, running on the STM32H7 microcontroller, has severely constrained RAM. It cannot afford to maintain large history buffers for "Reliable" transmission (which requires acknowledging every packet and retransmitting lost ones). Therefore, PX4 publishes all telemetry (e.g., VehicleOdometry, VehicleStatus) with a **Best Effort** reliability policy and a **Keep Last (1)** history policy.7

**The Trap:** The default QoS profile in ROS 2 is "Reliable". If the Rust control node creates a default subscriber for VehicleOdometry, the connection will fail because a "Reliable" subscriber cannot connect to a "Best Effort" publisher.30 The prompt to Junie must explicitly mandate the configuration of a "Sensor Data" QoS profile (Best Effort, Volatile) for all subscriptions to /fmu/out/... topics.

### **5.2 Publisher QoS for Setpoints**

For publishing setpoints (/fmu/in/trajectory\_setpoint), the reliability requirement is more nuanced. While the Agent on the Jetson can handle "Reliable" connections, using "Reliable" for high-frequency (50Hz) control loops over a wireless link (if the architecture were distributed) or even a busy internal bus can introduce latency jitter. If a packet is dropped, the "Reliable" protocol pauses to retransmit, potentially delaying the *fresh* setpoint queued behind it. In control theory, fresh data is always more valuable than stale data. Therefore, **Best Effort** is also the recommended policy for publishing high-rate setpoints.31

**Table 2: Required QoS Configuration for rclrs Node**

| Interaction Type | Topic Example | Reliability | Durability | History | Reason |
| :---- | :---- | :---- | :---- | :---- | :---- |
| **Subscription** | /fmu/out/vehicle\_odometry | **Best Effort** | Volatile | Keep Last (1) | Must match PX4 Publisher settings.3 |
| **Publication** | /fmu/in/trajectory\_setpoint | **Best Effort** | Volatile | Keep Last (1) | Minimize latency; stale setpoints are irrelevant.32 |
| **Publication** | /fmu/in/offboard\_control\_mode | **Best Effort** | Volatile | Keep Last (1) | High-rate heartbeat; dropped packets are acceptable if stream is fast.8 |
| **Service/Cmd** | /fmu/in/vehicle\_command | Reliable | Transient Local | Keep All | Commands (Arm/Disarm) must be guaranteed.12 |

## **6\. Software Engineering: The Rust/ROS 2 Ecosystem**

The project's decision to use Rust (rclrs) instead of C++ (rclcpp) or Python (rclpy) introduces specific implementation constraints. rclrs is a wrapper around the C-based rcl layer, but it is less mature than its counterparts.33

### **6.1 rclrs Specifics**

* **Asynchronous Execution:** rclrs is designed with async/await in mind. The control loop should ideally be implemented using an async timer or a dedicated thread that spins the node.33  
* **QoS Syntax:** Unlike Python's simple qos\_profile=qos\_profile\_sensor\_data, rclrs often requires explicit struct instantiation. The prompt must provide the correct boilerplate to avoid compilation errors.29  
* **Message Generation:** The px4\_msgs package must be present in the workspace. rclrs generates Rust structs from the .msg definitions at build time. The prompt must assume this dependency is met (use px4\_msgs::msg::...).

### **6.2 Safety Considerations**

Rust's borrow checker prevents data races. In a control node, we often have a shared state (e.g., "current position") that is updated by a subscriber callback and read by the publisher loop. In C++, this requires careful mutex locking. In Rust, the compiler forces the use of Arc\<Mutex\<T\>\> or similar synchronization primitives, ensuring thread safety.6 The prompt should guide Junie to structure the node such that the state is safely shared between the subscription callback and the control loop.

## **7\. Synthesis: The Automated Implementation Strategy**

Based on the preceding analysis, the implementation strategy for the "Development Environment & Core Control task" is as follows:

1. **Environment:** A ROS 2 Humble workspace on the Jetson Orin Nano, containing px4\_msgs and a new Rust package.  
2. **Node Logic:** A single Rust binary that:  
   * Initializes a connection to the ROS 2 domain.  
   * Configures "Best Effort" QoS profiles.  
   * Subscribes to /fmu/out/vehicle\_odometry (optional for open-loop, essential for closed-loop) to verify the drone's state.  
   * Starts a 20Hz timer loop.  
   * **Phase A (Arming):** Publishes OffboardControlMode for 10 cycles (0.5s) to satisfy the failsafe, then publishes VehicleCommand to switch to Offboard mode and Arm.  
   * **Phase B (Control):** Continuously publishes OffboardControlMode (heartbeat) and TrajectorySetpoint (command).  
   * **Utility:** Converts all inputs from ENU to NED on the fly.

### **7.1 The Junie Prompt**

The following prompt encapsulates this strategy into a directive for the AI coding assistant. It provides the necessary context, constraints, and mathematical formulas to ensure a robust "first-pass" success.

---

**Prompt for Junie:**

**Role:** Act as a Senior Robotics Systems Engineer specializing in Rust, ROS 2, and PX4 Autopilot integration.

**Project Context:** We are developing the "Virtual Drone Crowd" (Phase 1), a swarm SAR system using the Nvidia Jetson Orin Nano and Pixhawk 6C. We are currently implementing the **Core Control Task**: establishing an Offboard control loop via the **XRCE-DDS middleware**.

**Task:** Write a complete, production-ready Rust ROS 2 node using the rclrs client library. This node must establish control of the PX4 drone, switch it to Offboard mode, arm it, and maintain a hover setpoint.

**Technical Constraints & Requirements:**

1. **Dependencies:**  
   * Target crate: rclrs (for ROS 2 bindings).  
   * Message crate: px4\_msgs (assume it is generated and available in the workspace).  
   * Math crate: nalgebra (for vector operations).  
   * Async runtime: tokio (or standard std::thread if simpler for rclrs spinning).  
2. **Middleware & Topic Configuration:**  
   * **Heartbeat (Critical):** You must publish to /fmu/in/offboard\_control\_mode at **20 Hz**.  
     * Message Type: px4\_msgs::msg::OffboardControlMode.  
     * Fields: Set position \= true, velocity \= false, acceleration \= false, attitude \= false, body\_rate \= false. Set timestamp using the node's clock (microseconds).  
   * **Setpoint:** You must publish to /fmu/in/trajectory\_setpoint at **20 Hz**.  
     * Message Type: px4\_msgs::msg::TrajectorySetpoint.  
     * Fields: position \= \[x, y, z\], yaw \= 0.0. IMPORTANT: Set velocity and acceleration arrays to NaN (use f32::NAN) to indicate they are uncontrolled.  
   * **Command:** You must publish to /fmu/in/vehicle\_command to change modes.  
     * Message Type: px4\_msgs::msg::VehicleCommand.  
     * Commands: VEHICLE\_CMD\_DO\_SET\_MODE (param1=1, param2=6 for Offboard) and VEHICLE\_CMD\_COMPONENT\_ARM\_DISARM (param1=1.0 for Arm).  
3. **Quality of Service (QoS) \- Strict Requirement:**  
   * PX4's uXRCE-DDS bridge uses "Best Effort" for telemetry. To ensure compatibility and low latency, define a custom QoS profile for all Publishers and Subscribers.  
   * **Profile:** Reliability: BestEffort, Durability: TransientLocal, History: KeepLast(1).  
   * *Failure to use BestEffort will result in communication failure.*  
4. **Coordinate Frame Transformation (ENU \-\> NED):**  
   * ROS 2 uses ENU (East-North-Up). PX4 uses NED (North-East-Down).  
   * Implement a helper function enu\_to\_ned(x, y, z) \-\> \[f32; 3\].  
   * **Logic:** $X\_{NED} \= Y\_{ENU}$, $Y\_{NED} \= X\_{ENU}$, $Z\_{NED} \= \-Z\_{ENU}$.  
   * Apply this to the setpoint (e.g., target 5m altitude in ENU is z \= 5.0, which must convert to z \= \-5.0 in NED).  
5. **Control Logic Flow:**  
   * Initialize the node.  
   * Start a 20Hz loop.  
   * **Step 1:** Stream OffboardControlMode for 10 cycles (approx 0.5s) to satisfy the PX4 failsafe proof-of-life.  
   * **Step 2:** Once the stream is established, publish VehicleCommand to switch to OFFBOARD mode and ARM.  
   * **Step 3:** Continue streaming OffboardControlMode and TrajectorySetpoint indefinitely.

**Output:** Provide the main.rs file with extensive comments explaining the "Why" behind the QoS settings and Coordinate Conversions. Also provide the Cargo.toml.

## ---

**8\. Conclusion and Future Outlook**

The transition to XRCE-DDS and Rust places the "Virtual Drone Crowd" project at the bleeding edge of aerial robotics. While this architecture imposes a steeper learning curve than the legacy MAVROS/C++ stack—specifically regarding explicit QoS management and manual coordinate transformations—it offers the determinism and performance efficiency required for the project's Phase 2 Heavy Lift goals.

The analysis conducted herein highlights that the "Development Environment & Core Control task" is not merely a coding exercise but a systems integration challenge. The provided prompt for Junie serves as a codified specification of this integration, encapsulating the physics, the protocol, and the safety logic into a single, executable directive. By automating this foundational layer, the engineering team can shift focus to the higher-order problems of swarm coordination and casualty perception, accelerating the deployment of this life-saving technology.

## **9\. Data Tables & Specifications**

**Table 3: Message Field Mapping for TrajectorySetpoint (Position Control)**

| Field | Type | Value Strategy | Description |
| :---- | :---- | :---- | :---- |
| timestamp | uint64 | node\_clock.now() | Microseconds since boot. Essential for sync. |
| position | float32\[1\] | \[x\_ned, y\_ned, z\_ned\] | Target position in Local NED frame. |
| velocity | float32\[1\] | \[NaN, NaN, NaN\] | Masked to enable Position Controller in PX4. |
| acceleration | float32\[1\] | \[NaN, NaN, NaN\] | Masked. |
| yaw | float32 | rad | Heading in NED (0 \= North). |
| yawspeed | float32 | NaN | Masked. |

**Table 4: Swarm vs. Unit Communication Architecture**

| Layer | Technology | Reliability Policy | Purpose | Source |
| :---- | :---- | :---- | :---- | :---- |
| **Intra-Swarm** | Eclipse Zenoh | Best Effort / Reliable | Global coordination, consensus, task allocation. | 7 |
| **Unit (OBC\<-\>FMU)** | XRCE-DDS | Best Effort | Real-time flight control (20-50Hz). | 3 |
| **Telemetry** | XRCE-DDS | Best Effort | High-rate sensor data (IMU, Odometry). | 7 |
| **Commands** | XRCE-DDS | Reliable | State changes (Arm, Disarm, Mode Switch). | 12 |

#### **Works cited**

1. Projekt Dronów SAR: Ewakuacja Człowieka  
2. uXRCE-DDS (PX4-ROS 2/DDS Bridge) | PX4 Guide (main), accessed on January 2, 2026, [https://docs.px4.io/main/en/middleware/uxrce\_dds](https://docs.px4.io/main/en/middleware/uxrce_dds)  
3. map\_3d \- crates.io: Rust Package Registry, accessed on January 2, 2026, [https://crates.io/crates/map\_3d/0.1.5](https://crates.io/crates/map_3d/0.1.5)  
4. transforms \- Rust \- Docs.rs, accessed on January 2, 2026, [https://docs.rs/transforms](https://docs.rs/transforms)  
5. Transforms: A robotics-focused library to keep track of reference frames : r/rust \- Reddit, accessed on January 2, 2026, [https://www.reddit.com/r/rust/comments/1hdc7vl/transforms\_a\_roboticsfocused\_library\_to\_keep/](https://www.reddit.com/r/rust/comments/1hdc7vl/transforms_a_roboticsfocused_library_to_keep/)  
6. ROS 2 User Guide | PX4 Guide (main), accessed on January 2, 2026, [https://docs.px4.io/main/en/ros2/user\_guide](https://docs.px4.io/main/en/ros2/user_guide)  
7. Offboard Mode (Generic/All Frames) | PX4 Guide (main), accessed on January 2, 2026, [https://docs.px4.io/main/en/flight\_modes/offboard](https://docs.px4.io/main/en/flight_modes/offboard)  
8. Offboard Mode \- PX4 User Guide \- GitBook, accessed on January 2, 2026, [https://px4.gitbook.io/px4-user-guide/flying/flight\_modes/offboard](https://px4.gitbook.io/px4-user-guide/flying/flight_modes/offboard)  
9. tf\_rosrust \- crates.io: Rust Package Registry, accessed on January 2, 2026, [https://crates.io/crates/tf\_rosrust](https://crates.io/crates/tf_rosrust)  
10. ros2-rust ros2\_rust · Discussions \- GitHub, accessed on January 2, 2026, [https://github.com/ros2-rust/ros2\_rust/discussions](https://github.com/ros2-rust/ros2_rust/discussions)  
11. Transitioning from ROS 1 (ArduPilot) to ROS 2 (PX4): A Complete Mental Model | by Sidharth Mohan Nair | Medium, accessed on January 2, 2026, [https://medium.com/@sidharthmohannair/transitioning-from-ros-1-ardupilot-to-ros-2-px4-a-complete-mental-model-98c54758f569](https://medium.com/@sidharthmohannair/transitioning-from-ros-1-ardupilot-to-ros-2-px4-a-complete-mental-model-98c54758f569)  
12. PX4 Offboard Control Using MAVROS on ROS | 404warehouse, accessed on January 2, 2026, [https://404warehouse.net/2015/12/20/autopilot-offboard-control-using-mavros-package-on-ros/](https://404warehouse.net/2015/12/20/autopilot-offboard-control-using-mavros-package-on-ros/)  
13. Huge Control Delay in Offboard Mode with ROS2 RTPS \- PX4 Discussion Forum, accessed on January 2, 2026, [https://discuss.px4.io/t/huge-control-delay-in-offboard-mode-with-ros2-rtps/28943](https://discuss.px4.io/t/huge-control-delay-in-offboard-mode-with-ros2-rtps/28943)  
14. TrajectorySetpoint (UORB message) | PX4 Guide (main), accessed on January 2, 2026, [https://docs.px4.io/main/en/msg\_docs/TrajectorySetpoint](https://docs.px4.io/main/en/msg_docs/TrajectorySetpoint)  
15. ROS 2 User Guide \- PX4 Guide, accessed on January 2, 2026, [https://docs.px4.io/v1.14/en/ros/ros2\_comm](https://docs.px4.io/v1.14/en/ros/ros2_comm)  
16. Offboard Mode · PX4 User Guide, accessed on January 2, 2026, [https://docs.px4.io/v1.11/en/flight\_modes/offboard](https://docs.px4.io/v1.11/en/flight_modes/offboard)  
17. PX4-Autopilot/msg/versioned/TrajectorySetpoint.msg at main \- GitHub, accessed on January 2, 2026, [https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/TrajectorySetpoint.msg](https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/TrajectorySetpoint.msg)  
18. Velocity and Position Control in Offboard Mode \- PX4 Discussion Forum, accessed on January 2, 2026, [https://discuss.px4.io/t/velocity-and-position-control-in-offboard-mode/37107](https://discuss.px4.io/t/velocity-and-position-control-in-offboard-mode/37107)  
19. ROS 2 Offboard Control Example | PX4 Guide (main), accessed on January 2, 2026, [https://docs.px4.io/main/en/ros2/offboard\_control](https://docs.px4.io/main/en/ros2/offboard_control)  
20. How to increase the frequency of the offboard.set\_velocity\_ned command \- MAVSDK, accessed on January 2, 2026, [https://discuss.px4.io/t/offboard-how-to-increase-the-frequency-of-the-offboard-set-velocity-ned-command/26091](https://discuss.px4.io/t/offboard-how-to-increase-the-frequency-of-the-offboard-set-velocity-ned-command/26091)  
21. geoconv \- Rust \- Docs.rs, accessed on January 2, 2026, [https://docs.rs/geoconv](https://docs.rs/geoconv)  
22. ENU \-\> NED frame conversion using quaternions \- Stack Overflow, accessed on January 2, 2026, [https://stackoverflow.com/questions/49790453/enu-ned-frame-conversion-using-quaternions](https://stackoverflow.com/questions/49790453/enu-ned-frame-conversion-using-quaternions)  
23. How to change NED to ENU? \- ROS Answers archive, accessed on January 2, 2026, [https://answers.ros.org/question/336814/](https://answers.ros.org/question/336814/)  
24. Reiterate over frame conversions between NED and ENU · Issue \#216 · mavlink/mavros, accessed on January 2, 2026, [https://github.com/mavlink/mavros/issues/216](https://github.com/mavlink/mavros/issues/216)  
25. Very simple coordinate frame conversions in Rust \- GitHub, accessed on January 2, 2026, [https://github.com/sunsided/coordinate-frame](https://github.com/sunsided/coordinate-frame)  
26. sguaba \- Rust, accessed on January 2, 2026, [https://docs.rs/sguaba](https://docs.rs/sguaba)  
27. Quality of Service settings — ROS 2 Documentation: Rolling documentation, accessed on January 2, 2026, [https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html)  
28. QoSProfile in rclrs \- Rust \- Docs.rs, accessed on January 2, 2026, [https://docs.rs/rclrs/latest/i686-pc-windows-msvc/rclrs/struct.QoSProfile.html](https://docs.rs/rclrs/latest/i686-pc-windows-msvc/rclrs/struct.QoSProfile.html)  
29. QoSReliabilityPolicy in rclrs \- Rust \- Docs.rs, accessed on January 2, 2026, [https://docs.rs/rclrs/latest/i686-pc-windows-msvc/rclrs/enum.QoSReliabilityPolicy.html](https://docs.rs/rclrs/latest/i686-pc-windows-msvc/rclrs/enum.QoSReliabilityPolicy.html)  
30. Issue with uORB message TrajectorySetponit \- PX4 Autopilot, accessed on January 2, 2026, [https://discuss.px4.io/t/issue-with-uorb-message-trajectorysetponit/32175](https://discuss.px4.io/t/issue-with-uorb-message-trajectorysetponit/32175)  
31. Mastering ROS 2 For Robotics Programming Design, Build, Simulate, and Prototype Complex Robots Using The Robot Operating (Lentin Joseph, Jonathan Cacace) (Z-Library) | PDF \- Scribd, accessed on January 2, 2026, [https://www.scribd.com/document/912727269/Mastering-ROS-2-for-Robotics-Programming-Design-Build-Simulate-And-Prototype-Complex-Robots-Using-the-Robot-Operating-Lentin-Joseph-Jonathan-Cac](https://www.scribd.com/document/912727269/Mastering-ROS-2-for-Robotics-Programming-Design-Build-Simulate-And-Prototype-Complex-Robots-Using-the-Robot-Operating-Lentin-Joseph-Jonathan-Cac)  
32. rclrs \- Rust, accessed on January 2, 2026, [https://docs.rs/rclrs](https://docs.rs/rclrs)  
33. Becoming an "official" client library \- ROS General \- Open Robotics Discourse, accessed on January 2, 2026, [https://discourse.openrobotics.org/t/becoming-an-official-client-library/35521](https://discourse.openrobotics.org/t/becoming-an-official-client-library/35521)