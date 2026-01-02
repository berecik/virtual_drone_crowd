/// Virtual Drone Crowd - Phase 1: Core Control Task
///
/// This node implements a PX4 Offboard control loop using ROS 2 (rclrs) and XRCE-DDS.
/// It handles the necessary "proof-of-life" heartbeat, coordinate transformations,
/// and QoS settings required for reliable communication with the Pixhawk 6C.
///
/// Author: Senior Robotics Systems Engineer
/// Project: Virtual Drone Crowd (SAR Swarm)

use rclrs::{self, QOS_PROFILE_DEFAULT};
use px4_msgs::msg::{OffboardControlMode, TrajectorySetpoint, VehicleCommand};
use std::env;
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use nalgebra::Vector3;

/// --- COORDINATE TRANSFORMATION (ENU -> NED) ---
/// ROS 2 standard: ENU (East-North-Up)
/// PX4 standard: NED (North-East-Down)
///
/// Transformation Logic:
/// X_ned = Y_enu (North)
/// Y_ned = X_enu (East)
/// Z_ned = -Z_enu (Down)
///
/// This is critical because if we send a positive Z in ROS (Up),
/// without conversion, PX4 would interpret it as Down (plunging into the ground).
fn enu_to_ned(x: f32, y: f32, z: f32) -> [f32; 3] {
    [y, x, -z]
}

/// Helper to get current timestamp in microseconds, required by PX4 uORB topics.
fn get_clock_microseconds() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or(Duration::from_secs(0))
        .as_micros() as u64
}

struct OffboardControlNode {
    offboard_control_mode_publisher: Arc<rclrs::Publisher<OffboardControlMode>>,
    trajectory_setpoint_publisher: Arc<rclrs::Publisher<TrajectorySetpoint>>,
    vehicle_command_publisher: Arc<rclrs::Publisher<VehicleCommand>>,
    offboard_setpoint_counter: u64,
}

impl OffboardControlNode {
    fn new(node: &mut rclrs::Node) -> Result<Self, rclrs::RclrsError> {
        // --- QUALITY OF SERVICE (QoS) CONFIGURATION ---
        // PX4's uXRCE-DDS bridge uses "Best Effort" for high-frequency topics.
        // A "Reliable" (default) ROS 2 subscriber/publisher will NOT communicate with
        // a "Best Effort" peer. We must match the profile.
        // Reliability: BestEffort (Low latency, no retransmissions)
        // Durability: TransientLocal (Late-joining subscribers get the last message)
        // History: KeepLast(1) (Only the freshest data matters for control)
        let qos_best_effort = rclrs::QosProfile {
            reliability: rclrs::ReliabilityPolicy::BestEffort,
            durability: rclrs::DurabilityPolicy::TransientLocal,
            history: rclrs::HistoryPolicy::KeepLast,
            depth: 1,
            ..QOS_PROFILE_DEFAULT
        };

        // Command topics usually require "Reliable" to ensure the mode change or arming occurs.
        let qos_reliable = rclrs::QosProfile {
            reliability: rclrs::ReliabilityPolicy::Reliable,
            durability: rclrs::DurabilityPolicy::TransientLocal,
            history: rclrs::HistoryPolicy::KeepLast,
            depth: 1,
            ..QOS_PROFILE_DEFAULT
        };

        let offboard_control_mode_publisher = node.create_publisher::<OffboardControlMode>(
            "/fmu/in/offboard_control_mode",
            qos_best_effort.clone(),
        )?;

        let trajectory_setpoint_publisher = node.create_publisher::<TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint",
            qos_best_effort,
        )?;

        let vehicle_command_publisher = node.create_publisher::<VehicleCommand>(
            "/fmu/in/vehicle_command",
            qos_reliable,
        )?;

        Ok(Self {
            offboard_control_mode_publisher: Arc::new(offboard_control_mode_publisher),
            trajectory_setpoint_publisher: Arc::new(trajectory_setpoint_publisher),
            vehicle_command_publisher: Arc::new(vehicle_command_publisher),
            offboard_setpoint_counter: 0,
        })
    }

    /// Publish the OffboardControlMode heartbeat.
    /// This tells PX4 which control loops are active and satisfies the failsafe.
    fn publish_offboard_control_mode(&self) -> Result<(), rclrs::RclrsError> {
        let mut msg = OffboardControlMode::default();
        msg.timestamp = get_clock_microseconds();
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;

        self.offboard_control_mode_publisher.publish(&msg)
    }

    /// Publish a TrajectorySetpoint in NED coordinates.
    fn publish_trajectory_setpoint(&self, enu_pos: Vector3<f32>) -> Result<(), rclrs::RclrsError> {
        let mut msg = TrajectorySetpoint::default();
        msg.timestamp = get_clock_microseconds();

        let ned_pos = enu_to_ned(enu_pos.x, enu_pos.y, enu_pos.z);
        msg.position = ned_pos;
        msg.yaw = 0.0; // 0.0 in NED is North

        // IMPORTANT: Velocity and Acceleration must be NaN to indicate they are uncontrolled
        // in position control mode.
        msg.velocity = [f32::NAN; 3];
        msg.acceleration = [f32::NAN; 3];

        self.trajectory_setpoint_publisher.publish(&msg)
    }

    /// Send a MAVLink-style VehicleCommand to the FMU.
    fn publish_vehicle_command(&self, command: u32, param1: f32, param2: f32) -> Result<(), rclrs::RclrsError> {
        let mut msg = VehicleCommand::default();
        msg.timestamp = get_clock_microseconds();
        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;

        self.vehicle_command_publisher.publish(&msg)
    }

    /// Step 2: Switch to Offboard Mode
    fn arm(&self) -> Result<(), rclrs::RclrsError> {
        // VEHICLE_CMD_COMPONENT_ARM_DISARM (400), param1 = 1.0 (Arm)
        self.publish_vehicle_command(px4_msgs::msg::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
    }

    fn set_offboard_mode(&self) -> Result<(), rclrs::RclrsError> {
        // VEHICLE_CMD_DO_SET_MODE (176), param1 = 1.0 (Custom), param2 = 6.0 (Offboard)
        self.publish_vehicle_command(px4_msgs::msg::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = rclrs::Context::new(env::args())?;
    let mut node = rclrs::Node::new(&context, "offboard_control_node")?;

    let offboard_node = Arc::new(Mutex::new(OffboardControlNode::new(&mut node)?));

    println!("Starting Offboard Control Loop at 20Hz...");

    // Target Hover Setpoint: 5 meters Up (ENU: [0, 0, 5])
    let hover_setpoint = Vector3::new(0.0, 0.0, 5.0);

    // Create a 20Hz timer (50ms period)
    let node_timer = Arc::clone(&offboard_node);
    let _timer = node.create_wall_timer(Duration::from_millis(50), move || {
        let mut n = node_timer.lock().unwrap();

        // Step 1: Stream OffboardControlMode for 10 cycles (0.5s) before switching mode
        n.publish_offboard_control_mode().unwrap();

        if n.offboard_setpoint_counter == 10 {
            // Step 2: Change to Offboard mode and Arm
            println!("Handshake established. Switching to OFFBOARD and ARMING...");
            n.set_offboard_mode().unwrap();
            n.arm().unwrap();
        }

        // Step 3: Continue streaming setpoints
        // Even during the handshake, we should publish setpoints to avoid jumps
        // when the mode actually switches.
        n.publish_trajectory_setpoint(hover_setpoint).unwrap();

        if n.offboard_setpoint_counter < 11 {
            n.offboard_setpoint_counter += 1;
        }
    })?;

    // Spin the node to process the timer callbacks
    // Since rclrs::spin is blocking, and we are in an async main,
    // we can use a thread or just call spin directly if we don't need other async tasks.
    let node_spin = Arc::new(Mutex::new(node));
    let n_clone = Arc::clone(&node_spin);

    std::thread::spawn(move || {
        let n = n_clone.lock().unwrap();
        rclrs::spin(&n).map_err(|e| println!("ROS 2 Spin Error: {:?}", e)).ok();
    });

    println!("Offboard Node is active. Target: Hover at 5m.");

    // Keep the main thread alive
    loop {
        tokio::time::sleep(Duration::from_secs(1)).await;
    }
}
