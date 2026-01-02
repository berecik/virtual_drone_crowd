/// Virtual Drone Crowd - Phase 1: Core Control Task
///
/// This node implements a PX4 Offboard control loop using ROS 2 (rclrs) and XRCE-DDS.
/// It handles the necessary "proof-of-life" heartbeat, coordinate transformations,
/// and QoS settings required for reliable communication with the Pixhawk 6C.
///
/// Author: Senior Robotics Systems Engineer
/// Project: Virtual Drone Crowd (SAR Swarm)

use rclrs;
use sar_swarm_control::offboard_control_node::{OffboardControlNode, LifecycleState};
use std::env;
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = rclrs::Context::new(env::args())?;

    // 1. Create the node (Starts in Unconfigured)
    let mut offboard_node = OffboardControlNode::new(&context)?;

    // 2. Transition: Unconfigured -> Inactive
    offboard_node.on_configure()?;

    // 3. Transition: Inactive -> Active
    offboard_node.on_activate()?;

    println!("Offboard Lifecycle Node is now ACTIVE.");

    // Simulation of GPS fix
    println!("Waiting for simulated GPS fix...");
    tokio::time::sleep(Duration::from_secs(2)).await;

    // In a real application, we would spin the node here.
    // For this boilerplate demonstration, we'll simulate the 10Hz loop for a few seconds.
    for i in 0..50 {
        // Only publish heartbeat if active and we have simulated "fix"
        if i > 5 {
             offboard_node.publish_heartbeat()?;
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }

    // 4. Transition: Active -> Inactive
    offboard_node.on_deactivate()?;

    Ok(())
}
