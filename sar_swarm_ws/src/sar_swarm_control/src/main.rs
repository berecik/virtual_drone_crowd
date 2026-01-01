use rclrs;
use std::env;
use std::sync::{Arc, Mutex};
use px4_msgs::msg::VehicleOdometry;
use px4_msgs::msg::TrajectorySetpoint;

struct DroneControl {
    id: u32,
    target_id: Option<u32>,
    current_pos: [f32; 3],
    leader_pos: [f32; 3],
    offset: [f32; 3],
}

fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(env::args())?;

    let args: Vec<String> = env::args().collect();
    let drone_id: u32 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(1);
    let target_id: Option<u32> = args.get(2).and_then(|s| s.parse().ok());

    let node_name = format!("drone_control_{}", drone_id);
    let node = rclrs::create_node(&context, &node_name)?;

    let state = Arc::new(Mutex::new(DroneControl {
        id: drone_id,
        target_id,
        current_pos: [0.0, 0.0, 0.0],
        leader_pos: [0.0, 0.0, 0.0],
        offset: if target_id.is_some() {
            match drone_id {
                2 => [2.0, 0.0, 0.0],
                3 => [0.0, 2.0, 0.0],
                4 => [2.0, 2.0, 0.0],
                _ => [2.0, 0.0, 0.0],
            }
        } else {
            [0.0, 0.0, 0.0]
        },
    }));

    let setpoint_pub = node.create_publisher::<TrajectorySetpoint>(
        &format!("/drone_{}/fmu/in/trajectory_setpoint", drone_id),
        rclrs::QOS_PROFILE_DEFAULT,
    )?;

    let state_cb = Arc::clone(&state);
    let _odom_sub = node.create_subscription::<VehicleOdometry, _>(
        &format!("/drone_{}/fmu/out/vehicle_odometry", drone_id),
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: VehicleOdometry| {
            let mut s = state_cb.lock().unwrap();
            s.current_pos = msg.position;
        },
    )?;

    if let Some(tid) = target_id {
        let state_leader_cb = Arc::clone(&state);
        let _leader_sub = node.create_subscription::<VehicleOdometry, _>(
            &format!("/drone_{}/fmu/out/vehicle_odometry", tid),
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg: VehicleOdometry| {
                let mut s = state_leader_cb.lock().unwrap();
                s.leader_pos = msg.position;
            },
        )?;
    }

    // Goal subscriber for leader
    let state_goal_cb = Arc::clone(&state);
    let _goal_sub = node.create_subscription::<VehicleOdometry, _>(
        "/swarm/goal",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: VehicleOdometry| {
            let mut s = state_goal_cb.lock().unwrap();
            if s.target_id.is_none() {
                s.leader_pos = msg.position;
            }
        },
    )?;

    let state_timer_cb = Arc::clone(&state);
    let _timer = node.create_timer(std::time::Duration::from_millis(100), move || {
        let s = state_timer_cb.lock().unwrap();
        let mut setpoint = TrajectorySetpoint::default();

        let target_x = s.leader_pos[0] + s.offset[0];
        let target_y = s.leader_pos[1] + s.offset[1];
        let target_z = s.leader_pos[2] + s.offset[2];

        setpoint.position = [target_x, target_y, target_z];
        setpoint.velocity = [f32::NAN, f32::NAN, f32::NAN];
        setpoint.acceleration = [f32::NAN, f32::NAN, f32::NAN];
        setpoint.jerk = [f32::NAN, f32::NAN, f32::NAN];
        setpoint.yaw = 0.0;
        setpoint.yawspeed = f32::NAN;

        let _ = setpoint_pub.publish(&setpoint);
    })?;

    println!("Drone {} Control started!", drone_id);
    rclrs::spin(&node)
}
