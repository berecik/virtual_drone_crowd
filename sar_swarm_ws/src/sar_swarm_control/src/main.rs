/// Virtual Drone Crowd - Swarm Control Node
/// Author: beret <beret@hipisi.org.pl>
/// Company: Marysia Software Limited <ceo@marysia.app>
/// Domain: app.marysia.drone
/// Website: https://marysia.app

use rclrs;
use std::env;
use std::sync::{Arc, Mutex};
use nalgebra::Vector3;
use sar_swarm_control::boids::{Boid, calculate_flocking_vector};
use sar_swarm_control::communication::ZenohManager;
use px4_msgs::msg::{PoseStamped, Twist};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = rclrs::Context::new(env::args())?;
    let args: Vec<String> = env::args().collect();
    let drone_id_num: u32 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(1);
    let drone_id = format!("drone_{}", drone_id_num);

    let node_name = format!("swarm_node_{}", drone_id_num);
    let mut node = rclrs::Node::new(&context, &node_name)?;

    let my_boid = Arc::new(Mutex::new(Boid {
        drone_id: drone_id.clone(),
        position: Vector3::new(0.0, 0.0, 0.0),
        velocity: Vector3::new(0.0, 0.0, 0.0),
    }));

    let zenoh = Arc::new(ZenohManager::new(drone_id.clone()).await);

    let vel_pub = node.create_publisher::<Twist>(
        "/mavros/setpoint_velocity/cmd_vel_unstamped",
        rclrs::QOS_PROFILE_DEFAULT,
    )?;

    let my_boid_cb = Arc::clone(&my_boid);
    let _pose_sub = node.create_subscription::<PoseStamped, _>(
        "/mavros/local_position/pose",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: PoseStamped| {
            let mut b = my_boid_cb.lock().unwrap();
            b.position = Vector3::new(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
        },
    )?;

    let my_boid_timer = Arc::clone(&my_boid);
    let zenoh_timer = Arc::clone(&zenoh);
    let neighbors_timer = Arc::clone(&zenoh.neighbors);

    let _timer = node.create_wall_timer(std::time::Duration::from_millis(100), move || {
        let (me, neighbors_list) = {
            let me = my_boid_timer.lock().unwrap().clone();
            let neighbors = neighbors_timer.lock().unwrap();
            let neighbors_list: Vec<Boid> = neighbors.values().cloned().collect();
            (me, neighbors_list)
        };

        let flocking_force = calculate_flocking_vector(&me, &neighbors_list);

        // Update own velocity (simple integration for simulation)
        {
            let mut me_lock = my_boid_timer.lock().unwrap();
            me_lock.velocity += flocking_force * 0.1; // dt = 0.1
            // Limit velocity
            let max_vel = 2.0;
            if me_lock.velocity.norm() > max_vel {
                me_lock.velocity = me_lock.velocity.normalize() * max_vel;
            }
        }

        let final_vel = my_boid_timer.lock().unwrap().velocity;

        // Publish to Zenoh
        let zenoh_pub = Arc::clone(&zenoh_timer);
        let me_for_zenoh = my_boid_timer.lock().unwrap().clone();
        tokio::spawn(async move {
            zenoh_pub.publish_state(&me_for_zenoh).await;
        });

        // Publish to ROS 2
        let mut twist = Twist::default();
        twist.linear.x = final_vel.x;
        twist.linear.y = final_vel.y;
        twist.linear.z = final_vel.z;
        let _ = vel_pub.publish(&twist);
    })?;

    println!("Swarm Node {} started!", drone_id);

    // Use a separate thread for ROS 2 spinning to not block tokio
    let node_arc = Arc::new(Mutex::new(node));
    let node_spin = Arc::clone(&node_arc);
    std::thread::spawn(move || {
        let n = node_spin.lock().unwrap();
        rclrs::spin(&n).unwrap();
    });

    // Keep the main task alive
    loop {
        tokio::time::sleep(std::time::Duration::from_secs(1)).await;
    }
}
