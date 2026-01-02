/// Virtual Drone Crowd - Swarm Control Library
/// Author: beret <beret@hipisi.org.pl>
/// Company: Marysia Software Limited <ceo@marysia.app>
/// Domain: app.marysia.drone
/// Website: https://marysia.app

use rclrs;

pub mod boids;
pub mod communication;
pub mod utils;
pub mod search;
#[path = "main.rs"]
pub mod main_module;
#[cfg(test)]
pub mod tests;

pub fn hello_swarm() {
    println!("Hello from SAR Swarm Control!");
}

/// Calculate the target position based on leader position and offset.
pub fn calculate_target_pos(leader_pos: [f32; 3], offset: [f32; 3]) -> [f32; 3] {
    [
        leader_pos[0] + offset[0],
        leader_pos[1] + offset[1],
        leader_pos[2] + offset[2],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hello_swarm() {
        // Simple test to verify test infra
        hello_swarm();
    }

    #[test]
    fn test_calculate_target_pos() {
        let leader_pos = [1.0, 2.0, 3.0];
        let offset = [0.5, -1.0, 2.0];
        let expected = [1.5, 1.0, 5.0];
        assert_eq!(calculate_target_pos(leader_pos, offset), expected);
    }
}
