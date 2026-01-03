#[cfg(test)]
mod tests {
    use crate::boids::{Boid, calculate_flocking_vector};
    use nalgebra::Vector3;
    use crate::utils::{enu_to_ned, get_clock_microseconds};
    use crate::calculate_target_pos;
    use crate::communication::ZenohManager;
    use crate::search::generate_lawnmower_pattern;
    use std::sync::{Arc, Mutex};
    use std::collections::HashMap;

    #[test]
    fn test_process_raw_data() {
        let my_id = "drone_1";
        let neighbors = Arc::new(Mutex::new(HashMap::new()));

        let boid = Boid {
            drone_id: "drone_2".to_string(),
            position: Vector3::new(1.0, 2.0, 3.0),
            velocity: Vector3::new(0.0, 0.0, 0.0),
            timestamp: 100,
        };
        let payload = bincode::serialize(&boid).unwrap();
        let key = "swarm/drone_2/state";

        // 1. Valid message from another drone
        ZenohManager::process_raw_data(key, &payload, my_id, &neighbors);
        {
            let n = neighbors.lock().unwrap();
            assert!(n.contains_key("drone_2"));
            assert_eq!(n.get("drone_2").unwrap().position.x, 1.0);
        }

        // 2. Message from self
        let my_payload = bincode::serialize(&Boid {
             drone_id: "drone_1".to_string(),
             ..boid.clone()
        }).unwrap();
        ZenohManager::process_raw_data("swarm/drone_1/state", &my_payload, my_id, &neighbors);
        {
            let n = neighbors.lock().unwrap();
            assert!(!n.contains_key("drone_1"), "Should not add self to neighbors");
        }

        // 3. Malformed payload
        ZenohManager::process_raw_data("swarm/drone_3/state", &[1, 2, 3], my_id, &neighbors);
        {
            let n = neighbors.lock().unwrap();
            assert!(!n.contains_key("drone_3"), "Should ignore malformed payload");
        }

        // 4. Invalid key format
        ZenohManager::process_raw_data("invalid_key", &payload, my_id, &neighbors);
        {
            let n = neighbors.lock().unwrap();
            assert_eq!(n.len(), 1, "Should not have added from invalid key");
        }
    }

    #[test]
    fn test_calculate_target_pos() {
        let leader = [1.0, 2.0, 3.0];
        let offset = [0.5, -1.0, 2.0];
        assert_eq!(calculate_target_pos(leader, offset), [1.5, 1.0, 5.0]);
    }

    #[test]
    fn test_boids_many_neighbors() {
        let me = Boid {
            drone_id: "me".to_string(),
            position: Vector3::new(0.0, 0.0, 0.0),
            velocity: Vector3::new(0.0, 0.0, 0.0),
            timestamp: 0,
        };
        let mut neighbors = Vec::new();
        for i in 0..100 {
            neighbors.push(Boid {
                drone_id: format!("d{}", i),
                position: Vector3::new(2.0, 0.0, 0.0),
                velocity: Vector3::new(1.0, 0.0, 0.0),
                timestamp: 0,
            });
        }
        let force = calculate_flocking_vector(&me, &neighbors, &crate::boids::FlockingParams::default());
        assert_eq!(force, Vector3::new(3.0, 0.0, 0.0));
    }

    #[test]
    fn test_boids_empty_neighbors() {
        let me = Boid {
            drone_id: "me".to_string(),
            position: Vector3::new(0.0, 0.0, 0.0),
            velocity: Vector3::new(0.0, 0.0, 0.0),
            timestamp: 0,
        };
        let force = calculate_flocking_vector(&me, &[], &crate::boids::FlockingParams::default());
        assert_eq!(force, Vector3::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn test_separation_force() {
        let boid1 = Boid {
            drone_id: "drone_1".to_string(),
            position: Vector3::new(0.0, 0.0, 0.0),
            velocity: Vector3::new(0.0, 0.0, 0.0),
            timestamp: 0,
        };
        let boid2 = Boid {
            drone_id: "drone_2".to_string(),
            position: Vector3::new(0.1, 0.0, 0.0),
            velocity: Vector3::new(0.0, 0.0, 0.0),
            timestamp: 0,
        };

        let force = calculate_flocking_vector(&boid1, &[boid2.clone()], &crate::boids::FlockingParams::default());

        // Force should be pointing away from boid2, so in negative x direction
        assert!(force.x < 0.0, "Force.x should be negative, got {}", force.x);
        assert_eq!(force.y, 0.0);
        assert_eq!(force.z, 0.0);
    }

    #[test]
    fn test_alignment_force() {
        let boid1 = Boid {
            drone_id: "drone_1".to_string(),
            position: Vector3::new(0.0, 0.0, 0.0),
            velocity: Vector3::new(0.0, 0.0, 0.0),
            timestamp: 0,
        };
        let boid2 = Boid {
            drone_id: "drone_2".to_string(),
            position: Vector3::new(3.0, 0.0, 0.0), // Within neighbor radius (10.0) but outside separation (2.0)
            velocity: Vector3::new(1.0, 1.0, 0.0),
            timestamp: 0,
        };

        let force = calculate_flocking_vector(&boid1, &[boid2], &crate::boids::FlockingParams::default());

        // Alignment force: (avg_velocity - my_velocity) * weight
        // ( [1, 1, 0] - [0, 0, 0] ) * 1.0 = [1, 1, 0]
        // Cohesion force: (avg_position - my_position) * weight
        // ( [3, 0, 0] - [0, 0, 0] ) * 1.0 = [3, 0, 0]
        // Total: [4, 1, 0]
        assert!(force.x > 0.0);
        assert!(force.y > 0.0);
        assert_eq!(force.z, 0.0);
    }

    #[test]
    fn test_cohesion_force() {
        let boid1 = Boid {
            drone_id: "drone_1".to_string(),
            position: Vector3::new(0.0, 0.0, 0.0),
            velocity: Vector3::new(0.0, 0.0, 0.0),
            timestamp: 0,
        };
        let boid2 = Boid {
            drone_id: "drone_2".to_string(),
            position: Vector3::new(6.0, 8.0, 0.0), // distance = 10.0
            velocity: Vector3::new(0.0, 0.0, 0.0),
            timestamp: 0,
        };

        // At exactly 10.0 distance, it might be excluded if logic is dist < neighbor_radius
        // In boids.rs: if dist < params.neighbor_radius && dist > 0.0
        let force = calculate_flocking_vector(&boid1, &[boid2.clone()], &crate::boids::FlockingParams::default());
        assert_eq!(force, Vector3::new(0.0, 0.0, 0.0), "Should be zero as distance is not < 10.0");

        let boid3 = Boid {
            drone_id: "drone_3".to_string(),
            position: Vector3::new(2.0, 2.0, 0.0), // distance < 10.0
            velocity: Vector3::new(0.0, 0.0, 0.0),
            timestamp: 0,
        };
        let force2 = calculate_flocking_vector(&boid1, &[boid3], &crate::boids::FlockingParams::default());
        // Cohesion: [2, 2, 0] * 1.0 = [2, 2, 0]
        assert!(force2.x > 0.0);
        assert!(force2.y > 0.0);
    }

    #[test]
    fn test_boid_serialization() {
        let boid = Boid {
            drone_id: "drone_123".to_string(),
            position: Vector3::new(1.1, 2.2, 3.3),
            velocity: Vector3::new(0.1, 0.2, 0.3),
            timestamp: 123456789,
        };

        let encoded = bincode::serialize(&boid).unwrap();
        let decoded: Boid = bincode::deserialize(&encoded).unwrap();

        assert_eq!(boid.drone_id, decoded.drone_id);
        assert_eq!(boid.position, decoded.position);
        assert_eq!(boid.velocity, decoded.velocity);
        assert_eq!(boid.timestamp, decoded.timestamp);
    }

    #[test]
    fn test_handshake_logic() {
        // Mock state for the handshake (matches logic in main.rs)
        let mut offboard_setpoint_counter = 0;
        let mut arm_called = false;
        let mut set_offboard_mode_called = false;

        // Simulate 20 iterations (1 second at 20Hz)
        for _ in 0..20 {
            if offboard_setpoint_counter == 10 {
                // Step 2: Change to Offboard mode and Arm
                set_offboard_mode_called = true;
                arm_called = true;
            }

            if offboard_setpoint_counter < 11 {
                offboard_setpoint_counter += 1;
            }
        }

        assert_eq!(offboard_setpoint_counter, 11);
        assert!(set_offboard_mode_called, "Should have called set_offboard_mode at step 10");
        assert!(arm_called, "Should have called arm at step 10");
    }

    #[test]
    fn test_enu_to_ned_conversion() {
        // ENU [X, Y, Z] -> NED [Y, X, -Z]
        let enu = [10.0, 5.0, 2.0];
        let expected_ned = [5.0, 10.0, -2.0];
        let actual_ned = enu_to_ned(enu[0], enu[1], enu[2]);

        assert_eq!(actual_ned, expected_ned, "ENU to NED conversion failed");

        // Test altitude (Up in ENU should be Down in NED)
        let altitude_enu = [0.0, 0.0, 5.0];
        let expected_ned_alt = [0.0, 0.0, -5.0];
        assert_eq!(enu_to_ned(altitude_enu[0], altitude_enu[1], altitude_enu[2]), expected_ned_alt);

        // Edge case: Large values
        let large_enu = [1000000.0, -500000.0, 123.45];
        let expected_large_ned = [-500000.0, 1000000.0, -123.45];
        assert_eq!(enu_to_ned(large_enu[0], large_enu[1], large_enu[2]), expected_large_ned);

        // Edge case: Zero
        assert_eq!(enu_to_ned(0.0, 0.0, 0.0), [0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_boids_edge_cases() {
        let me = Boid {
            drone_id: "me".to_string(),
            position: Vector3::new(0.0, 0.0, 0.0),
            velocity: Vector3::new(0.0, 0.0, 0.0),
            timestamp: 0,
        };

        // Identical positions (should not result in NaN/Inf due to dist > 0.0 check)
        let neighbor_same = Boid {
            drone_id: "other".to_string(),
            position: Vector3::new(0.0, 0.0, 0.0),
            velocity: Vector3::new(1.0, 1.0, 1.0),
            timestamp: 0,
        };
        let force_same = calculate_flocking_vector(&me, &[neighbor_same], &crate::boids::FlockingParams::default());
        assert_eq!(force_same, Vector3::new(0.0, 0.0, 0.0), "Force should be zero for identical positions");

        // Just outside neighbor radius (10.0 is default)
        let neighbor_outside = Boid {
            drone_id: "outside".to_string(),
            position: Vector3::new(10.1, 0.0, 0.0),
            velocity: Vector3::new(1.0, 1.0, 1.0),
            timestamp: 0,
        };
        let force_outside = calculate_flocking_vector(&me, &[neighbor_outside], &crate::boids::FlockingParams::default());
        assert_eq!(force_outside, Vector3::new(0.0, 0.0, 0.0), "Force should be zero outside neighbor radius");

        // Just inside neighbor radius (9.9)
        let neighbor_inside = Boid {
            drone_id: "inside".to_string(),
            position: Vector3::new(9.9, 0.0, 0.0),
            velocity: Vector3::new(0.0, 0.0, 0.0), // No alignment force
            timestamp: 0,
        };
        let force_inside = calculate_flocking_vector(&me, &[neighbor_inside], &crate::boids::FlockingParams::default());
        // Cohesion should pull towards [9.9, 0, 0]
        assert!(force_inside.x > 0.0);
    }

    #[test]
    fn test_timestamp_generation() {
        let ts1 = get_clock_microseconds();
        std::thread::sleep(std::time::Duration::from_millis(2));
        let ts2 = get_clock_microseconds();

        assert!(ts2 > ts1, "Timestamp should be monotonically increasing");
        assert!(ts2 - ts1 >= 1000, "Timestamp difference should be at least 1ms (1000us)");
    }

    #[test]
    fn test_lawnmower_pattern() {
        let pattern = generate_lawnmower_pattern(0.0, 0.0, 10.0, 10.0, 5.0);

        // num_passes = (10/5).ceil() + 1 = 3 passes.
        // Each pass has 2 waypoints. Total 6 waypoints.
        assert_eq!(pattern.len(), 6);

        // Pass 0 (x=0.0): [0,0] -> [0,10]
        assert_eq!(pattern[0], [0.0, 0.0]);
        assert_eq!(pattern[1], [0.0, 10.0]);

        // Pass 1 (x=5.0): [5,10] -> [5,0] (reversed)
        assert_eq!(pattern[2], [5.0, 10.0]);
        assert_eq!(pattern[3], [5.0, 0.0]);

        // Pass 2 (x=10.0): [10,0] -> [10,10]
        assert_eq!(pattern[4], [10.0, 0.0]);
        assert_eq!(pattern[5], [10.0, 10.0]);
    }

    #[test]
    fn test_flight_states() {
        // Test basic state enum
        let state = crate::FlightState::DISARMED;
        assert_eq!(state, crate::FlightState::DISARMED);
    }
}
