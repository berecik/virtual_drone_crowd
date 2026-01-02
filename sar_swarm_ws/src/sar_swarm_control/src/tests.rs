#[cfg(test)]
mod tests {
    use crate::boids::{Boid, calculate_flocking_vector};
    use nalgebra::Vector3;

    #[test]
    fn test_separation_force() {
        let boid1 = Boid {
            drone_id: "drone_1".to_string(),
            position: Vector3::new(0.0, 0.0, 0.0),
            velocity: Vector3::new(0.0, 0.0, 0.0),
        };
        let boid2 = Boid {
            drone_id: "drone_2".to_string(),
            position: Vector3::new(0.1, 0.0, 0.0),
            velocity: Vector3::new(0.0, 0.0, 0.0),
        };

        let force = calculate_flocking_vector(&boid1, &[boid2.clone()]);

        // Force should be pointing away from boid2, so in negative x direction
        assert!(force.x < 0.0, "Force.x should be negative, got {}", force.x);
        assert_eq!(force.y, 0.0);
        assert_eq!(force.z, 0.0);
    }
}
