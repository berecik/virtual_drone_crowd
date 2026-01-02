use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Boid {
    pub drone_id: String,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
}

pub fn calculate_flocking_vector(me: &Boid, neighbors: &[Boid]) -> Vector3<f64> {
    let mut separation = Vector3::new(0.0, 0.0, 0.0);
    let mut alignment = Vector3::new(0.0, 0.0, 0.0);
    let mut cohesion = Vector3::new(0.0, 0.0, 0.0);

    if neighbors.is_empty() {
        return Vector3::new(0.0, 0.0, 0.0);
    }

    let mut center_of_mass = Vector3::new(0.0, 0.0, 0.0);

    for other in neighbors {
        // Separation: inverse square law
        let diff = me.position - other.position;
        let dist_sq = diff.norm_squared();
        if dist_sq > 0.001 {
            separation += diff / dist_sq;
        }

        // Alignment: velocity matching
        alignment += other.velocity;

        // Cohesion: part 1 (summing positions)
        center_of_mass += other.position;
    }

    alignment /= neighbors.len() as f64;
    center_of_mass /= neighbors.len() as f64;
    cohesion = center_of_mass - me.position;

    // Weights (can be adjusted)
    let separation_weight = 1.5;
    let alignment_weight = 1.0;
    let cohesion_weight = 1.0;

    separation * separation_weight + (alignment - me.velocity) * alignment_weight + cohesion * cohesion_weight
}
