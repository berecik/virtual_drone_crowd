use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Boid {
    pub drone_id: String,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub timestamp: u64, // microseconds
}

#[derive(Clone, Debug)]
pub struct FlockingParams {
    pub neighbor_radius: f64,
    pub separation_radius: f64,
    pub separation_weight: f64,
    pub alignment_weight: f64,
    pub cohesion_weight: f64,
}

impl Default for FlockingParams {
    fn default() -> Self {
        Self {
            neighbor_radius: 10.0,
            separation_radius: 2.0,
            separation_weight: 2.0,
            alignment_weight: 1.0,
            cohesion_weight: 1.0,
        }
    }
}

pub fn calculate_flocking_vector(me: &Boid, neighbors: &[Boid], params: &FlockingParams) -> Vector3<f64> {
    let mut separation = Vector3::new(0.0, 0.0, 0.0);
    let mut alignment = Vector3::new(0.0, 0.0, 0.0);
    let mut cohesion = Vector3::new(0.0, 0.0, 0.0);

    if neighbors.is_empty() {
        return Vector3::new(0.0, 0.0, 0.0);
    }

    let mut neighbors_count = 0;
    let mut center_of_mass = Vector3::new(0.0, 0.0, 0.0);

    for other in neighbors {
        let diff = me.position - other.position;
        let dist = diff.norm();

        if dist < params.neighbor_radius && dist > 0.0 {
            neighbors_count += 1;

            // Separation: move away from neighbors that are too close
            if dist < params.separation_radius {
                separation += diff.normalize() / dist;
            }

            // Alignment: match velocity of neighbors
            alignment += other.velocity;

            // Cohesion: move towards average position of neighbors
            center_of_mass += other.position;
        }
    }

    if neighbors_count > 0 {
        alignment /= neighbors_count as f64;
        center_of_mass /= neighbors_count as f64;
        cohesion = center_of_mass - me.position;
    }

    separation * params.separation_weight + (alignment - me.velocity) * params.alignment_weight + cohesion * params.cohesion_weight
}
