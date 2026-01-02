pub mod msg {
    use serde::{Deserialize, Serialize};

    #[derive(Serialize, Deserialize, Debug, Default, Clone)]
    pub struct VehicleOdometry {
        pub position: [f32; 3],
        pub q: [f32; 4],
        // Add other fields as needed for compatibility, but main.rs only uses position
    }

    #[derive(Serialize, Deserialize, Debug, Default, Clone)]
    pub struct TrajectorySetpoint {
        pub position: [f32; 3],
        pub velocity: [f32; 3],
        pub acceleration: [f32; 3],
        pub jerk: [f32; 3],
        pub yaw: f32,
        pub yawspeed: f32,
    }
    #[derive(Serialize, Deserialize, Debug, Default, Clone)]
    pub struct PoseStamped {
        pub pose: Pose,
    }

    #[derive(Serialize, Deserialize, Debug, Default, Clone)]
    pub struct Pose {
        pub position: Point,
    }

    #[derive(Serialize, Deserialize, Debug, Default, Clone)]
    pub struct Point {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    #[derive(Serialize, Deserialize, Debug, Default, Clone)]
    pub struct Twist {
        pub linear: Vector3,
    }

    #[derive(Serialize, Deserialize, Debug, Default, Clone)]
    pub struct Vector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }
}
