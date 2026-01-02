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
        pub timestamp: u64,
        pub position: [f32; 3],
        pub velocity: [f32; 3],
        pub acceleration: [f32; 3],
        pub jerk: [f32; 3],
        pub yaw: f32,
        pub yawspeed: f32,
    }
    #[derive(Serialize, Deserialize, Debug, Default, Clone)]
    pub struct OffboardControlMode {
        pub timestamp: u64,
        pub position: bool,
        pub velocity: bool,
        pub acceleration: bool,
        pub attitude: bool,
        pub body_rate: bool,
        pub thrust_and_try: bool,
    }
    #[derive(Serialize, Deserialize, Debug, Default, Clone)]
    pub struct VehicleCommand {
        pub timestamp: u64,
        pub param1: f32,
        pub param2: f32,
        pub param3: f32,
        pub param4: f32,
        pub param5: f32,
        pub param6: f32,
        pub param7: f32,
        pub command: u32,
        pub target_system: u8,
        pub target_component: u8,
        pub source_system: u8,
        pub source_component: u16,
        pub confirmation: u8,
        pub from_external: bool,
    }

    #[derive(Serialize, Deserialize, Debug, Default, Clone)]
    pub struct VehicleGlobalPosition {
        pub timestamp: u64,
        pub lat: f64,
        pub lon: f64,
        pub alt: f32,
        pub eph: f32,
        pub epv: f32,
    }

    pub const VEHICLE_CMD_DO_SET_MODE: u32 = 176;
    pub const VEHICLE_CMD_COMPONENT_ARM_DISARM: u32 = 400;

    pub mod srv {
        use serde::{Deserialize, Serialize};
        use super::VehicleCommand;

        #[derive(Serialize, Deserialize, Debug, Default, Clone)]
        pub struct VehicleCommand_Request {
            pub request: VehicleCommand,
        }

        #[derive(Serialize, Deserialize, Debug, Default, Clone)]
        pub struct VehicleCommand_Response {
            pub success: bool,
            pub result: u8,
        }
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
