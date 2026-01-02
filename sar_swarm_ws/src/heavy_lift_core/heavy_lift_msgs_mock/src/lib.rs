pub mod msg {
    use serde::{Deserialize, Serialize};

    #[derive(Serialize, Deserialize, Debug, Default, Clone)]
    pub struct EvacuateGoal {
        pub latitude: f64,
        pub longitude: f64,
        pub altitude: f64,
        pub priority_code: String,
    }

    #[derive(Serialize, Deserialize, Debug, Default, Clone)]
    pub struct EvacuateResult {
        pub success: bool,
        pub final_status: String,
    }

    #[derive(Serialize, Deserialize, Debug, Default, Clone)]
    pub struct EvacuateFeedback {
        pub distance_to_target: f32,
        pub current_state: String,
    }
}
