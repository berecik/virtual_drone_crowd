use rclrs;
use std::env;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use heavy_lift_msgs::msg::{EvacuateGoal, EvacuateResult, EvacuateFeedback};

#[derive(Debug, PartialEq, Clone, Copy)]
enum ExtractionState {
    IDLE,
    EN_ROUTE,
    DESCENDING,
    LIFTING,
    RETURN,
}

struct EvacuationDirector {
    state: ExtractionState,
    current_goal: Option<EvacuateGoal>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = rclrs::Context::new(env::args())?;
    let mut node = rclrs::Node::new(&context, "evacuation_director")?;

    let director_state = Arc::new(Mutex::new(EvacuationDirector {
        state: ExtractionState::IDLE,
        current_goal: None,
    }));

    // Mocking an action server with a simple subscription for goals
    // and a publisher for feedback/results
    let state_clone = Arc::clone(&director_state);
    let _goal_sub = node.create_subscription::<EvacuateGoal, _>(
        "evacuate/goal",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: EvacuateGoal| {
            let mut s = state_clone.lock().unwrap();
            if s.state == ExtractionState::IDLE {
                println!("Received new evacuation goal to {:?}", msg);
                s.current_goal = Some(msg);
                s.state = ExtractionState::EN_ROUTE;
            }
        },
    )?;

    let feedback_pub = node.create_publisher::<EvacuateFeedback>(
        "evacuate/feedback",
        rclrs::QOS_PROFILE_DEFAULT,
    )?;

    let result_pub = node.create_publisher::<EvacuateResult>(
        "evacuate/result",
        rclrs::QOS_PROFILE_DEFAULT,
    )?;

    let state_timer_cb = Arc::clone(&director_state);
    let _timer = node.create_wall_timer(Duration::from_secs(2), move || {
        let mut s = state_timer_cb.lock().unwrap();

        let next_state = match s.state {
            ExtractionState::IDLE => None,
            ExtractionState::EN_ROUTE => {
                println!("State: EN_ROUTE -> DESCENDING");
                Some(ExtractionState::DESCENDING)
            },
            ExtractionState::DESCENDING => {
                println!("State: DESCENDING -> LIFTING");
                Some(ExtractionState::LIFTING)
            },
            ExtractionState::LIFTING => {
                println!("State: LIFTING -> RETURN");
                Some(ExtractionState::RETURN)
            },
            ExtractionState::RETURN => {
                println!("State: RETURN -> IDLE");

                let result = EvacuateResult {
                    success: true,
                    final_status: "Evacuation completed successfully".to_string(),
                };
                let _ = result_pub.publish(&result);

                Some(ExtractionState::IDLE)
            },
        };

        if let Some(ns) = next_state {
            s.state = ns;

            let feedback = EvacuateFeedback {
                distance_to_target: 0.0, // Mock distance
                current_state: format!("{:?}", s.state),
            };
            let _ = feedback_pub.publish(&feedback);
        }
    })?;

    println!("Evacuation Director started!");
    rclrs::spin(&node)?;

    Ok(())
}
