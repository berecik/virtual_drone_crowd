use heavy_lift_msgs::msg::EvacuateGoal;

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum ExtractionState {
    IDLE,
    EN_ROUTE,
    DESCENDING,
    LIFTING,
    RETURN,
}

pub struct EvacuationDirector {
    pub state: ExtractionState,
    pub current_goal: Option<EvacuateGoal>,
}

impl EvacuationDirector {
    pub fn new() -> Self {
        Self {
            state: ExtractionState::IDLE,
            current_goal: None,
        }
    }

    pub fn next_state(&mut self) -> Option<ExtractionState> {
        match self.state {
            ExtractionState::IDLE => None,
            ExtractionState::EN_ROUTE => Some(ExtractionState::DESCENDING),
            ExtractionState::DESCENDING => Some(ExtractionState::LIFTING),
            ExtractionState::LIFTING => Some(ExtractionState::RETURN),
            ExtractionState::RETURN => Some(ExtractionState::IDLE),
        }
    }

    pub fn transition(&mut self) -> bool {
        if let Some(ns) = self.next_state() {
            self.state = ns;
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_state_transitions() {
        let mut director = EvacuationDirector::new();
        assert_eq!(director.state, ExtractionState::IDLE);

        // Manually trigger EN_ROUTE
        director.state = ExtractionState::EN_ROUTE;

        assert!(director.transition());
        assert_eq!(director.state, ExtractionState::DESCENDING);

        assert!(director.transition());
        assert_eq!(director.state, ExtractionState::LIFTING);

        assert!(director.transition());
        assert_eq!(director.state, ExtractionState::RETURN);

        assert!(director.transition());
        assert_eq!(director.state, ExtractionState::IDLE);

        assert!(!director.transition());
    }
}
