use crate::utils::{enu_to_ned, get_clock_microseconds};
use crate::LifecycleState;
use crate::OffboardControlNode;
use nalgebra::Vector3;

#[test]
fn test_lifecycle_initial_state() {
    let node = OffboardControlNode::new();
    assert_eq!(node.state, LifecycleState::Unconfigured);
}

#[test]
fn test_lifecycle_transitions() {
    let mut node = OffboardControlNode::new();

    // Manual transition for testing logic since on_configure requires rclrs::Node
    node.state = LifecycleState::Inactive;
    assert_eq!(node.state, LifecycleState::Inactive);

    node.on_activate().unwrap();
    assert_eq!(node.state, LifecycleState::Active);

    node.on_deactivate().unwrap();
    assert_eq!(node.state, LifecycleState::Inactive);

    node.on_cleanup().unwrap();
    assert_eq!(node.state, LifecycleState::Unconfigured);
}

#[test]
fn test_enu_to_ned_conversion_standalone() {
    let enu = Vector3::new(1.0, 2.0, 3.0);
    let ned = enu_to_ned(enu.x, enu.y, enu.z);
    assert_eq!(ned, [2.0, 1.0, -3.0]);
}

#[test]
fn test_timestamp_micro() {
    let ts1 = get_clock_microseconds();
    std::thread::sleep(std::time::Duration::from_millis(1));
    let ts2 = get_clock_microseconds();
    assert!(ts2 > ts1);
}
