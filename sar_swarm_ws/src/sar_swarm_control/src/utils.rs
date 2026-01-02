use std::time::{Duration, SystemTime, UNIX_EPOCH};

/// --- COORDINATE TRANSFORMATION (ENU -> NED) ---
/// ROS 2 standard: ENU (East-North-Up)
/// PX4 standard: NED (North-East-Down)
///
/// Transformation Logic:
/// X_ned = Y_enu (North)
/// Y_ned = X_enu (East)
/// Z_ned = -Z_enu (Down)
pub fn enu_to_ned(x: f32, y: f32, z: f32) -> [f32; 3] {
    [y, x, -z]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_enu_to_ned() {
        // ENU: X=East, Y=North, Z=Up
        // NED: X=North, Y=East, Z=Down
        // Transformation: [y_enu, x_enu, -z_enu]
        let enu = (1.0, 2.0, 3.0); // East=1, North=2, Up=3
        let ned = enu_to_ned(enu.0, enu.1, enu.2);
        assert_eq!(ned, [2.0, 1.0, -3.0]); // North=2, East=1, Down=-3
    }
}

/// Helper to get current timestamp in microseconds, required by PX4 uORB topics.
pub fn get_clock_microseconds() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or(Duration::from_secs(0))
        .as_micros() as u64
}
