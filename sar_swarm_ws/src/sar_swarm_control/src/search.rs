/// Search pattern generation for SAR missions.

/// Generates a lawnmower (boustrophedon) search pattern.
/// Returns a list of (x, y) waypoints.
pub fn generate_lawnmower_pattern(
    start_x: f32,
    start_y: f32,
    width: f32,
    height: f32,
    spacing: f32,
) -> Vec<[f32; 2]> {
    let mut waypoints = Vec::new();
    let num_passes = (width / spacing).ceil() as i32 + 1;

    for i in 0..num_passes {
        let x = start_x + (i as f32 * spacing);
        if i % 2 == 0 {
            // Move up
            waypoints.push([x, start_y]);
            waypoints.push([x, start_y + height]);
        } else {
            // Move down
            waypoints.push([x, start_y + height]);
            waypoints.push([x, start_y]);
        }
    }
    waypoints
}
