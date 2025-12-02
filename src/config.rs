use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Config {
    // map params
    pub voxel_size: f32,
    pub max_range: f32,
    pub min_range: f32,
    pub max_points_per_voxel: u16,

    // th parms
    pub min_motion_th: f64,
    pub initial_threshold: f64,

    // registration params
    pub max_num_iterations: u16,
    pub convergence_criterion: f64,
    pub max_num_threads: u8,

    // Motion compensation
    pub deskew: bool,

    // Point aging (None = disabled, Some(seconds) = enabled)
    pub max_point_age_seconds: Option<f64>,

    /// Maximum allowed distance between consecutive poses for deskewing (in meters)
    pub max_distance_between_poses: f64,
    /// Maximum allowed angle between consecutive poses for deskewing (in radians)
    pub max_angle_between_poses: f64,
}
impl Config {
    pub fn default_values() -> Config {
        Config {
            voxel_size: 1.0,
            max_range: 100.0,
            min_range: 1.0,
            max_points_per_voxel: 20,

            // th parms
            min_motion_th: 0.1,
            initial_threshold: 2.0,

            // registration params
            max_num_iterations: 500,
            convergence_criterion: 0.0001,
            max_num_threads: 0,

            // Motion compensation
            deskew: false,

            // Point aging
            max_point_age_seconds: Some(30.0), // 30 seconds default
            max_distance_between_poses: 0.05,  // 5 centimeters
            max_angle_between_poses: std::f64::consts::PI / 18.0, // 10 degrees
        }
    }
}
