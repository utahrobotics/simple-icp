use nalgebra as na;

use std::time::{Instant, SystemTime, UNIX_EPOCH};

#[derive(Debug, Clone, Copy)]
pub struct Point3d {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
    /// Unix timestamp in seconds
    pub global_timestamp: Instant,
}

impl Point3d {
    pub fn new(x: f32, y: f32, z: f32, intensity: f32) -> Self {
        Point3d {
            x,
            y,
            z,
            intensity,
            global_timestamp: Instant::now(),
        }
    }

    pub fn new_with_timestamp(x: f32, y: f32, z: f32, intensity: f32, timestamp: Instant) -> Self {
        Point3d {
            x,
            y,
            z,
            intensity,
            global_timestamp: timestamp,
        }
    }

    pub fn age_seconds(&self) -> f64 {
        self.global_timestamp.elapsed().as_secs_f64()
    }
}

impl Point3d {
    pub fn square(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }
    pub fn to_na_vec_f64(&self) -> na::Vector3<f64> {
        na::Vector3::<f64>::new(self.x as f64, self.y as f64, self.z as f64)
    }
    pub fn to_na_point3_f64(&self) -> na::Point3<f64> {
        na::Point3::<f64>::new(self.x as f64, self.y as f64, self.z as f64)
    }
}

pub fn clip_point_cloud_by_distance(
    point_cloud: &[Point3d],
    min_distance: f32,
    max_distance: f32,
) -> Vec<Point3d> {
    let min2 = min_distance * min_distance;
    let max2 = max_distance * max_distance;
    point_cloud
        .iter()
        .filter_map(|pt| {
            let s = pt.square();
            if s < min2 || s > max2 {
                None
            } else {
                Some(*pt)
            }
        })
        .collect()
}

pub fn clip_point_cloud_by_distance_and_intensity(
    point_cloud: &[Point3d],
    min_distance: f32,
    max_distance: f32,
    min_intensity: f32,
) -> Vec<Point3d> {
    let min2 = min_distance * min_distance;
    let max2 = max_distance * max_distance;
    point_cloud
        .iter()
        .filter_map(|pt| {
            let s = pt.square();
            if s < min2 || s > max2 || pt.intensity < min_intensity {
                None
            } else {
                Some(*pt)
            }
        })
        .collect()
}
