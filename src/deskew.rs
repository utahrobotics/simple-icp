use std::time::Instant;

use nalgebra::Isometry3;

use crate::point3d::Point3d;

use nalgebra::{Translation3, UnitQuaternion};

/// Deskews a point cloud by interpolating robot poses for every point
/// and projecting them to the frame of the scan's end time.
///
/// # Arguments
/// * `points` - The mutable slice of points to deskew.
/// * `poses` - A history of robot poses (Global Frame) sorted by time.
pub fn deskew_scan(
    points: &mut [Point3d],
    poses: &[(Instant, Isometry3<f64>)],
    max_angle_between_poses: f64,
    max_distance_between_poses: f64,
) {
    if points.is_empty()
        || poses.is_empty()
        || !validate_poses(poses, max_distance_between_poses, max_angle_between_poses)
    {
        eprintln!("Deskewing skipped due to insufficient data or invalid poses.");
        return;
    }
    let scan_end_time = points
        .iter()
        .map(|p| p.global_timestamp)
        .max()
        .unwrap_or(points.last().unwrap().global_timestamp);

    let pose_at_scan_end = match interpolate_pose_at_time(poses, scan_end_time) {
        Some(pose) => pose,
        None => return,
    };
    let pose_at_scan_end_inv = pose_at_scan_end.inverse();

    for point in points.iter_mut() {
        if let Some(pose_at_point_time) = interpolate_pose_at_time(poses, point.global_timestamp) {
            // P_world = Pose(t) * P_raw
            // P_corrected = Pose(end)^-1 * P_world
            // (Pose(end)^-1 * Pose(t)) * P_raw

            let correction_transform = pose_at_scan_end_inv * pose_at_point_time;

            let original_point = point.to_na_point3_f64();
            let corrected_point = correction_transform * original_point;

            point.x = corrected_point.x as f32;
            point.y = corrected_point.y as f32;
            point.z = corrected_point.z as f32;
        }
    }
}

fn validate_poses(
    poses: &[(Instant, Isometry3<f64>)],
    max_distance_between_poses: f64,
    max_angle_between_poses: f64,
) -> bool {
    for window in poses.windows(2) {
        let pose1 = &window[0];
        let pose2 = &window[1];
        if pose2.0 <= pose1.0 {
            return false;
        }
        // if poses are too far apart that means a loop may have been closed or apriltag seen, or something to cause a discontinuity
        if pose1.1.rotation.angle_to(&pose2.1.rotation) > max_angle_between_poses {
            return false;
        }
        if pose1
            .1
            .translation
            .vector
            .metric_distance(&pose2.1.translation.vector)
            > max_distance_between_poses
        {
            return false;
        }
    }
    true
}

fn interpolate_pose_at_time(
    poses: &[(Instant, Isometry3<f64>)],
    time: Instant,
) -> Option<Isometry3<f64>> {
    if let Some(first) = poses.first() {
        if time <= first.0 {
            return Some(first.1);
        }
    }
    if let Some(last) = poses.last() {
        if time >= last.0 {
            return Some(last.1);
        }
    }

    let idx = poses.partition_point(|(t, _)| *t <= time);

    if idx == 0 || idx >= poses.len() {
        //just in case
        return None;
    }

    let (t_prev, pose_prev) = poses[idx - 1];
    let (t_next, pose_next) = poses[idx];

    let total_duration = t_next.duration_since(t_prev).as_secs_f64();
    if total_duration <= f64::EPSILON {
        return Some(pose_prev);
    }

    let current_duration = time.duration_since(t_prev).as_secs_f64();
    let alpha = current_duration / total_duration;

    let translation = Translation3::from(
        pose_prev
            .translation
            .vector
            .lerp(&pose_next.translation.vector, alpha),
    );

    let rotation = UnitQuaternion::slerp(&pose_prev.rotation, &pose_next.rotation, alpha);

    Some(Isometry3::from_parts(translation, rotation))
}
