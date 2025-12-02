use crate::point3d::Point3d;

pub fn deskew_scan(
    cloud: &mut [Point3d],
    timestamped_poses: &Vec<(nalgebra::Isometry3<f64>, f64)>,
) {
    if timestamped_poses.len() < 2 {
        return;
    }

    let reference_pose = &timestamped_poses[0].0;

    for point in cloud.iter_mut() {
        let mut before_pose = &timestamped_poses[0].0;
        let mut after_pose = &timestamped_poses[0].0;
        let mut before_time = timestamped_poses[0].1;
        let mut after_time = timestamped_poses[0].1;

        for i in 0..timestamped_poses.len() - 1 {
            if point.timestamp >= timestamped_poses[i].1
                && point.timestamp <= timestamped_poses[i + 1].1
            {
                before_pose = &timestamped_poses[i].0;
                after_pose = &timestamped_poses[i + 1].0;
                before_time = timestamped_poses[i].1;
                after_time = timestamped_poses[i + 1].1;
                break;
            }
        }

        let alpha = (point.timestamp - before_time) / (after_time - before_time);
        let interpolated_translation =
            before_pose.translation.vector * (1.0 - alpha) + after_pose.translation.vector * alpha;
        let interpolated_rotation = before_pose.rotation.slerp(&after_pose.rotation, alpha);
        let sensor_pose_at_point_time = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::from(interpolated_translation),
            interpolated_rotation,
        );

        // point_in_reference = reference_pose^-1 * sensor_pose_at_time * point
        let original_point = nalgebra::Point3::new(point.x as f64, point.y as f64, point.z as f64);
        let transform = reference_pose.inverse() * sensor_pose_at_point_time;
        let deskewed_point = transform * original_point;

        point.x = deskewed_point.x as f32;
        point.y = deskewed_point.y as f32;
        point.z = deskewed_point.z as f32;
    }
}
