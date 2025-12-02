use nalgebra as na;
use std::collections::HashMap;

use crate::{
    point3d::{self},
    voxel_util::{self, na_vec_to_voxel},
};

pub type Voxel = na::Vector3<i32>;
type VoxelPoints = Vec<point3d::Point3d>;

pub struct VoxelHashMap {
    pub voxel_size: f32,
    pub max_distance: f64,
    pub max_points_per_voxel: usize,
    pub map: HashMap<na::Vector3<i32>, VoxelPoints>,
    pub last_batch_points: VoxelPoints,
    pub max_point_age_seconds: Option<f64>,
}

fn get_adjacent_voxels(voxel: &Voxel, adjacent_voxels: i32) -> Vec<Voxel> {
    let mut voxel_neighborhood = Vec::<Voxel>::new();
    for x in voxel.x - adjacent_voxels..voxel.x + adjacent_voxels + 1 {
        for y in voxel.y - adjacent_voxels..voxel.y + adjacent_voxels + 1 {
            for z in voxel.z - adjacent_voxels..voxel.z + adjacent_voxels + 1 {
                voxel_neighborhood.push(Voxel::new(x, y, z));
            }
        }
    }
    voxel_neighborhood
}

impl VoxelHashMap {
    pub fn default_values() -> VoxelHashMap {
        VoxelHashMap {
            voxel_size: 1.0,
            max_distance: 100.0,
            max_points_per_voxel: 20,
            map: HashMap::new(),
            last_batch_points: Vec::new(),
            max_point_age_seconds: Some(30.0),
        }
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.map.is_empty()
    }

    pub fn map_len(&self) -> usize {
        self.map.iter().fold(0, |acc, (_, v)| acc + v.len())
    }

    fn update(&mut self, points: &VoxelPoints, current_origin: &na::Vector3<f64>) {
        self.add_points(points);
        self.remove_points_too_far(current_origin);
        if self.max_point_age_seconds.is_some() {
            self.remove_aged_points();
        }
    }

    pub fn get_na_points(&self) -> Vec<na::Vector3<f64>> {
        self.map
            .values()
            .map(|v| {
                v.iter()
                    .map(|p| p.to_na_vec_f64())
                    .collect::<Vec<na::Vector3<f64>>>()
            })
            .reduce(|mut acc, mut e| {
                acc.append(&mut e);
                acc
            })
            .unwrap()
    }

    pub fn update_with_pose(
        &mut self,
        points: &[point3d::Point3d],
        t_origin_current: &na::Isometry3<f64>,
    ) {
        let transformed_points: VoxelPoints = points
            .iter()
            .map(|pt| {
                let transformed_na = t_origin_current * pt.to_na_point3_f64();
                point3d::Point3d {
                    x: transformed_na.x as f32,
                    y: transformed_na.y as f32,
                    z: transformed_na.z as f32,
                    intensity: pt.intensity,
                    timestamp: pt.timestamp, // Preserve original timestamp
                }
            })
            .collect();
        self.update(&transformed_points, &t_origin_current.translation.vector);
    }

    fn add_points(&mut self, points: &VoxelPoints) {
        let mut last_batch = Vec::new();
        let map_resolution =
            (self.voxel_size * self.voxel_size / self.max_points_per_voxel as f32).sqrt() as f64;
        points.iter().for_each(|pt| {
            let voxel = na_vec_to_voxel(&pt.to_na_vec_f64(), self.voxel_size as f64);
            if let Some(voxel_points) = self.map.get_mut(&voxel) {
                if voxel_points.len() >= self.max_points_per_voxel
                    || voxel_points.iter().any(|vpt| {
                        (vpt.to_na_vec_f64() - pt.to_na_vec_f64()).norm() < map_resolution
                    })
                {
                    // return;
                } else {
                    voxel_points.push(*pt);
                    last_batch.push(*pt);
                }
            } else {
                self.map.insert(voxel, vec![*pt]);
                last_batch.push(*pt);
            }
        });
        self.last_batch_points = last_batch;
    }
    fn remove_points_too_far(&mut self, current_origin: &na::Vector3<f64>) {
        let max_distance2 = self.max_distance * self.max_distance;
        let keys_too_far: Vec<Voxel> = self
            .map
            .iter()
            .filter_map(|(k, vps)| {
                if (vps[0].to_na_vec_f64() - current_origin).norm_squared() >= max_distance2 {
                    Some(k.to_owned())
                } else {
                    None
                }
            })
            .collect();
        keys_too_far.iter().for_each(|k| {
            self.map.remove(k);
        });
    }

    fn remove_aged_points(&mut self) {
        if let Some(max_age) = self.max_point_age_seconds {
            let mut empty_voxels = Vec::new();

            for (voxel_key, voxel_points) in self.map.iter_mut() {
                voxel_points.retain(|point| point.age_seconds() <= max_age);

                if voxel_points.is_empty() {
                    empty_voxels.push(*voxel_key);
                }
            }

            for voxel_key in empty_voxels {
                self.map.remove(&voxel_key);
            }
        }
    }

    pub fn get_point_count_by_age(&self, max_age_seconds: f64) -> usize {
        self.map
            .values()
            .flat_map(|points| points.iter())
            .filter(|point| point.age_seconds() <= max_age_seconds)
            .count()
    }

    pub fn get_oldest_point_age(&self) -> Option<f64> {
        self.map
            .values()
            .flat_map(|points| points.iter())
            .map(|point| point.age_seconds())
            .max_by(|a, b| a.partial_cmp(b).unwrap())
    }

    pub fn get_closest_neighbor(
        &self,
        point: &point3d::Point3d,
    ) -> Option<(point3d::Point3d, f64)> {
        let voxel = voxel_util::point_to_voxel(point, self.voxel_size);
        let query_voxels = get_adjacent_voxels(&voxel, 1);
        let point_na = point.to_na_vec_f64();
        let neighbors: Vec<(point3d::Point3d, f64)> = query_voxels
            .iter()
            .filter_map(|query_voxel| {
                if let Some(voxel_points) = self.map.get(query_voxel) {
                    let neighbor = voxel_points
                        .iter()
                        .reduce(|acc, pt| {
                            if (acc.to_na_vec_f64() - point_na).norm()
                                < (pt.to_na_vec_f64() - point_na).norm()
                            {
                                acc
                            } else {
                                pt
                            }
                        })
                        .unwrap();
                    let distance = (neighbor.to_na_vec_f64() - point_na).norm();
                    Some((*neighbor, distance))
                } else {
                    None
                }
            })
            .collect();
        neighbors
            .iter()
            .reduce(
                |acc, neighbor| {
                    if acc.1 < neighbor.1 {
                        acc
                    } else {
                        neighbor
                    }
                },
            )
            .copied()
    }
}
