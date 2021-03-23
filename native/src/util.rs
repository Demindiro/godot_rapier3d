use gdnative::prelude::Vector3;
use rapier3d::na;

pub fn vec_gd_to_na(from: Vector3) -> na::Vector3<f32> {
    na::Vector3::new(from.x, from.y, from.z)
}

pub fn vec_na_to_gd(from: na::Vector3<f32>) -> Vector3 {
    Vector3::new(from.x, from.y, from.z)
}
