use gdnative::core_types::*;
use rapier3d::math::*;
use rapier3d::na;

pub fn vec_gd_to_na(from: Vector3) -> na::Vector3<f32> {
	na::Vector3::new(from.x, from.y, from.z)
}

pub fn vec_na_to_gd(from: na::Vector3<f32>) -> Vector3 {
	Vector3::new(from.x, from.y, from.z)
}

pub fn transform_to_isometry(transform: Transform) -> Isometry<f32> {
	let origin = Translation::new(transform.origin.x, transform.origin.y, transform.origin.z);
	let rotation = transform.basis.to_quat();
	let rotation = na::Unit::from_quaternion(na::Quaternion::new(
		rotation.r, rotation.i, rotation.j, rotation.k,
	));
	Isometry::from_parts(origin, rotation)
}
