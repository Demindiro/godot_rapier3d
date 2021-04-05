use gdnative::core_types::*;
use rapier3d::math::*;
use rapier3d::na;

pub fn vec_gd_to_na(from: Vector3) -> na::Vector3<f32> {
	na::Vector3::new(from.x, from.y, from.z)
}

pub fn vec_na_to_gd(from: na::Vector3<f32>) -> Vector3 {
	Vector3::new(from.x, from.y, from.z)
}

pub fn transform_to_isometry_and_scale(transform: &Transform) -> (Isometry<f32>, Vector3) {
	let origin = Translation::new(transform.origin.x, transform.origin.y, transform.origin.z);
	let rotation = transform.basis.to_quat();
	let scale = transform.basis.to_scale();
	let rotation = na::Unit::from_quaternion(na::Quaternion::new(
		rotation.r, rotation.i, rotation.j, rotation.k,
	));
	(Isometry::from_parts(origin, rotation), scale)
}

pub fn transform_to_isometry(transform: Transform) -> Isometry<f32> {
	transform_to_isometry_and_scale(&transform).0
}

pub fn isometry_to_transform(isometry: &Isometry<f32>) -> Transform {
	let origin = Vector3::new(
		isometry.translation.x,
		isometry.translation.y,
		isometry.translation.z,
	);
	let rot = isometry.rotation.to_rotation_matrix();
	let basis = mat3_to_basis(&rot.matrix());
	Transform { basis, origin }
}

pub fn mat3_to_basis(mat: &na::Matrix3<f32>) -> Basis {
	Basis {
		elements: [
			Vector3::new(mat[(0, 0)], mat[(0, 1)], mat[(0, 2)]),
			Vector3::new(mat[(1, 0)], mat[(1, 1)], mat[(1, 2)]),
			Vector3::new(mat[(2, 0)], mat[(2, 1)], mat[(2, 2)]),
		],
	}
}
