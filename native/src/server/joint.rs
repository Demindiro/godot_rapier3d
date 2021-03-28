use super::*;
use crate::vec_gd_to_na;
use core::mem;
use gdnative::core_types::*;
use gdnative::sys;
use rapier3d::dynamics::{self, JointHandle, JointParams, RevoluteJoint};
use rapier3d::math::{Point, Vector};
use rapier3d::na::Unit;

pub struct Joint {
	joint: Instance<JointHandle, LooseJoint>,
}

struct LooseJoint {
	params: JointParams,
	body_a: u32,
	body_b: u32,
}

impl Joint {
	fn new<T>(joint: T, body_a: u32, body_b: u32) -> Self
	where
		T: Into<JointParams> + Copy,
	{
		let params = joint.into();
		let result = Index::read_body(body_a, |body_a| {
			Index::read_body(body_b, |body_b| {
				if let Some((body_a, space_a)) = body_a.as_attached() {
					if let Some((body_b, space_b)) = body_b.as_attached() {
						if space_a == space_b {
							return Some((space_a.modify(|space| {
								space.joints.insert(&mut space.bodies, body_a, body_b, joint)
							}).expect("Invalid space"), space_a));
						} else {
							godot_error!("Bodies are in different spaces");
						}
					}
				}
				None
			})
		});
		if let Ok(Ok(Some((joint, space)))) = result {
			Self {
				joint: Instance::Attached(joint, space),
			}
		} else {
			Self {
				joint: Instance::Loose(LooseJoint {
					params,
					body_a,
					body_b,
				}),
			}
		}
	}
}

pub fn init(ffi: &mut ffi::FFI) {
	ffi.joint_create_hinge(create_hinge);
}

fn create_hinge(
	body_a: Index,
	transform_a: &Transform,
	body_b: Index,
	transform_b: &Transform,
) -> Option<Index> {

	let body_a = if let Index::Body(index) = body_a {
		index
	} else {
		godot_error!("ID A does not point to a body");
		return None;
	};
	let body_b = if let Index::Body(index) = body_b {
		index
	} else {
		godot_error!("ID B does not point to a body");
		return None;
	};

	let origin_a = transform_a.origin;
	let origin_b = transform_b.origin;
	let origin_a = Point::new(origin_a.x, origin_a.y, origin_a.z);
	let origin_b = Point::new(origin_b.x, origin_b.y, origin_b.z);

	let basis_a = transform_a.basis;
	let basis_b = transform_b.basis;
	let axis_a = basis_a.elements[0];
	let axis_b = basis_b.elements[0];
	let axis_a = Unit::new_normalize(vec_gd_to_na(axis_a));
	let axis_b = Unit::new_normalize(vec_gd_to_na(axis_b));
	let basis_a = [
		vec_gd_to_na(basis_a.elements[1]),
		vec_gd_to_na(basis_a.elements[2]),
	];
	let basis_b = [
		vec_gd_to_na(basis_b.elements[1]),
		vec_gd_to_na(basis_b.elements[2]),
	];

	let mut joint = RevoluteJoint::new(origin_a, axis_a, origin_b, axis_b);
	joint.basis1 = basis_a;
	joint.basis2 = basis_b;

	Some(Index::add_joint(Joint::new(joint, body_a, body_b)))
}
