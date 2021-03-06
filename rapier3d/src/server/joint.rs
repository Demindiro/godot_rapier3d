use super::*;
use crate::body;
use crate::util::*;
use core::convert::TryFrom;
use gdnative::core_types::*;
use gdnative::godot_error;
use rapier3d::dynamics::{JointHandle, JointParams, RevoluteJoint, SpringModel};
use rapier3d::math::Point;
use rapier3d::na::Unit;

pub struct Joint {
	joint: Instance<JointHandle, LooseJoint>,
	exclude_bodies: bool,
}

struct LooseJoint {
	params: JointParams,
	#[allow(dead_code)]
	body_a: BodyIndex,
	#[allow(dead_code)]
	body_b: BodyIndex,
}

#[derive(Debug)]
enum HingeParam {
	Bias(f32),
	LimitUpper(f32),
	LimitLower(f32),
	LimitBias(f32),
	LimitSoftness(f32),
	LimitRelaxation(f32),
	MotorTargetVelocity(f32),
	MotorMaxImpulse(f32),
}

enum HingeFlag {
	UseLimit(bool),
	EnableMotor(bool),
}

#[derive(Debug)]
enum ParamError {
	InvalidParam(i32),
}

impl Joint {
	fn new<T>(joint: T, body_a: BodyIndex, body_b: BodyIndex) -> Self
	where
		T: Into<JointParams> + Copy,
	{
		let params = joint.into();
		let bodies = BodyIndex::read_all();
		let result = bodies.get(body_a.into()).and_then(|body_a| {
			bodies.get(body_b.into()).and_then(|body_b| {
				if let Some((body_a, space_a)) = body_a.as_attached() {
					if let Some((body_b, space_b)) = body_b.as_attached() {
						if space_a == space_b {
							return Some((
								space_a
									.map_mut(|space| space.add_joint(params, body_a, body_b))
									.expect("Invalid space"),
								space_a,
							));
						} else {
							godot_error!("Bodies are in different spaces");
						}
					}
				}
				None
			})
		});
		if let Some((joint, space)) = result {
			Self {
				joint: Instance::Attached(joint, space),
				exclude_bodies: false,
			}
		} else {
			Self {
				joint: Instance::loose(LooseJoint {
					params,
					body_a,
					body_b,
				}),
				exclude_bodies: false,
			}
		}
	}

	/// Frees this hinge, removing it from it's attached bodies (if any)
	fn free(mut self) {
		match &mut self.joint {
			Instance::Attached(jh, space) => {
				space
					.map_mut(|space| space.remove_joint(*jh))
					.expect("Invalid space");
			}
			Instance::Loose(_) => {}
		}
	}
}

impl HingeParam {
	fn new(n: i32, value: f32) -> Result<HingeParam, ParamError> {
		Ok(match n {
			0 => Self::Bias(value),
			1 => Self::LimitUpper(value),
			2 => Self::LimitLower(value),
			3 => Self::LimitBias(value),
			4 => Self::LimitSoftness(value),
			5 => Self::LimitRelaxation(value),
			6 => Self::MotorTargetVelocity(value),
			7 => Self::MotorMaxImpulse(value),
			_ => return Err(ParamError::InvalidParam(n)),
		})
	}
}

impl HingeFlag {
	fn new(n: i32, value: bool) -> Result<HingeFlag, ParamError> {
		Ok(match n {
			0 => Self::UseLimit(value),
			1 => Self::EnableMotor(value),
			_ => return Err(ParamError::InvalidParam(n)),
		})
	}
}

pub fn init(ffi: &mut ffi::FFI) {
	ffi!(ffi, joint_create_hinge, create_hinge);
	ffi!(
		ffi,
		joint_disable_collisions_between_bodies,
		disable_collisions_between_bodies
	);
	ffi!(ffi, joint_get_solver_priority, |_| 0);
	ffi!(ffi, joint_set_solver_priority, |_, _| {});
	ffi!(ffi, hinge_joint_set_flag, set_hinge_flag);
	ffi!(ffi, hinge_joint_set_param, set_hinge_param);
}

/// Frees the given hinge, removing it from it's attached bodies (if any)
pub fn free(joint: Joint) {
	joint.free();
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

	let basis_a = transform_a.basis.transposed();
	let basis_b = transform_b.basis.transposed();
	let axis_a = basis_a.elements[2];
	let axis_b = basis_b.elements[2];
	let axis_a = Unit::new_normalize(vec_gd_to_na(axis_a));
	let axis_b = Unit::new_normalize(vec_gd_to_na(axis_b));
	let basis_a = [
		vec_gd_to_na(basis_a.elements[0]),
		vec_gd_to_na(basis_a.elements[1]),
	];
	let basis_b = [
		vec_gd_to_na(basis_b.elements[0]),
		vec_gd_to_na(basis_b.elements[1]),
	];

	let mut joint = RevoluteJoint::new(origin_a, axis_a, origin_b, axis_b);
	joint.basis1 = basis_a;
	joint.basis2 = basis_b;

	Some(Index::Joint(JointIndex::add(Joint::new(
		joint, body_a, body_b,
	))))
}

fn disable_collisions_between_bodies(joint: Index, disable: bool) {
	// *disable* collisions = *enable* exclusion
	let enable = disable;
	map_or_err!(joint, map_joint_mut, |joint, _| {
		if joint.exclude_bodies != enable {
			joint.exclude_bodies = enable;
			if let Instance::Attached(joint, space) = joint.joint {
				map_or_err!(space, map_mut, |space| {
					let joint = space.joints().get(joint).expect("Invalid joint");
					// FIXME this can (and will) panic if a body is freed. Check again later
					// for the cleanest way to handle this (should the joint be freed?)
					let a = space.bodies().get(joint.body1).expect("Invalid body A");
					let b = space.bodies().get(joint.body2).expect("Invalid body B");
					let a = body::RigidBodyUserdata::try_from(a).unwrap().index();
					let b = body::RigidBodyUserdata::try_from(b).unwrap().index();
					// Adding an exclusion multiple times is valid.
					if enable {
						let _ = space.add_body_exclusion(a, b);
					} else {
						let _ = space.remove_body_exclusion(a, b);
					}
				});
			}
		}
	});
}

fn set_hinge_flag(joint: Index, flag: i32, value: bool) {
	let flag = match HingeFlag::new(flag, value) {
		Ok(f) => f,
		Err(e) => {
			godot_error!("Failed to apply hinge joint parameter: {:?}", e);
			return;
		}
	};
	let apply = |joint: &mut JointParams| {
		if let JointParams::RevoluteJoint(j) = joint {
			match flag {
				HingeFlag::UseLimit(_) => godot_error!("TODO"),
				HingeFlag::EnableMotor(v) => {
					j.motor_model = if v {
						SpringModel::default()
					} else {
						SpringModel::Disabled
					}
				}
			}
		} else {
			godot_error!("Joint is not a hinge joint");
		}
	};
	map_or_err!(joint, map_joint_mut, |joint, _| {
		match &mut joint.joint {
			Instance::Attached(jh, space) => {
				space
					.map_mut(|space| {
						apply(
							&mut space
								.joints_mut()
								.get_mut(*jh)
								.expect("Invalid joint handle")
								.params,
						);
					})
					.expect("Invalid space");
			}
			Instance::Loose(j) => apply(&mut j.params),
		}
	});
}

fn set_hinge_param(joint: Index, param: i32, value: f32) {
	let param = match HingeParam::new(param, value) {
		Ok(p) => p,
		Err(e) => {
			godot_error!("Failed to apply hinge joint parameter: {:?}", e);
			return;
		}
	};
	let apply = |joint: &mut JointParams| {
		if let JointParams::RevoluteJoint(j) = joint {
			let e = || godot_error!("TODO");
			match param {
				// FIXME it seems this parameter doesn't exist in nphysics3d either. How should
				// we handle it?
				HingeParam::Bias(_) => e(),
				HingeParam::LimitBias(_) => e(),
				// FIXME it seems there are no configurable limits on hinge joints?
				// Judging by the nphysics3d documentation and the fact Rapier is a successor,
				// it will hopefully be implemented soon.
				HingeParam::LimitUpper(_) => e(),
				HingeParam::LimitLower(_) => e(),
				HingeParam::LimitSoftness(_) => e(),
				HingeParam::LimitRelaxation(_) => e(),
				HingeParam::MotorTargetVelocity(v) => j.configure_motor_velocity(-v, 1.0),
				HingeParam::MotorMaxImpulse(v) => j.motor_max_impulse = v,
			}
		} else {
			godot_error!("Joint is not a hinge joint");
		}
	};
	map_or_err!(joint, map_joint_mut, |joint, _| {
		match &mut joint.joint {
			Instance::Attached(jh, space) => {
				space
					.map_mut(|space| {
						apply(
							&mut space
								.joints_mut()
								.get_mut(*jh)
								.expect("Invalid joint handle")
								.params,
						);
					})
					.expect("Invalid space");
			}
			Instance::Loose(j) => apply(&mut j.params),
		}
	});
}
