use crate::util::*;
use gdnative::api::World;
use gdnative::prelude::*;
use rapier3d::dynamics::{self, RigidBodyBuilder, RigidBodyHandle};
use rapier3d::na::Isometry3;
use rapier3d::na::Quaternion;
use rapier3d::na::UnitQuaternion;

#[derive(NativeClass)]
#[inherit(Spatial)]
pub struct RigidBody {
	body: Option<(RigidBodyHandle, crate::SpaceHandle)>,
}

#[methods]
impl RigidBody {
	fn new(_owner: TRef<Spatial>) -> Self {
		Self { body: None }
	}

	#[export]
	fn _enter_tree(&mut self, owner: TRef<Spatial>) {
		assert!(self.body.is_none(), "Body handle is not None");
		let world = owner.get_world().expect("Failed to get World");
		let transform = owner.global_transform();
		let quat = transform.basis.to_quat();
		let quat = Quaternion::new(quat.r, quat.i, quat.j, quat.k);
		let quat = UnitQuaternion::from_quaternion(quat);
		let origin = vec_gd_to_na(transform.origin);
		let isometry = Isometry3::from_parts(origin.into(), quat);
		let body = RigidBodyBuilder::new_dynamic().position(isometry).build();
		self.body = Some(crate::add_rigid_body(world, body));
	}

	#[export]
	fn _exit_tree(&mut self, owner: TRef<Spatial>) {
		let body = self.body.expect("Body handle is None");
		let world = owner.get_world().expect("Failed to get World");
		crate::remove_rigid_body(body.1, body.0).expect("Failed to remove body");
	}

	#[export]
	fn _physics_process(&self, owner: TRef<Spatial>, _delta: f32) {
		let body = self.body.expect("Body handle is None");
		let world = owner.get_world().expect("Failed to get World");
		let transform = crate::get_transform(body.1, body.0).expect("Failed to get transform");
		owner.set_global_transform(transform);
	}
}

impl super::Body for RigidBody {
	fn handle(&self) -> Option<(RigidBodyHandle, crate::SpaceHandle)> {
		self.body
	}
}
