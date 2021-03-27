use crate::body::*;
use crate::util::*;
use gdnative::api::Spatial;
use gdnative::prelude::*;
use rapier3d::dynamics::RigidBodyHandle;
use rapier3d::geometry::{ColliderBuilder, ColliderHandle};

#[derive(NativeClass)]
#[inherit(Spatial)]
pub struct Box {
	#[property]
	extents: Vector3,
	collider: Option<(ColliderHandle, crate::SpaceHandle)>,
}

#[methods]
impl Box {
	fn new(_owner: TRef<Spatial>) -> Self {
		Self {
			extents: Vector3::new(1.0, 1.0, 1.0),
			collider: None,
		}
	}

	#[export]
	fn _enter_tree(&mut self, owner: TRef<Spatial>) {
		assert!(self.collider.is_none(), "Body handle is not None");
		if let Some(parent) = owner.get_parent() {
			unsafe {
				if let Some(parent) = parent.assume_safe().cast::<Spatial>() {
					let handle = if let Some(parent) = parent.cast_instance::<RigidBody>() {
						parent.map(|s, _| s.handle())
					} else if let Some(parent) = parent.cast_instance::<StaticBody>() {
						parent.map(|s, _| s.handle())
					} else {
						godot_warn!("Parent is not a Body");
						return;
					}
					.unwrap()
					.expect("Parent has no handle");
					let collider =
						ColliderBuilder::cuboid(self.extents.x, self.extents.y, self.extents.z)
							.build();
					self.collider = Some((
						crate::add_collider(handle.1, handle.0, collider)
							.expect("Failed to add collider"),
						handle.1,
					));
				} else {
					godot_warn!("Parent is not a Spatial");
				}
			}
		} else {
			godot_warn!("Colliders need to be attached to a Body")
		}
	}

	#[export]
	fn _exit_tree(&mut self, _owner: TRef<Spatial>) {
		let collider = self.collider.expect("Collider & body handle is None");
		crate::remove_collider(collider.1, collider.0).expect("Failed to remove collider");
	}
}
