use super::*;
use gdnative::core_types::*;
use gdnative::sys;
use rapier3d::dynamics::{ActivationStatus, RigidBody, RigidBodyBuilder, RigidBodyHandle};
use rapier3d::geometry::{ColliderBuilder, ColliderHandle};
use rapier3d::math::{Isometry, Rotation, Translation, Vector};
use rapier3d::na;
use std::mem;

#[derive(Debug)]
enum Type {
	Static,
	Kinematic,
	Rigid,
	Character,
}

#[derive(Debug)]
enum State {
	Transform(Transform),
	LinearVelocity(Vector3),
	AngularVelocity(Vector3),
	Sleeping(bool),
	CanSleep(bool),
}

#[derive(Debug)]
enum StateError {
	InvalidType,
	InvalidValue,
}

struct Shape {
	index: usize,
	transform: Isometry<f32>,
	enabled: bool,
}

pub struct Body {
	body: Instance<(RigidBodyHandle, Vec<Option<ColliderHandle>>), RigidBody>,
	object_id: Option<ObjectID>,
	shapes: Vec<Shape>,
}

impl Type {
	fn new(n: i32) -> Result<Type, ()> {
		Ok(match n {
			0 => Type::Static,
			1 => Type::Kinematic,
			2 => Type::Rigid,
			3 => Type::Character,
			_ => return Err(()),
		})
	}

	fn create_body(&self, sleep: bool) -> RigidBody {
		match self {
			Type::Static => RigidBodyBuilder::new_static(),
			Type::Kinematic => RigidBodyBuilder::new_kinematic(),
			Type::Rigid => RigidBodyBuilder::new_dynamic(),
			Type::Character => RigidBodyBuilder::new_dynamic(),
		}
		.sleeping(sleep)
		.build()
	}
}

impl State {
	fn new(r#type: i32, value: &Variant) -> Result<State, StateError> {
		Ok(match r#type {
			0 => State::Transform(value.try_to_transform().ok_or(StateError::InvalidValue)?),
			1 => State::LinearVelocity(value.try_to_vector3().ok_or(StateError::InvalidValue)?),
			2 => State::AngularVelocity(value.try_to_vector3().ok_or(StateError::InvalidValue)?),
			3 => State::Sleeping(value.try_to_bool().ok_or(StateError::InvalidValue)?),
			4 => State::CanSleep(value.try_to_bool().ok_or(StateError::InvalidValue)?),
			_ => return Err(StateError::InvalidType),
		})
	}
}

impl Body {
	fn new(r#type: Type, sleep: bool) -> Self {
		Self {
			body: Instance::Loose(r#type.create_body(sleep)),
			object_id: None,
			shapes: Vec::new(),
		}
	}

	fn map_shape_mut<F>(&mut self, index: usize, f: F)
	where
		F: FnOnce(&mut Shape),
	{
		if let Some(shape) = self.shapes.get_mut(index) {
			f(shape);
		} else {
			eprintln!("Invalid shape index");
		}
	}

	pub fn as_attached(&self) -> Option<(RigidBodyHandle, SpaceHandle)> {
		if let Instance::Attached(body, space) = &self.body {
			Some((body.0, *space))
		} else {
			None
		}
	}
}

pub fn init(ffi: &mut ffi::FFI) {
	ffi.body_create(create);
	ffi.body_attach_object_instance_id(attach_object_instance_id);
	ffi.body_get_direct_state(get_direct_state);
	ffi.body_add_shape(add_shape);
	ffi.body_set_shape_transform(set_shape_transform);
	ffi.body_set_shape_disabled(set_shape_disabled);
	ffi.body_set_space(set_space);
	ffi.body_set_state(set_state);
	ffi.body_set_ray_pickable(set_ray_pickable);
	ffi.body_remove_shape(remove_shape);
}

fn create(r#type: i32, sleep: bool) -> *const Index {
	if let Ok(r#type) = Type::new(r#type) {
		Index::add_body(Body::new(r#type, sleep)).raw()
	} else {
		eprintln!("Invalid body type");
		core::ptr::null()
	}
}

fn add_shape(
	body: Index,
	shape: Index,
	transform: &Transform,
	disabled: bool,
) {
	map_or_err!(body, map_body_mut, |v| {
		if let Index::Shape(index) = shape {
			v.shapes.push(Shape {
				index,
				transform: transform_to_isometry(*transform),
				enabled: !disabled,
			})
		} else {
			godot_error!("ID does not point to a shape");
		}
	});
}

fn attach_object_instance_id(body: Index, id: u32) {
	map_or_err!(body, map_body_mut, |v| v.object_id =
		ObjectID::new(id));
}

fn get_direct_state(body: Index, state: &mut ffi::PhysicsBodyState) {
	map_or_err!(body, map_body, |body| {
		match &body.body {
			Instance::Attached((body, _), space) => {
				let transform = crate::get_transform(*space, *body).expect("Invalid body or space");
				// FIXME make this safe
				unsafe {
					state.transform = *transform.sys();
				}
			}
			Instance::Loose(_) => {}
		}
	});
}

fn remove_shape(body: Index, shape: i32) {
	let shape = shape as usize;
	map_or_err!(body, map_body_mut, |body| {
		// TODO this can panic
		body.shapes.remove(shape);
		if let Instance::Attached((_, colliders), space) = &mut body.body {
			crate::modify_space(*space, |space| {
				// TODO can panic too
				colliders
					.remove(shape)
					.map(|c| space.colliders.remove(c, &mut space.bodies, true));
			})
			.expect("Failed to modify space");
		}
	});
}

fn set_shape_transform(
	body: Index,
	shape: i32,
	transform: &Transform,
) {
	let shape = shape as usize;
	map_or_err!(body, map_body_mut, |v| v
		.map_shape_mut(shape, |v| v.transform = transform_to_isometry(*transform)));
}

fn set_shape_disabled(body: Index, shape: i32, disable: bool) {
	let shape = shape as usize;
	map_or_err!(body, map_body_mut, |v| v
		.map_shape_mut(shape, |v| v.enabled = !disable));
}

fn set_space(body: Index, space: Option<Index>) {
	if let Some(space) = space {
		if let Ok(space) = space.map_space(|&v| v) {
			// We need to get the shapes now, as the index array will be write locked later
			let colliders = body
				.map_body(|body| {
					let mut colliders = Vec::with_capacity(body.shapes.len());
					for shape in &body.shapes {
						if shape.enabled {
							let transform = shape.transform;
							let result = Index::read_shape(shape.index, |shape| {
								let collider = Some(
									ColliderBuilder::new(shape.shape().clone())
										.position(transform)
										.build(),
								);
								colliders.push(collider);
							});
							if let Err(_) = result {
								godot_error!("Shape is invalid: {}", shape.index);
								colliders.push(None); // Make sure the collider count does still match
							}
						} else {
							colliders.push(None);
						}
					}
					colliders
				})
				.expect("Body is invalid");
			map_or_err!(body, map_body_mut, |body| {
				let b = Instance::Attached((RigidBodyHandle::invalid(), Vec::new()), space);
				let b = mem::replace(&mut body.body, b);
				body.body = match b {
					Instance::Attached(_, _) => todo!(),
					Instance::Loose(b) => {
						let mut handle = None;
						let mut collider_handles = Vec::with_capacity(colliders.len());
						crate::modify_space(space, |space| {
							handle = Some(space.bodies.insert(b));
							for collider in colliders {
								let handle = collider.map(|c| {
									space
										.colliders
										.insert(c, handle.unwrap(), &mut space.bodies)
								});
								collider_handles.push(handle);
							}
						})
						.expect("Invalid space");
						Instance::Attached((handle.unwrap(), collider_handles), space)
					}
				};
			});
		}
	} else {
		map_or_err!(body, map_body_mut, |body| match body.body {
			Instance::Attached((body, _), space) => {
				crate::modify_space(space, |space| {
					space
						.bodies
						.remove(body, &mut space.colliders, &mut space.joints);
				})
				.expect("Failed to modify space");
			}
			Instance::Loose(_) => todo!(),
		});
	}
}

fn set_ray_pickable(body: Index, enable: bool) {
	// TODO IIRC there is no equivalent in Rapier3D, unless I misunderstand the purpose of this function
	let _ = (body, enable);
}

fn set_state(body: Index, state: i32, value: &Variant) {
	let apply = |body: &mut RigidBody, state| match state {
		State::Transform(trf) => body.set_position(transform_to_isometry(trf), true),
		State::LinearVelocity(vel) => body.set_linvel(vec_gd_to_na(vel), true),
		State::AngularVelocity(vel) => body.set_angvel(vec_gd_to_na(vel), true),
		State::Sleeping(sleep) => body.activation.sleeping = sleep,
		State::CanSleep(sleep) => {
			body.activation.threshold = if sleep {
				ActivationStatus::default_threshold()
			} else {
				-1.0
			}
		}
	};

	map_or_err!(body, map_body_mut, |body| {
		match State::new(state, value) {
			Ok(state) => match &mut body.body {
				Instance::Attached(body, space) => {
					modify_rigid_body(*space, body.0, |body| apply(body, state))
						.expect("Invalid body or space")
				}
				Instance::Loose(body) => apply(body, state),
			},
			Err(e) => eprintln!("Invalid state: {:?}", e),
		}
	});
}
