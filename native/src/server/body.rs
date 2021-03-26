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

	fn create_body(&self) -> RigidBody {
		match self {
			Type::Static => RigidBodyBuilder::new_static(),
			Type::Kinematic => RigidBodyBuilder::new_kinematic(),
			Type::Rigid => RigidBodyBuilder::new_dynamic(),
			Type::Character => RigidBodyBuilder::new_dynamic(),
		}
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
	fn new(r#type: Type) -> Self {
		Self {
			body: Instance::Loose(r#type.create_body()),
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

unsafe extern "C" fn create(r#type: i32, sleep: bool) -> ffi::Index {
	if let Ok(r#type) = Type::new(r#type) {
		Index::add_body(Body::new(r#type)).raw()
	} else {
		eprintln!("Invalid body type");
		core::ptr::null()
	}
}

unsafe extern "C" fn add_shape(
	body: ffi::Index,
	shape: ffi::Index,
	transform: *const ffi::godot_transform,
	disabled: bool,
) {
	Index::copy_raw(body).map_body_mut(|v| {
		if let Index::Shape(index) = Index::copy_raw(shape) {
			v.shapes.push(Shape {
				index,
				transform: conv_transform(*transform),
				enabled: !disabled,
			})
		} else {
			godot_error!("ID does not point to a shape");
		}
	});
}

unsafe extern "C" fn attach_object_instance_id(body: ffi::Index, id: u32) {
	Index::copy_raw(body).map_body_mut(|v| v.object_id = ObjectID::new(id));
}

unsafe extern "C" fn get_direct_state(body: ffi::Index, state: *mut ffi::body_state) {
	Index::copy_raw(body).map_body(|body| {
		match &body.body {
			Instance::Attached((body, _), space) => {
				let transform = crate::get_transform(*space, *body).expect("Invalid body or space");
				let transform = transform.sys();
				// SAFETY: sys::godot_transform is the exact same as ffi::godot_transform
				let transform: *const ffi::godot_transform = mem::transmute(transform);
				(*state).transform = *transform;
			}
			Instance::Loose(_) => {}
		}
	});
}

unsafe extern "C" fn remove_shape(body: ffi::Index, shape: i32) {
	let shape = shape as usize;
	Index::copy_raw(body).map_body_mut(|body| {
		// TODO this can panic
		body.shapes.remove(shape);
		if let Instance::Attached((_, colliders), space) = &mut body.body {
			crate::modify_space(*space, |space| {
				// TODO can panic too
				colliders
					.remove(shape)
					.map(|c| space.colliders.remove(c, &mut space.bodies, true));
			});
		}
	});
}

unsafe extern "C" fn set_shape_transform(
	body: ffi::Index,
	shape: i32,
	transform: *const ffi::godot_transform,
) {
	let shape = shape as usize;
	Index::copy_raw(body)
		.map_body_mut(|v| v.map_shape_mut(shape, |v| v.transform = conv_transform(*transform)));
}

unsafe extern "C" fn set_shape_disabled(body: ffi::Index, shape: i32, disable: bool) {
	let shape = shape as usize;
	Index::copy_raw(body).map_body_mut(|v| v.map_shape_mut(shape, |v| v.enabled = !disable));
}

unsafe extern "C" fn set_space(body: ffi::Index, space: ffi::Index) {
	let body = Index::copy_raw(body);
	if space == std::ptr::null() {
		body.map_body_mut(|body| match body.body {
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
	} else {
		if let Ok(space) = Index::copy_raw(space).map_space(|&v| v) {
			// We need to get the shapes now, as the index array will be write locked later
			let mut colliders = body
				.map_body(|body| {
					let mut colliders = Vec::with_capacity(body.shapes.len());
					for shape in &body.shapes {
						if shape.enabled {
							let transform = shape.transform;
							Index::read_shape(shape.index, |shape| {
								let collider = Some(
									ColliderBuilder::new(shape.shape().clone())
										.position(transform)
										.build(),
								);
								colliders.push(collider);
							});
						} else {
							colliders.push(None);
						}
					}
					colliders
				})
				.expect("Body is invalid");
			body.map_body_mut(|body| {
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
	}
}

unsafe extern "C" fn set_ray_pickable(body: ffi::Index, enable: bool) {
	// TODO IIRC there is no equivalent in Rapier3D, unless I misunderstand the purpose of this function
	let _ = (body, enable);
}

unsafe extern "C" fn set_state(body: ffi::Index, state: i32, value: *const ffi::godot_variant) {
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

	Index::copy_raw(body).map_body_mut(|body| {
		// SAFETY: sys::godot_variant is the exact same as ffi::godot_variant
		let value: *const sys::godot_variant = mem::transmute(value);
		let value = Variant::from_sys(*value);
		match State::new(state, &value) {
			Ok(state) => match &mut body.body {
				Instance::Attached(body, space) => {
					modify_rigid_body(*space, body.0, |body| apply(body, state))
						.expect("Invalid body or space")
				}
				Instance::Loose(body) => apply(body, state),
			},
			Err(e) => eprintln!("Invalid state: {:?}", e),
		}
		// The caller still owns the actual Variant. forget() prevents a double free.
		value.forget();
	});
}
