use super::index::BodyIndex;
use super::*;
use crate::util::*;
use gdnative::core_types::*;
use gdnative::godot_error;
use rapier3d::dynamics::{
	ActivationStatus, MassProperties, RigidBody, RigidBodyBuilder, RigidBodyHandle,
};
use rapier3d::geometry::{Collider, ColliderBuilder, ColliderHandle};
use rapier3d::math::Isometry;
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
enum Param {
	Bounce(f32),
	Friction(f32),
	Mass(f32),
	GravityScale(f32),
	LinearDamp(f32),
	AngularDamp(f32),
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
	index: ShapeIndex,
	transform: Isometry<f32>,
	scale: Vector3,
	enabled: bool,
}

pub struct Body {
	body: Instance<(RigidBodyHandle, Vec<Option<ColliderHandle>>), RigidBody>,
	object_id: Option<ObjectID>,
	shapes: Vec<Shape>,
	scale: Vector3,
	exclusions: Vec<BodyIndex>,
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

impl Param {
	fn new(n: i32, value: f32) -> Result<Param, ()> {
		Ok(match n {
			0 => Self::Bounce(value),
			1 => Self::Friction(value),
			2 => Self::Mass(value),
			3 => Self::GravityScale(value),
			4 => Self::LinearDamp(value),
			5 => Self::AngularDamp(value),
			_ => return Err(()),
		})
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
			scale: Vector3::one(),
			exclusions: Vec::new(),
		}
	}

	fn map_shape_mut<F>(&mut self, index: u32, f: F)
	where
		F: FnOnce(&mut Shape),
	{
		if let Some(shape) = self.shapes.get_mut(index as usize) {
			f(shape);
		} else {
			godot_error!("Invalid shape index");
		}
	}

	pub fn as_attached(&self) -> Option<(RigidBodyHandle, SpaceIndex)> {
		if let Instance::Attached(body, space) = &self.body {
			Some((body.0, *space))
		} else {
			None
		}
	}

	fn add_shape(&mut self, index: ShapeIndex, transform: &Transform, enabled: bool) {
		self.shapes.push(Shape {
			index,
			transform: transform_to_isometry(*transform),
			enabled,
			scale: Vector3::one(),
		});
		self.recalculate_inertia();
	}

	/// Recalculates the inertia based on all active shapes
	fn recalculate_inertia(&mut self) {
		let shapes = &self.shapes;
		// TODO avoid calculating the MassProperties twice
		let calc_mp = |body: &mut RigidBody| {
			let mut mp = Vec::with_capacity(shapes.len());
			for shape in shapes.iter() {
				if shape.enabled {
					let trf = shape.transform;
					let shape = shape
						.index
						.map(|shape| shape.shape().clone())
						.expect("Invalid shape index");
					mp.push((trf, shape));
				}
			}
			let mp_f = MassProperties::from_compound(1.0, &mp[..]);
			let ratio = mp_f.inv_mass / body.mass_properties().inv_mass;
			let mut mp = MassProperties::from_compound(ratio, &mp[..]);
			// Workaround for https://github.com/dimforge/parry/issues/20
			if mp.local_com.x.is_nan() || mp.local_com.x.is_infinite() {
				mp.local_com = na::Point3::new(0.0, 0.0, 0.0);
			};
			body.set_mass_properties(mp, true);
		};
		match &mut self.body {
			Instance::Attached((body, _), space) => {
				space
					.map_mut(|space| {
						calc_mp(
							space
								.bodies_mut()
								.get_mut(*body)
								.expect("Invalid body handle"),
						)
					})
					.expect("Invalid space handle");
			}
			Instance::Loose(body) => calc_mp(body),
		}
	}

	fn set_shape_userdata(collider: &mut Collider, body: BodyIndex, shape: u32) {
		let body = body.index() as u128 | ((body.generation() as u128) << 32);
		let shape = (shape as u128) << 64;
		collider.user_data = body | shape;
	}

	pub fn get_shape_userdata(collider: &Collider) -> (BodyIndex, u32) {
		let body = collider.user_data as u64;
		let shape = (collider.user_data >> 64) as u32;
		(BodyIndex::new(body as u32, (body >> 32) as u16), shape)
	}

	pub fn object_id(&self) -> Option<ObjectID> {
		self.object_id
	}

	/// Creates colliders according to shape enable status and transform
	fn create_colliders(&self, index: BodyIndex) -> Vec<Option<Collider>> {
		let mut colliders = Vec::with_capacity(self.shapes.len());
		for (i, shape) in self.shapes.iter().enumerate() {
			if shape.enabled {
				let transform = shape.transform;
				let shape_scale = transform.rotation * vec_gd_to_na(shape.scale);
				let shape_scale = vec_gd_to_na(self.scale).component_mul(&shape_scale);
				let shape_scale = vec_na_to_gd(shape_scale);
				let result = shape.index.map(|shape| {
					let mut collider = ColliderBuilder::new(shape.scaled(shape_scale))
						.position_wrt_parent(transform)
						.build();
					Body::set_shape_userdata(&mut collider, index, i as u32);
					colliders.push(Some(collider));
				});
				if let Err(_) = result {
					godot_error!("Shape is invalid: {:?}", shape.index);
					colliders.push(None); // Make sure the collider count does still match
				}
			} else {
				colliders.push(None);
			}
		}
		colliders
	}

	/// Store the given index in the userdata
	fn set_index(&mut self, index: BodyIndex) {
		if let Instance::Loose(body) = &mut self.body {
			body.user_data = index.index() as u128 | ((index.generation() as u128) << 32);
		} else {
			// Indices shouldn't be able to change after being attached
			unreachable!();
		}
	}

	/// Get the index from the given body's userdata
	pub fn get_index(body: &RigidBody) -> BodyIndex {
		let body = body.user_data as u64;
		BodyIndex::new(body as u32, (body >> 32) as u16)
	}

	/// Frees this body, removing it from it's attached space (if any)
	fn free(self) {
		if let Instance::Attached((rb, _), space) = &self.body {
			space
				.map_mut(|space| {
					space.remove_body(*rb);
				})
				.expect("Invalid space");
		}
	}
}

pub fn init(ffi: &mut ffi::FFI) {
	ffi.body_add_force(add_force);
	ffi.body_add_shape(add_shape);
	ffi.body_add_collision_exception(add_collision_exception);
	ffi.body_apply_impulse(apply_impulse);
	ffi.body_attach_object_instance_id(attach_object_instance_id);
	ffi.body_create(create);
	ffi.body_get_direct_state(get_direct_state);
	ffi.body_remove_shape(remove_shape);
	ffi.body_set_param(set_param);
	ffi.body_set_shape_transform(set_shape_transform);
	ffi.body_set_shape_disabled(set_shape_disabled);
	ffi.body_set_space(set_space);
	ffi.body_set_state(set_state);
	ffi.body_set_ray_pickable(set_ray_pickable);
}

/// Frees the given body, removing it from it's attached space (if any)
pub fn free(body: Body) {
	body.free()
}

fn create(r#type: i32, sleep: bool) -> Option<Index> {
	if let Ok(r#type) = Type::new(r#type) {
		let index = BodyIndex::add(Body::new(r#type, sleep));
		index.map_mut(|b| b.set_index(index)).unwrap();
		Some(Index::Body(index))
	} else {
		godot_error!("Invalid body type");
		None
	}
}

fn add_collision_exception(body_a: Index, body_b: Index) {
	map_or_err!(body_a, map_body_mut, |body_a, index_a| {
		if let Index::Body(index_b) = body_b {
			if !body_a.exclusions.contains(&index_b) {
				body_a.exclusions.push(index_b);
				if let Instance::Attached(_, space) = &body_a.body {
					space
						.map_mut(|space| {
							if let Err(_) = space.add_body_exclusion(index_a, index_b) {
								godot_error!("Failed to remove body exclusion");
							}
						})
						.expect("Invalid space");
				}
			} else {
				godot_error!("Body A already excludes B");
			}
		}
	});
	map_or_err!(body_b, map_body_mut, |body_b, _| {
		if let Index::Body(index_a) = body_a {
			if !body_b.exclusions.contains(&index_a) {
				body_b.exclusions.push(index_a);
			} else {
				godot_error!("Body B already excludes A");
			}
		}
	});
}

fn add_force(body: Index, force: &Vector3, position: &Vector3) {
	map_or_err!(body, map_body, |body, _| {
		if let Instance::Attached((body, _), space) = &body.body {
			space
				.map_mut(|space| {
					let body = space
						.bodies_mut()
						.get_mut(*body)
						.expect("Invalid body handle");
					let (position, force) = transform_force_arguments(body, position, force);
					body.apply_force_at_point(force, position, true);
				})
				.expect("Invalid space handle");
		} else {
			godot_error!("Can't apply force to body outside space");
		}
	});
}

fn add_shape(body: Index, shape: Index, transform: &Transform, disabled: bool) {
	map_or_err!(body, map_body_mut, |body, _| {
		if let Index::Shape(index) = shape {
			body.add_shape(index, transform, !disabled);
		} else {
			godot_error!("ID does not point to a shape");
		}
	});
}

fn apply_impulse(body: Index, position: &Vector3, impulse: &Vector3) {
	map_or_err!(body, map_body, |body, _| {
		if let Instance::Attached((body, _), space) = &body.body {
			space
				.map_mut(|space| {
					let body = space
						.bodies_mut()
						.get_mut(*body)
						.expect("Invalid body handle");
					let (position, impulse) = transform_force_arguments(body, position, impulse);
					body.apply_impulse_at_point(impulse, position, true);
				})
				.expect("Invalid space handle");
		} else {
			godot_error!("Can't apply impulse to body outside space");
		}
	});
}

fn attach_object_instance_id(body: Index, id: u32) {
	map_or_err!(body, map_body_mut, |v, _| v.object_id = ObjectID::new(id));
}

fn get_direct_state(body: Index, state: &mut ffi::PhysicsBodyState) {
	map_or_err!(body, map_body, |body, _| {
		match &body.body {
			Instance::Attached((body, _), space) => {
				space
					.map(|s| {
						let body = s.bodies().get(*body).expect("Invalid body handle");
						state.set_transform(&isometry_to_transform(body.position()));
						state.set_space(Index::Space(*space));
						state.set_linear_velocity(vec_na_to_gd(*body.linvel()));
						state.set_angular_velocity(vec_na_to_gd(*body.angvel()));
						state.set_sleeping(body.is_sleeping());
						state.set_inv_mass(body.mass_properties().inv_mass);
					})
					.expect("Invalid space");
			}
			Instance::Loose(_) => {}
		}
	});
}

fn remove_shape(body: Index, shape: i32) {
	let shape = shape as usize;
	map_or_err!(body, map_body_mut, |body, _| {
		// TODO this can panic
		body.shapes.remove(shape);
		if let Instance::Attached((_, colliders), space) = &mut body.body {
			space
				.map_mut(|space| {
					// TODO can panic too
					colliders.remove(shape).map(|c| space.remove_collider(c));
				})
				.expect("Failed to modify space");
		}
	});
}

fn set_param(body: Index, param: i32, value: f32) {
	let param = if let Ok(param) = Param::new(param, value) {
		param
	} else {
		godot_error!("Invalid param");
		return;
	};
	let f = |body: &mut RigidBody| match param {
		Param::Mass(mass) => {
			let mut p = *body.mass_properties();
			p.inv_mass = 1.0 / mass;
			body.set_mass_properties(p, true);
		}
		Param::LinearDamp(damp) => body.linear_damping = damp,
		Param::AngularDamp(damp) => body.angular_damping = damp,
		Param::Bounce(_bounce) => godot_error!("TODO bounce"),
		Param::Friction(_friction) => godot_error!("TODO friction"),
		Param::GravityScale(scale) => body.set_gravity_scale(scale, true),
	};
	map_or_err!(body, map_body_mut, |body, _| {
		match &mut body.body {
			Instance::Attached((body, _), space) => {
				space
					.map_mut(|space| {
						f(space
							.bodies_mut()
							.get_mut(*body)
							.expect("Invalid body handle"))
					})
					.expect("Invalid space handle");
			}
			Instance::Loose(body) => {
				f(body);
			}
		}
		body.recalculate_inertia();
	});
}

fn set_shape_transform(body: Index, shape: i32, transform: &Transform) {
	let shape = shape as u32;
	map_or_err!(body, map_body_mut, |v, _| v.map_shape_mut(shape, |v| (
		v.transform,
		v.scale
	) =
		transform_to_isometry_and_scale(transform)));
}

fn set_shape_disabled(body: Index, shape: i32, disable: bool) {
	let shape = shape as u32;
	map_or_err!(body, map_body_mut, |v, _| v
		.map_shape_mut(shape, |v| v.enabled = !disable));
}

fn set_space(body: Index, space: Option<Index>) {
	if let Some(space) = space {
		if let Index::Space(space) = space {
			let colliders = body
				.map_body(|body, index| body.create_colliders(index))
				.expect("Body is invalid");
			map_or_err!(body, map_body_mut, |body, body_index| {
				// FIXME remove exceptions
				let b = Instance::Attached((RigidBodyHandle::invalid(), Vec::new()), space);
				body.recalculate_inertia();
				let b = mem::replace(&mut body.body, b);
				body.body = match b {
					Instance::Attached(_, _) => todo!(),
					Instance::Loose(b) => {
						let mut collider_handles = Vec::with_capacity(colliders.len());
						let handle = space
							.map_mut(|space| {
								let handle = space.add_body(b);
								for collider in colliders {
									let handle = collider.map(|c| space.add_collider(c, handle));
									collider_handles.push(handle);
								}
								for &exclude in body.exclusions.iter() {
									if let Err(_) = space.add_body_exclusion(body_index, exclude) {
										godot_error!("Failed to add body exclusion");
									}
								}
								handle
							})
							.expect("Invalid space");
						Instance::Attached((handle, collider_handles), space)
					}
				};
			});
		}
	} else {
		// FIXME remove exceptions
		map_or_err!(body, map_body_mut, |body, _| {
			let rb = match body.body {
				Instance::Attached((body, _), space) => {
					space
						.map_mut(|space| space.remove_body(body))
						.expect("Failed to modify space")
						.expect("Invalid body handle")
				}
				Instance::Loose(_) => todo!(),
			};
			body.body = Instance::Loose(rb);
		});
	}
}

fn set_ray_pickable(body: Index, enable: bool) {
	// TODO IIRC there is no equivalent in Rapier3D, unless I misunderstand the purpose of this function
	let _ = (body, enable);
}

fn set_state(body: Index, state: i32, value: &Variant) {
	let apply = |scale: &mut _, body: &mut RigidBody, state| match state {
		State::Transform(trf) => {
			let (iso, scl) = transform_to_isometry_and_scale(&trf);
			body.set_position(iso, true);
			*scale = scl;
		}
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

	map_or_err!(body, map_body_mut, |body, body_index| {
		let mut scale = body.scale;
		let mut scale_changed = false;
		match State::new(state, value) {
			Ok(state) => match &mut body.body {
				Instance::Attached((rb, _), space) => {
					let old_scale = scale;
					space
						.map_mut(|space| {
							apply(
								&mut scale,
								space
									.bodies_mut()
									.get_mut(*rb)
									.expect("Invalid body handle"),
								state,
							)
						})
						.expect("Invalid body or space");
					if (old_scale.x - scale.x).abs() > 1e-6
						|| (old_scale.y - scale.y).abs() > 1e-6
						|| (old_scale.z - scale.z).abs() > 1e-6
					{
						scale_changed = true;
						body.scale = scale;
					}
				}
				Instance::Loose(rb) => {
					apply(&mut scale, rb, state);
					body.scale = scale;
				}
			},
			Err(e) => godot_error!("Invalid state: {:?}", e),
		}

		// Recreating colliders is potentially very expensive, so avoid it when possible
		// is_equal_approx (not implemented yet) should be close enough
		// FIXME `set_shape` exists since 0.7, use that instead of the current remove/read thing.
		// FIXME temporarily disabled due to https://github.com/dimforge/rapier/issues/163
		if scale_changed {
			body.recalculate_inertia();
			let colliders = body.create_colliders(body_index);
			if let Instance::Attached((rb, _), space) = &mut body.body {
				space
					.map_mut(|space| {
						// Removing the body also removes all of it's colliders
						// FIXME this also removes all joints, so those need to be recreated
						let rigid = space.remove_body(*rb).expect("Invalid body");
						*rb = space.add_body(rigid);
						let mut collider_handles = Vec::with_capacity(colliders.len());
						for collider in colliders {
							let handle = collider.map(|c| space.add_collider(c, *rb));
							collider_handles.push(handle);
						}
					})
					.expect("Invalid space");
			} else {
				unreachable!();
			}
		}
	});
}

/// Godot's "It's local position but global rotation" is such a mindfuck that this function exists
/// to help out
fn transform_force_arguments(
	body: &RigidBody,
	position: &Vector3,
	direction: &Vector3,
) -> (na::Point3<f32>, na::Vector3<f32>) {
	let position = na::Point3::new(position.x, position.y, position.z);
	let tr = body.position().translation;
	let position = position + na::Vector3::new(tr.x, tr.y, tr.z);
	let direction = vec_gd_to_na(*direction);
	(position, direction)
}
