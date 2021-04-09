use super::index::BodyIndex;
use super::*;
use crate::body::Body;
use crate::util::*;
use gdnative::core_types::*;
use gdnative::{godot_error, godot_warn};
use rapier3d::dynamics::{ActivationStatus, BodyStatus, RigidBody, RigidBodyBuilder};

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

enum Mode {
	Static,
	Kinematic,
	Rigid,
	Character,
}

#[derive(Debug)]
enum StateError {
	InvalidType,
	InvalidValue,
}

#[derive(Debug)]
struct InvalidMode;

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
		.additional_mass(1.0)
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

impl Mode {
	fn new(n: i32) -> Result<Self, InvalidMode> {
		Ok(match n {
			0 => Self::Static,
			1 => Self::Kinematic,
			2 => Self::Rigid,
			3 => Self::Character,
			_ => return Err(InvalidMode),
		})
	}
}

pub fn init(ffi: &mut ffi::FFI) {
	ffi!(ffi, body_add_force, add_force);
	ffi!(ffi, body_add_shape, add_shape);
	ffi!(ffi, body_add_collision_exception, add_collision_exception);
	ffi!(ffi, body_apply_impulse, apply_impulse);
	ffi!(
		ffi,
		body_attach_object_instance_id,
		attach_object_instance_id
	);
	ffi!(ffi, body_create, create);
	ffi!(
		ffi,
		body_is_continuous_collision_detection_enabled,
		is_continuous_collision_detection_enabled
	);
	ffi!(ffi, body_get_contact, get_contact);
	ffi!(ffi, body_get_direct_state, get_direct_state);
	ffi!(ffi, body_get_kinematic_safe_margin, |_| 0.0);
	ffi!(ffi, body_remove_shape, remove_shape);
	ffi!(ffi, body_set_collision_layer, set_collision_layer);
	ffi!(ffi, body_set_collision_mask, set_collision_mask);
	ffi!(
		ffi,
		body_set_enable_continuous_collision_detection,
		set_enable_continuous_collision_detection
	);
	ffi!(ffi, body_set_kinematic_safe_margin, |_, _| ());
	ffi!(
		ffi,
		body_set_max_contacts_reported,
		set_max_contacts_reported
	);
	ffi!(ffi, body_set_mode, set_mode);
	ffi!(
		ffi,
		body_set_omit_force_integration,
		set_omit_force_integration
	);
	ffi!(ffi, body_set_param, set_param);
	ffi!(ffi, body_set_shape_transform, set_shape_transform);
	ffi!(ffi, body_set_shape_disabled, set_shape_disabled);
	ffi!(ffi, body_set_space, set_space);
	ffi!(ffi, body_set_state, set_state);
	ffi!(ffi, body_set_ray_pickable, set_ray_pickable);
}

/// Frees the given body, removing it from it's attached space (if any)
pub fn free(body: Body) {
	body.free()
}

fn create(typ: i32, sleep: bool) -> Option<Index> {
	if let Ok(typ) = Type::new(typ) {
		let index = BodyIndex::add(Body::new(typ.create_body(sleep)));
		index.map_mut(|b| b.set_index(index)).unwrap();
		Some(Index::Body(index))
	} else {
		godot_error!("Invalid body type");
		None
	}
}

fn add_collision_exception(body_a: Index, body_b: Index) {
	if body_a == body_b {
		return; // There is no point excluding the body from itself
	}
	if let Some(body_a) = body_a.as_body() {
		if let Some(body_b) = body_b.as_body() {
			let mut bodies = BodyIndex::write_all();
			let (body_a, body_b) = bodies.get2_mut(body_a.into(), body_b.into());
			if let Some(body_a) = body_a {
				if let Some(body_b) = body_b {
					// "Adding" the same exclusion multiple times is valid
					let _ = body_a.add_exclusion(body_b);
				} else {
					godot_error!("No body at index B");
				}
			} else {
				godot_error!("No body at index A");
			}
		} else {
			godot_error!("Index B does not point to a body");
		}
	} else {
		godot_error!("Index A does not point to a body");
	}
}

fn add_force(body: Index, force: &Vector3, position: &Vector3) {
	map_or_err!(body, map_body_mut, |body, _| {
		let (position, force) = transform_force_arguments(body, position, force);
		body.add_force_at_position(force, position);
	});
}

fn add_shape(body: Index, shape: Index, transform: &Transform, disabled: bool) {
	map_or_err!(body, map_body_mut, |body, _| {
		map_or_err!(shape, map_shape_mut, |shape, _| {
			body.add_shape(shape, transform, !disabled);
		});
	});
}

fn apply_impulse(body: Index, position: &Vector3, impulse: &Vector3) {
	map_or_err!(body, map_body_mut, |body, _| {
		let (position, impulse) = transform_force_arguments(body, position, impulse);
		body.add_impulse_at_position(impulse, position);
	});
}

fn attach_object_instance_id(body: Index, id: u32) {
	map_or_err!(body, map_body_mut, |b, _| b
		.set_object_id(ObjectID::new(id)));
}

fn get_direct_state(body: Index, state: &mut ffi::PhysicsBodyState) {
	map_or_err!(body, map_body, |body, _| {
		body.read_body(|rb, space| {
			if let Some(space) = space {
				state.set_transform(&isometry_to_transform(rb.position()));
				state.set_space(Index::Space(space));
				state.set_linear_velocity(vec_na_to_gd(*rb.linvel()));
				state.set_angular_velocity(vec_na_to_gd(*rb.angvel()));
				state.set_sleeping(rb.is_sleeping());
				state.set_linear_damp(rb.linear_damping);
				state.set_angular_damp(rb.angular_damping);
				let mp = rb.mass_properties();
				state.set_inv_mass(mp.inv_mass);
				let inv_inertia_sqrt = vec_na_to_gd(mp.inv_principal_inertia_sqrt);
				state.set_inv_inertia(inv_inertia_sqrt.component_mul(inv_inertia_sqrt));
				let inv_inertia_tensor = mp.reconstruct_inverse_inertia_matrix();
				state.set_inv_inertia_tensor(&mat3_to_basis(&inv_inertia_tensor));
				state.set_contact_count(body.contact_count());
			}
		});
	});
}

fn get_contact(body: Index, id: u32, contact: &mut ffi::PhysicsBodyContact) {
	map_or_err!(body, map_body, |body, _| {
		if let Some(c) = body.get_contact(id) {
			body.read_body(|rb, _| {
				contact.set_index(Index::Body(c.index()));
				contact.set_local_position(c.local_position(rb));
				contact.set_local_normal(c.local_normal(rb));
				contact.set_position(c.position());
				contact.set_object_id(c.object_id());
				contact.set_shape(c.other_shape());
				contact.set_local_shape(c.self_shape());
			});
		} else {
			godot_error!("Invalid contact");
		}
	});
}

fn remove_shape(body: Index, shape: i32) {
	map_or_err!(body, map_body_mut, |body, _| body
		.remove_shape(shape as u32));
}

fn set_param(body: Index, param: i32, value: f32) {
	if let Ok(param) = Param::new(param, value) {
		map_or_err!(body, map_body_mut, |body, _| {
			match param {
				Param::Mass(mass) => body.set_mass(mass),
				Param::LinearDamp(damp) => body.set_linear_damp(damp),
				Param::AngularDamp(damp) => body.set_angular_damp(damp),
				Param::GravityScale(scale) => body.set_gravity_scale(scale),
				Param::Bounce(bounce) => body.set_restitution(bounce),
				Param::Friction(friction) => body.set_friction(friction),
			}
		});
	} else {
		godot_error!("Invalid param");
	};
}

fn set_collision_layer(body: Index, layer: u32) {
	// Check if bits 16-20 are intentionally toggled by the user
	if ((layer >> 16) & 0xf) != 0 && ((layer >> 16) & 0xf) != 0xf {
		godot_warn!("Rapier3D only supports 16 bit collision groups. Don't use the upper 16 bits");
	}
	map_or_err!(body, map_body_mut, |body, _| body.set_groups(layer as u16));
}

fn set_collision_mask(body: Index, mask: u32) {
	// Check if bits 16-20 are intentionally toggled by the user
	if ((mask >> 16) & 0xf) != 0 && ((mask >> 16) & 0xf) != 0xf {
		godot_warn!("Rapier3D only supports 16 bit collision groups. Don't use the upper 16 bits");
	}
	map_or_err!(body, map_body_mut, |body, _| body.set_mask(mask as u16));
}

fn set_mode(body: Index, mode: i32) {
	match Mode::new(mode) {
		Ok(mode) => {
			let mode = match mode {
				Mode::Static => BodyStatus::Static,
				Mode::Kinematic => BodyStatus::Kinematic,
				Mode::Rigid => BodyStatus::Dynamic,
				Mode::Character => {
					godot_error!("Character mode is not supported");
					return;
				}
			};
			map_or_err!(body, map_body_mut, |body, _| body.set_body_status(mode));
		}
		Err(_) => godot_error!("Invalid mode"),
	}
}

fn set_omit_force_integration(body: Index, enable: bool) {
	map_or_err!(body, map_body_mut, |body, _| body
		.set_omit_force_integration(enable));
}

fn set_shape_transform(body: Index, shape: i32, transform: &Transform) {
	let shape = shape as u32;
	map_or_err!(body, map_body_mut, |body, _| body
		.set_shape_transform(shape, transform));
}

fn set_shape_disabled(body: Index, shape: i32, disable: bool) {
	map_or_err!(body, map_body_mut, |body, _| body
		.set_shape_enable(shape as u32, !disable));
}

fn set_space(body: Index, space: Option<Index>) {
	map_or_err!(body, map_body_mut, |body, _| {
		if let Some(space) = space {
			map_or_err!(space, map_space_mut, |space, _| body.set_space(space));
		} else {
			body.remove_from_space();
		}
	});
}

fn set_ray_pickable(body: Index, enable: bool) {
	map_or_err!(body, map_body_mut, |body, _| body.set_ray_pickable(enable));
}

fn set_state(body: Index, state: i32, value: &Variant) {
	map_or_err!(body, map_body_mut, |body, _| {
		match State::new(state, value) {
			Ok(state) => match state {
				State::Transform(trf) => body.set_transform(&trf),
				State::LinearVelocity(vel) => body.set_linear_velocity(vel),
				State::AngularVelocity(vel) => body.set_angular_velocity(vel),
				State::Sleeping(sleep) => body.set_sleeping(sleep),
				State::CanSleep(sleep) => body.set_sleep_threshold(if sleep {
					ActivationStatus::default_threshold()
				} else {
					-1.0
				}),
			},
			Err(e) => godot_error!("Invalid state: {:?}", e),
		}
	});
}

fn set_max_contacts_reported(body: Index, count: i32) {
	let count = count as u32;
	map_or_err!(body, map_body_mut, |body, _| body.set_max_contacts(count));
}

fn is_continuous_collision_detection_enabled(body: Index) -> bool {
	map_or_err!(body, map_body, |body, _| body.is_ccd_enabled()).unwrap_or(false)
}

fn set_enable_continuous_collision_detection(body: Index, enable: bool) {
	map_or_err!(body, map_body_mut, |body, _| body.enable_ccd(enable));
}

/// Godot's "It's local position but global rotation" is such a mindfuck that this function exists
/// to help out
fn transform_force_arguments(
	body: &Body,
	position: &Vector3,
	direction: &Vector3,
) -> (Vector3, Vector3) {
	(*position + body.translation(), *direction)
}
