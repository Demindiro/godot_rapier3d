#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(improper_ctypes)]
#![allow(unused_parens)]
#![allow(dead_code)]
#![allow(clippy::all)]

//include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
include!(concat!(env!("OUT_DIR"), "/ffi.rs"));

use super::ObjectID;
use gdnative::core_types::*;
use gdnative::sys;
use std::slice;

pub type PhysicsCallResult = physics_call_result;
pub type PhysicsBodyState = physics_body_state;
pub type PhysicsBodyContact = physics_body_contact;
pub type PhysicsAreaMonitorEvent = physics_area_monitor_event;
pub type PhysicsRayResult = physics_ray_result;
pub type PhysicsRayInfo = physics_ray_info;
pub type PhysicsShapeResult = physics_shape_result;
pub type PhysicsShapeInfo = physics_shape_info;
pub type PhysicsServer = physics_server;

#[macro_export]
macro_rules! gdphysics_init {
	($fn:ident) => {
		#[no_mangle]
		extern "C" fn gdphysics_init(server: *mut ffi::PhysicsServer, table: *mut ffi::UnsafeApi) {
			if table.is_null() {
				println!("Function table pointer is null");
			} else {
				let mut ffi = ffi::FFI { table, server };
				$fn(&mut ffi);
			}
		}
	};
}

pub struct FFI {
	pub table: *mut UnsafeApi,
	pub server: *const PhysicsServer,
}

#[allow(dead_code)]
pub enum VariantType {
	Nil,
	Bool,
	Int,
	Real,
	String,
	Vector2, // 5
	Rect2,
	Vector3,
	Transform2d,
	Plane,
	Quat, // 10,
	Aabb,
	Basis,
	Transform,
	Color,
	NodePath, // 15
	Rid,
	Object,
	Dictionary,
	Array,
	PoolByteArray, // 20
	PoolIntArray,
	PoolRealArray,
	PoolStringArray,
	PoolVector2Array,
	PoolVector3Array, // 25
	PoolColorArray,
}

/// Error that may be returned for `ffi::call´
pub enum PhysicsCallError {
	InvalidMethod,
	InvalidArgument {
		argument: u8,
		expected_type: VariantType,
	},
	TooManyArguments,
	TooFewArguments,
	InstanceIsNull,
}

use super::Index;

impl From<VariantType> for u8 {
	fn from(vt: VariantType) -> u8 {
		match vt {
			VariantType::Nil => 0,
			VariantType::Bool => 1,
			VariantType::Int => 2,
			VariantType::Real => 3,
			VariantType::String => 4,
			VariantType::Vector2 => 5,
			VariantType::Rect2 => 6,
			VariantType::Vector3 => 7,
			VariantType::Transform2d => 8,
			VariantType::Plane => 9,
			VariantType::Quat => 10,
			VariantType::Aabb => 11,
			VariantType::Basis => 12,
			VariantType::Transform => 13,
			VariantType::Color => 14,
			VariantType::NodePath => 15,
			VariantType::Rid => 16,
			VariantType::Object => 17,
			VariantType::Dictionary => 18,
			VariantType::Array => 19,
			VariantType::PoolByteArray => 20,
			VariantType::PoolIntArray => 21,
			VariantType::PoolRealArray => 22,
			VariantType::PoolStringArray => 23,
			VariantType::PoolVector2Array => 24,
			VariantType::PoolVector3Array => 25,
			VariantType::PoolColorArray => 26,
		}
	}
}

impl PhysicsCallError {
	pub fn invalid_argument(argument: u8, expected_type: VariantType) -> Self {
		Self::InvalidArgument {
			argument,
			expected_type,
		}
	}
}

impl PhysicsCallResult {
	pub fn new(from: Result<Variant, PhysicsCallError>) -> Self {
		match from {
			Ok(v) => Self {
				value: v.forget(),
				status: 0,
				argument: 0,
				expected_type: 0,
			},
			Err(e) => Self {
				value: Variant::new().forget(),
				status: match &e {
					PhysicsCallError::InvalidMethod => 1,
					PhysicsCallError::InvalidArgument { .. } => 2,
					PhysicsCallError::TooManyArguments => 3,
					PhysicsCallError::TooFewArguments => 4,
					PhysicsCallError::InstanceIsNull => 5,
				},
				argument: match &e {
					PhysicsCallError::InvalidArgument { argument, .. } => *argument,
					_ => 0,
				},
				expected_type: match e {
					PhysicsCallError::InvalidArgument { expected_type, .. } => expected_type.into(),
					_ => 0,
				},
			},
		}
	}
}

impl PhysicsBodyState {
	pub fn set_space(&mut self, index: Option<Index>) {
		self.space = index.map(Index::raw).unwrap_or(Index::INVALID_RAW);
	}

	pub fn set_transform(&mut self, transform: &Transform) {
		// SAFETY: transform is guaranteed to be valid
		unsafe {
			self.transform = *transform.sys();
		}
	}

	pub fn set_linear_velocity(&mut self, velocity: Vector3) {
		self.linear_velocity = velocity.to_sys();
	}

	pub fn set_angular_velocity(&mut self, velocity: Vector3) {
		self.angular_velocity = velocity.to_sys();
	}

	pub fn set_inv_inertia(&mut self, inv_inertia: Vector3) {
		self.inv_inertia = inv_inertia.to_sys();
	}

	pub fn set_inv_inertia_tensor(&mut self, inv_inertia_tensor: &Basis) {
		// SAFETY: inv_inertia_tensor is guaranteed to be valid
		unsafe {
			self.inv_inertia_tensor = *inv_inertia_tensor.sys();
		}
	}

	pub fn set_gravity(&mut self, gravity: Vector3) {
		self.gravity = gravity.to_sys();
	}

	pub fn set_linear_damp(&mut self, damp: f32) {
		self.linear_damp = damp;
	}

	pub fn set_angular_damp(&mut self, damp: f32) {
		self.angular_damp = damp;
	}

	pub fn set_sleeping(&mut self, sleeping: bool) {
		self.sleeping = sleeping;
	}

	pub fn set_mass(&mut self, mass: f32) {
		self.inv_mass = 1.0 / mass;
	}

	pub fn set_inv_mass(&mut self, inv_mass: f32) {
		self.inv_mass = inv_mass;
	}

	pub fn set_center_of_mass(&mut self, center: Vector3) {
		self.center_of_mass = center.to_sys();
	}

	pub fn set_contact_count(&mut self, count: u32) {
		self.contact_count = count;
	}
}

impl PhysicsBodyContact {
	pub fn set_index(&mut self, index: Index) {
		self.index = index.raw();
	}

	pub fn set_position(&mut self, position: Vector3) {
		self.position = position.to_sys();
	}

	pub fn set_object_id(&mut self, id: Option<ObjectID>) {
		self.object_id = id.map(ObjectID::get).unwrap_or(0) as i32;
	}

	pub fn set_shape(&mut self, shape: u32) {
		self.shape = shape;
	}

	pub fn set_local_position(&mut self, position: Vector3) {
		self.local_position = position.to_sys();
	}

	pub fn set_local_normal(&mut self, normal: Vector3) {
		self.local_normal = normal.to_sys();
	}

	pub fn set_local_shape(&mut self, shape: u32) {
		self.local_shape = shape;
	}

	pub fn set_impulse(&mut self, impulse: f32) {
		self.impulse = impulse;
	}
}

impl PhysicsRayResult {
	pub fn set_position(&mut self, position: Vector3) {
		self.position = Vector3::to_sys(position)
	}

	pub fn set_normal(&mut self, normal: Vector3) {
		self.normal = Vector3::to_sys(normal)
	}

	pub fn set_index(&mut self, index: Index) {
		self.id = index.raw();
	}

	pub fn set_object_id(&mut self, id: Option<ObjectID>) {
		self.object_id = id.map(ObjectID::get).unwrap_or(0) as i32;
	}

	pub fn set_shape(&mut self, index: u32) {
		self.shape = index as i32;
	}
}

impl PhysicsRayInfo {
	pub fn from(&self) -> Vector3 {
		Vector3::from_sys(self.from)
	}

	pub fn to(&self) -> Vector3 {
		Vector3::from_sys(self.to)
	}

	pub fn exclude_raw<'a>(&'a self) -> &'a [u64] {
		// SAFETY: exclude has at least as many elements as specified in exclude_count
		unsafe { slice::from_raw_parts(self.exclude, self.exclude_count) }
	}

	pub fn collision_mask(&self) -> u32 {
		self.collision_mask
	}

	pub fn collide_with_bodies(&self) -> bool {
		self.collide_with_bodies
	}

	pub fn collide_with_areas(&self) -> bool {
		self.collide_with_areas
	}

	pub fn pick_ray(&self) -> bool {
		self.pick_ray
	}
}

impl PhysicsShapeResult {
	pub fn set_index(&mut self, index: Index) {
		self.id = index.raw();
	}

	pub fn set_object_id(&mut self, id: Option<ObjectID>) {
		self.object_id = id.map(ObjectID::get).unwrap_or(0) as i32;
	}

	pub fn set_shape(&mut self, index: u32) {
		self.shape = index as i32;
	}
}

#[derive(Debug)]
pub enum InvalidShapeIndex {
	NotAShape,
	InvalidIndex(super::InvalidIndex),
}

impl PhysicsShapeInfo {
	pub fn shape(&self) -> Result<super::ShapeIndex, InvalidShapeIndex> {
		Index::from_raw(self.shape)
			.map_err(InvalidShapeIndex::InvalidIndex)
			.and_then(|i| i.as_shape().ok_or(InvalidShapeIndex::NotAShape))
	}

	pub fn transform(&self) -> &Transform {
		// SAFETY: the two types are ABI-compatible.
		unsafe { &*(self.transform as *const _ as *const _) }
	}

	pub fn exclude_raw<'a>(&'a self) -> &'a [u64] {
		// SAFETY: exclude has at least as many elements as specified in exclude_count
		unsafe { slice::from_raw_parts(self.exclude, self.exclude_count) }
	}

	pub fn collision_mask(&self) -> u32 {
		self.collision_mask
	}

	pub fn collide_with_bodies(&self) -> bool {
		self.collide_with_bodies
	}

	pub fn collide_with_areas(&self) -> bool {
		self.collide_with_areas
	}
}

impl PhysicsAreaMonitorEvent {
	pub fn set_object_id(&mut self, id: Option<ObjectID>) {
		self.object_id = id.map(ObjectID::get).unwrap_or(0) as i32;
	}

	pub fn set_index(&mut self, index: Index) {
		self.id = index.raw();
	}

	pub fn set_added(&mut self, added: bool) {
		self.added = added;
	}
}
