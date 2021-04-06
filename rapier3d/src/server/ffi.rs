#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(improper_ctypes)]
#![allow(unused_parens)]
#![allow(dead_code)]
#![allow(unknown_lints)]
#![allow(clippy)]

//include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
include!(concat!(env!("OUT_DIR"), "/ffi.rs"));

use super::ObjectID;
use gdnative::core_types::*;
use gdnative::sys;
use std::slice;

pub type PhysicsBodyState = physics_body_state;
pub type PhysicsAreaMonitorEvent = physics_area_monitor_event;
pub type PhysicsRayResult = physics_ray_result;
pub type PhysicsRayInfo = physics_ray_info;

#[macro_export]
macro_rules! gdphysics_init {
	($fn:ident) => {
		#[no_mangle]
		extern "C" fn gdphysics_init(table: *mut ffi::UnsafeApi) {
			if table.is_null() {
				println!("Function table pointer is null");
			} else {
				let mut ffi = ffi::FFI::_new(table);
				$fn(&mut ffi);
			}
		}
	};
}

pub struct FFI {
	table: *mut UnsafeApi,
}

use super::Index;

impl FFI {
	pub fn _new(table: *mut UnsafeApi) -> Self {
		Self { table }
	}
}

impl PhysicsBodyState {
	pub fn set_space(&mut self, index: Index) {
		self.space = index.raw();
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
