#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(improper_ctypes)]
#![allow(unused_parens)]

//include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
include!(concat!(env!("OUT_DIR"), "/ffi.rs"));

use super::ObjectID;
use gdnative::core_types::*;
use gdnative::sys;
use std::slice;

// TODO ditto below
#[repr(C)]
pub struct PhysicsBodyState {
	transform: sys::godot_transform,
	linear_velocity: sys::godot_vector3,
	angular_velocity: sys::godot_vector3,
	center_of_mass: sys::godot_vector3,
	inv_mass: f32,
	space_index: u64,
	sleeping: bool,
}

#[repr(C)]
pub struct PhysicsSpaceState {}

#[repr(C)]
pub struct AreaMonitorEvent {}

// TODO include this in api.json
#[repr(C)]
pub struct PhysicsRayResult {
	position: sys::godot_vector3,
	normal: sys::godot_vector3,
	id: u64,
	object_id: Option<ObjectID>,
	shape: i32,
}

// TODO include this in api.json
#[repr(C)]
pub struct PhysicsRayInfo {
	from: sys::godot_vector3,
	to: sys::godot_vector3,
	exclude: *const u64,
	exclude_count: usize,
	collision_mask: u32,
	collide_with_bodies: bool,
	collide_with_areas: bool,
	pick_ray: bool,
}

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
		self.space_index = index.raw();
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
		self.object_id = id
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
