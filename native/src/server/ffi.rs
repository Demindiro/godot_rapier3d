#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(improper_ctypes)]

//include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
include!(concat!(env!("OUT_DIR"), "/ffi.rs"));

use gdnative::sys;


#[repr(C)]
pub struct PhysicsBodyState {
	pub transform: sys::godot_transform,
}

#[repr(C)]
pub struct PhysicsSpaceState {}

#[repr(C)]
pub struct AreaMonitorEvent {}


#[macro_export]
macro_rules! gdphysics_init {
	($fn:ident) => {
		#[no_mangle]
		extern "C" fn gdphysics_init(table: *mut ffi::Api) {
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
	table: *mut Api,
}

use super::Index;

macro_rules! ffi {
	($fn:ident, $ret:ty) => {
		pub fn $fn(&mut self, f: unsafe extern "C" fn() -> $ret) {
			unsafe {
				(*self.table).$fn = Some(f);
			}
		}
	};
	($fn:ident, $ret:ty, $($args:ty),*) => {
		pub fn $fn(&mut self, f: unsafe extern "C" fn($($args, )*) -> $ret) {
			unsafe {
				(*self.table).$fn = Some(f);
			}
		}
	};
}

impl FFI {
	pub fn _new(table: *mut Api) -> Self {
		Self { table }
	}

	// TODO generate this with a build script
	ffi!(area_set_param, (), *const Index, i32, *const sys::godot_variant);
	ffi!(body_create, *const Index, i32, bool);
	ffi!(flush_queries, ());
	ffi!(free, (), *mut Index);
	ffi!(init, ());

	ffi!(body_attach_object_instance_id, (), *const Index, u32);
	ffi!(body_get_direct_state, (), *const Index, *mut PhysicsBodyState);
	ffi!(body_remove_shape, (), *const Index, i32);
	ffi!(body_set_ray_pickable, (), *const Index, bool);
	ffi!(
		body_add_shape,
		(),
		*const Index,
		*const Index,
		*const sys::godot_transform,
		bool
	);
	ffi!(body_set_shape_disabled, (), *const Index, i32, bool);
	ffi!(
		body_set_shape_transform,
		(),
		*const Index,
		i32,
		*const sys::godot_transform
	);
	ffi!(body_set_space, (), *const Index, *const Index);
	ffi!(body_set_state, (), *const Index, i32, *const sys::godot_variant);

	ffi!(hinge_joint_get_flag, bool, *const Index, i32);
	ffi!(hinge_joint_get_param, f32, *const Index, i32);
	ffi!(hinge_joint_set_flag, (), *const Index, i32, bool);
	ffi!(hinge_joint_set_param, (), *const Index, i32, f32);
	ffi!(
		joint_create_hinge,
		*const Index,
		*const Index,
		*const sys::godot_transform,
		*const Index,
		*const sys::godot_transform
	);

	ffi!(shape_create, *const Index, i32);
	ffi!(shape_set_data, (), *const Index, *const sys::godot_variant);

	ffi!(space_create, *const Index);
	ffi!(space_get_contacts, sys::godot_pool_vector3_array, *const Index);
	ffi!(space_get_contact_count, i32, *const Index);
	ffi!(space_set_active, (), *const Index, bool);
	ffi!(space_set_debug_contacts, (), *const Index, i32);

	ffi!(step, (), f32);
	ffi!(sync, ());
}
