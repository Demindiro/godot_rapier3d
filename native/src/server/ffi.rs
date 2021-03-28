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
