#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(improper_ctypes)]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

#[macro_export]
macro_rules! gdphysics_init {
	($fn:ident) => {
		#[no_mangle]
		extern "C" fn gdphysics_init(table: *mut ffi::fn_table) {
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
	table: *mut fn_table,
}

pub type Index = u32;

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
	pub fn _new(table: *mut fn_table) -> Self {
		Self { table }
	}

	// TODO generate this with a build script
	ffi!(area_set_param, (), Index, i32, *const godot_variant);
	ffi!(body_create, Index, i32, bool);
	ffi!(flush_queries, ());
	ffi!(free, (), Index);
	ffi!(init, ());

	ffi!(body_attach_object_instance_id, (), Index, u32);
	ffi!(body_get_direct_state, (), Index, *mut body_state);
	ffi!(body_remove_shape, (), Index, i32);
	ffi!(body_set_ray_pickable, (), u32, bool);
	ffi!(
		body_add_shape,
		(),
		Index,
		Index,
		*const godot_transform,
		bool
	);
	ffi!(body_set_shape_disabled, (), Index, i32, bool);
	ffi!(
		body_set_shape_transform,
		(),
		Index,
		i32,
		*const godot_transform
	);
	ffi!(body_set_space, (), Index, Index);
	ffi!(body_set_state, (), Index, i32, *const godot_variant);

	ffi!(shape_create, Index, i32);
	ffi!(shape_set_data, (), u32, *const godot_variant);

	ffi!(space_create, Index);
	ffi!(space_get_contacts, godot_pool_vector3_array, Index);
	ffi!(space_get_contact_count, i32, Index);
	ffi!(space_set_active, (), Index, bool);
	ffi!(space_set_debug_contacts, (), Index, i32);

	ffi!(step, (), f32);
	ffi!(sync, ());
}
