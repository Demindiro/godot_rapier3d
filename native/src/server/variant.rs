// The gdnative_core Variant type requires GODOT_API to be set or it'll abort
// This is a temporary drop-in replacement, I'm certain this can be fixed in the gdnative library
use gdnative::core_types::*;
use gdnative::sys;
use super::ffi;
use core::mem;


pub enum Variant {
	Transform(Transform),
	Vector3(Vector3),
	Bool(bool),
}

macro_rules! try_to_type {
	($fn:ident, $variant:path, $ret:ident) => {
		pub fn $fn(&self) -> Option<$ret> {
			if let $variant(v) = self {
				Some(v.clone())
			} else {
				None
			}
		}
	};
}

impl Variant {
	pub fn from_sys(from: ffi::godot_variant) -> Self {
		// Used methods are listed in godot-headers/gdnative/variant.h
		// SAFETY: from is a valid pointer
		match unsafe { ffi::godot_variant_get_type(&from) } {
			ffi::godot_variant_type_GODOT_VARIANT_TYPE_VECTOR3 => {
				// SAFETY: from is a valid Vector3
				unsafe {
					let v = ffi::godot_variant_as_vector3(&from);
					Self::Vector3(Vector3::from_sys(mem::transmute(v)))
				}
			}
			ffi::godot_variant_type_GODOT_VARIANT_TYPE_TRANSFORM => {
				// SAFETY: from is a valid Transform
				unsafe {
					let v = ffi::godot_variant_as_transform(&from);
					Self::Transform(Transform::from_sys(mem::transmute(v)))
				}
			}
			ffi::godot_variant_type_GODOT_VARIANT_TYPE_BOOL => {
				// SAFETY: from is a valid Bool
				unsafe {
					Self::Bool(ffi::godot_variant_as_bool(&from))
				}
			}
			_ => todo!(),
		}
	}

	pub fn forget(self) {

	}

	try_to_type!(try_to_transform, Self::Transform, Transform);
	try_to_type!(try_to_vector3, Self::Vector3, Vector3);
	try_to_type!(try_to_bool, Self::Bool, bool);

}
