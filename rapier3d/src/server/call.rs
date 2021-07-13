use super::*;
use gdnative::prelude::*;

pub(super) type Result = core::result::Result<Variant, ffi::PhysicsCallError>;

/// Method used to access Rapier-specific functionality.
pub(super) fn call(
	method: *const wchar::wchar_t,
	arguments: *const &Variant,
	arguments_count: usize,
) -> ffi::PhysicsCallResult {
	// Godot's String includes a null char, so we can determine the length from that
	let method = unsafe {
		let mut size = 0;
		let mut m = method;
		while *m != 0 {
			m = m.add(1);
			size += 1;
		}
		core::slice::from_raw_parts(method, size)
	};
	// We'll have to trust Godot on this one (just like with 'method', really).
	let arguments = unsafe { core::slice::from_raw_parts(arguments, arguments_count) };

	use wchar::wch;
	ffi::PhysicsCallResult::new(match method {
		wch!("body_set_local_com") => body::set_local_com(arguments),
		_ => Err(ffi::PhysicsCallError::InvalidMethod),
	})
}
