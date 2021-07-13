use super::*;
use gdnative::prelude::*;

pub(super) type Result = core::result::Result<Variant, ffi::PhysicsCallError>;

macro_rules! call_get_arg {
	(@INTERNAL $args:expr, $index:literal, $ty_fn:ident, $ty:ident) => {
		$args[$index]
			.$ty_fn()
			.ok_or(PhysicsCallError::invalid_argument($index, VariantType::$ty))
	};
	(@INTERNAL @maybe $args:expr, $index:literal, $ty_fn:ident, $ty:ident, $default:expr) => {
		if let Some(v) = $args.get($index) {
			v
				.$ty_fn()
				.ok_or(PhysicsCallError::invalid_argument($index, VariantType::$ty))
		} else {
			Ok($default)
		}
	};
	($args:ident[$index:literal] => bool) => {
		call_get_arg!(@INTERNAL $args, $index, try_to_bool, Bool)
	};
	($args:ident[$index:literal] => bool || $default:expr) => {
		call_get_arg!(@INTERNAL @maybe $args, $index, try_to_bool, Bool, $default)
	};
	($args:ident[$index:literal] => Vector3) => {
		call_get_arg!(@INTERNAL $args, $index, try_to_vector3, Vector3)
	};
	($args:ident[$index:literal] => Vector3 || $default:expr) => {
		call_get_arg!(@INTERNAL @maybe $args, $index, try_to_vector3, Vector3, $default)
	};
	($args:ident[$index:literal] => Rid) => {
		call_get_arg!(@INTERNAL $args, $index, try_to_rid, Rid)
	};
	($args:ident[$index:literal] => Rid || $default:expr) => {
		call_get_arg!(@INTERNAL @maybe $args, $index, try_to_rid, Rid, $default)
	};
}

macro_rules! call_check_arg_count {
	($args:ident in $min:literal..$max:literal) => {
		{
			const _MIN_MAX_CHECK: usize = $max - $min;
			if $args.len() < $min {
				Err(PhysicsCallError::TooFewArguments)
			} else if $args.len() > $max {
				Err(PhysicsCallError::TooManyArguments)
			} else {
				Ok(())
			}
		}
	}
}

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
