#![feature(destructuring_assignment)]

mod area;
mod body;
mod indices;
mod server;
mod space;
mod util;

// TODO find a proper workaround for E0446
pub use server::Index;

use gdnative::prelude::InitHandle;

extern "C" {
	fn feenableexcept(flags: i32);
}

/// Call this if you are getting NaN errors *somewhere*
#[allow(dead_code)]
fn enable_sigfpe() {
	unsafe {
		feenableexcept(9);
	}
}

/// Bogus method just so not everything gets optimized away
/// Use this with `godot_nativescript_init`
pub fn init(_: InitHandle) {}
