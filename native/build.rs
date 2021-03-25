extern crate bindgen;

use std::env;
use std::path::PathBuf;

fn main() {
	generate_fn_table();
	generate_ffi_module();
}

fn generate_fn_table() {
	println!("cargo:rerun-if-changed=../module/fn_table.h");

	let bindings = bindgen::Builder::default()
		.header("../module/fn_table.h")
		.clang_arg("-DUSE_GDNATIVE_HEADERS")
		.clang_arg("-I../godot-headers")
		.parse_callbacks(Box::new(bindgen::CargoCallbacks))
		.generate()
		.expect("Unable to generate bindings");

	let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
	bindings
		.write_to_file(out_path.join("bindings.rs"))
		.expect("Couldn't write bindings!");
}

fn generate_ffi_module() {
	//println!("cargo:rustc-link-search=native={}", env::var("GODOT_PATH").unwrap());
}
