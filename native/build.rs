use proc_macro2::{TokenStream, Ident};
use quote::{quote, IdentFragment, TokenStreamExt, format_ident, ToTokens};
use std::env;
use std::fs::File;
use std::io::{Read, Write};
use std::path::PathBuf;

fn main() {
	println!("cargo:rerun-if-changed=build.rs");
	println!("cargo:rerun-if-changed=path/to/Cargo.lock");
	println!("cargo:rerun-if-changed=../module/api.json");

	let mut file = File::open("../module/api.json").expect("Failed to open API");
	let mut api = String::new();
	file.read_to_string(&mut api).expect("Failed to read API");
	drop(file);
	let api = json::parse(&api).expect("Failed to parse API");
	
	let mut methods = TokenStream::new();
	for (method, info) in api.entries() {
		let ret_type = info["return"]["type"].as_str().unwrap();
		let mut args = TokenStream::new();
		for a in info["args"].members() {
			let name = a["name"].as_str().unwrap();
			assert_eq!(name.find("*"), None, "Method name: \"{}\"", name);
			args.extend::<TokenStream>(name.parse().unwrap());
			args.extend(quote!(:));
			args.extend(map_type(a["type"].as_str().unwrap()));
			args.extend(quote!(,));
		}
		let method = format_ident!("{}", method);
		let ret = map_type(ret_type);
		methods.extend(quote! {
			#method: Option<unsafe extern "C" fn(#args) -> #ret>,
		});
	}
	let bindings = quote!{
		#[repr(C)]
		pub struct Api {
			#methods
		}
	};

	let out = PathBuf::from(env::var("OUT_DIR").unwrap()).join("ffi.rs");
	let mut out = File::create(out).expect("Failed to create bindings");
	out.write(bindings.to_string().as_bytes()).expect("Failed to write bindings!");
}


fn map_type(t: &str) -> TokenStream {
	let (t, ptr) = if t.ends_with(" *") { (&t[..t.len()-2], true) } else { (t, false) };
	let t = match t {
		"index_t" => quote!(*const Index),
		"index_mut_t" => quote!(*mut Index),
		"void" => quote!(()),
		"uint32_t" => quote!(u32),
		"int" => quote!(i32),
		"bool" => quote!(bool),
		"float" => quote!(f32),
		"physics_body_state_mut_t" => quote!(*mut PhysicsBodyState),
		"physics_space_state_mut_t" => quote!(*mut PhysicsSpaceState),
		"physics_area_monitor_event_mut_t" => quote!(*mut AreaMonitorEvent),
		_ if t.starts_with("godot_") => format!("gdnative::sys::{}", t).parse().unwrap(),
		_ => panic!("Unhandled type: {}", t),
	};
	if ptr {
		let mut ptr = quote!(*const);
		ptr.extend(t);
		ptr
	} else {
		t
	}
}
