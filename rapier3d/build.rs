use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use std::env;
use std::fs::File;
use std::io::{Read, Write};
use std::path::PathBuf;

fn main() {
	println!("cargo:rerun-if-changed=build.rs");
	println!("cargo:rerun-if-changed=Cargo.lock");
	println!("cargo:rerun-if-changed=api.json");

	let mut file = File::open("api.json").expect("Failed to open API");
	let mut api = String::new();
	file.read_to_string(&mut api).expect("Failed to read API");
	drop(file);
	let api = json::parse(&api).expect("Failed to parse API");

	let mut bindings = generate_struct(&api);
	bindings.extend(generate_impl(&api));

	let out = PathBuf::from(env::var("OUT_DIR").unwrap()).join("ffi.rs");
	let mut out = File::create(out).expect("Failed to create bindings");
	out.write(bindings.to_string().as_bytes())
		.expect("Failed to write bindings!");
}

fn generate_struct(api: &json::JsonValue) -> TokenStream {
	let mut methods_unsafe = TokenStream::new();
	let mut methods_safe = TokenStream::new();
	for (method, info) in api.entries() {
		let ret_type = info["return"]["type"].as_str().unwrap();
		let mut args_unsafe = TokenStream::new();
		let mut args_safe = TokenStream::new();
		for a in info["args"].members() {
			let name = a["name"].as_str().unwrap();
			assert_eq!(name.find("*"), None, "Method name: \"{}\"", name);
			let name: TokenStream = name.parse().unwrap();
			args_unsafe.extend(name.clone());
			args_unsafe.extend(quote!(:));
			args_unsafe.extend(map_type(a["type"].as_str().unwrap()));
			args_unsafe.extend(quote!(,));
			let safe_typ = get_type_from_sys(a["type"].as_str().unwrap());
			args_safe.extend(name);
			args_safe.extend(quote!(:));
			args_safe.extend(safe_typ);
			args_safe.extend(quote!(,));
		}

		let method = format_ident!("{}", method);
		let ret = map_type(ret_type);
		let ret_safe = map_type_safe(ret_type);
		methods_unsafe.extend(quote! {
			#method: Option<unsafe extern "C" fn(#args_unsafe) -> #ret>,
		});

		methods_safe.extend(quote! {
			#method: Option<fn(#args_safe) -> #ret_safe>,
		});
	}
	quote! {
		#[repr(C)]
		pub struct UnsafeApi {
			#methods_unsafe
		}

		pub struct SafeApi {
			#methods_safe
		}
	}
}

fn generate_impl(api: &json::JsonValue) -> TokenStream {
	let mut methods = TokenStream::new();
	let mut methods_none = TokenStream::new();
	for (method, info) in api.entries() {
		let ret_type = info["return"]["type"].as_str().unwrap();
		let mut args = TokenStream::new();
		let mut params = TokenStream::new();
		let mut params_safe = TokenStream::new();
		let mut convert_from_sys = TokenStream::new();
		let mut forget = TokenStream::new();
		for a in info["args"].members() {
			let name = a["name"].as_str().unwrap();
			let typ = a["type"].as_str().unwrap();
			assert_eq!(name.find("*"), None, "Method name: \"{}\"", name);
			let name: TokenStream = name.parse().unwrap();
			params.extend(name.clone());
			params.extend(quote!(:));
			params.extend(map_type(typ));
			params.extend(quote!(,));
			params_safe.extend(name.clone());
			params_safe.extend(quote!(:));
			params_safe.extend(get_type_from_sys(typ));
			params_safe.extend(quote!(,));
			let (tk, do_forget) = convert_type_from_sys(&name, typ);
			if do_forget {
				args.extend(quote!(&));
				forget.extend(quote!(#name.forget();));
			}
			args.extend(name.clone());
			args.extend(quote!(,));
			convert_from_sys.extend(tk);
			convert_from_sys.extend(quote!(;));
		}
		let method = format_ident!("{}", method);
		let ret = map_type(ret_type);
		let ret_safe = map_type_safe(ret_type);
		let default_ret = default_value_for_type(ret_type);
		let (ret_wrap_pre, ret_wrap_post) = if ret_type == "maybe_index_t" {
			(quote!(if let Some(r) = ), quote!({ r.raw() } else { 0 }))
		} else if ret_type == "index_t" {
			(quote!(), quote!(.raw()))
		} else {
			(quote!(), quote!())
		};
		methods.extend(quote! {
			pub fn #method(&mut self, f: fn(#params_safe) -> #ret_safe) {
				unsafe extern "C" fn wrap(#params) -> #ret {
					let w = WRAPPERS.read().expect("Failed to add method");
					if let Some(m) = w.#method {
						#convert_from_sys
						let r = m(#args);
						#forget
						#ret_wrap_pre r #ret_wrap_post
					} else {
						//godot_error!("Method {} is not set", stringify!(#method));
						#default_ret
					}
				}
				// SAFETY: self.table is a valid pointer
				unsafe {
					(*self.table).#method = Some(wrap);
					let mut w = WRAPPERS.write().expect("Failed to add method");
					w.#method = Some(f);
				}
			}
		});
		methods_none.extend(quote!(#method: None,));
	}
	quote! {
		use std::sync::RwLock;
		use gdnative::prelude::*;
		use core::ptr;

		lazy_static::lazy_static! {
			static ref WRAPPERS: RwLock<SafeApi> = RwLock::new(SafeApi { #methods_none });
		}

		impl FFI {
			#methods
		}
	}
}

fn map_type(t: &str) -> TokenStream {
	let (t, ptr) = if t.ends_with(" *") {
		(&t[..t.len() - 2], true)
	} else {
		(t, false)
	};
	let mut cons = t != "godot_object";
	let t = match t {
		"index_t" | "maybe_index_t" | "index_mut_t" => quote!(u64),
		"void" => quote!(()),
		"uint32_t" => quote!(u32),
		"int" => quote!(i32),
		"bool" => quote!(bool),
		"float" => quote!(f32),
		"size_t" => quote!(usize),
		"physics_body_state_mut_t" => quote!(*mut PhysicsBodyState),
		"physics_space_state_mut_t" => quote!(*mut PhysicsSpaceState),
		"physics_area_monitor_event_mut_t" => quote!(*mut AreaMonitorEvent),
		"struct physics_ray_result" => {
			cons = false;
			quote!(PhysicsRayResult)
		}
		"struct physics_ray_info" => {
			cons = true;
			quote!(PhysicsRayInfo)
		}
		_ if t.starts_with("godot_") => format!("gdnative::sys::{}", t).parse().unwrap(),
		_ => panic!("Unhandled type: {}", t),
	};
	if ptr {
		let mut ptr = if cons { quote!(*const) } else { quote!(*mut) };
		ptr.extend(t);
		ptr
	} else {
		t
	}
}

fn map_type_safe(t: &str) -> TokenStream {
	let (t, ptr) = if t.ends_with(" *") {
		(&t[..t.len() - 2], true)
	} else {
		(t, false)
	};
	let cons = t != "godot_object";
	let t = match t {
		"index_t" | "index_mut_t" => quote!(Index),
		"maybe_index_t" => quote!(Option<Index>),
		"void" => quote!(()),
		"uint32_t" => quote!(u32),
		"int" => quote!(i32),
		"bool" => quote!(bool),
		"float" => quote!(f32),
		"physics_body_state_mut_t" => quote!(&mut PhysicsBodyState),
		"physics_space_state_mut_t" => quote!(&mut PhysicsSpaceState),
		"physics_area_monitor_event_mut_t" => quote!(&mut AreaMonitorEvent),
		_ if t.starts_with("godot_") => format!("gdnative::sys::{}", t).parse().unwrap(),
		_ => panic!("Unhandled type: {}", t),
	};
	if ptr {
		let mut ptr = if cons { quote!(&) } else { quote!(&mut) };
		ptr.extend(t);
		ptr
	} else {
		t
	}
}

fn default_value_for_type(t: &str) -> TokenStream {
	let (t, _) = if t.ends_with(" *") {
		(&t[..t.len() - 2], true)
	} else {
		(t, false)
	};
	match t {
		"index_t" | "maybe_index_t" | "index_mut_t" => quote!(0),
		"void" => quote!(()),
		"uint32_t" => quote!(0),
		"int" => quote!(0),
		"bool" => quote!(false),
		"float" => quote!(0.0),
		"physics_body_state_mut_t" => quote!(ptr::null()),
		"physics_space_state_mut_t" => quote!(ptr::null()),
		"physics_area_monitor_event_mut_t" => quote!(ptr::null()),
		"godot_variant" => quote!(Variant::new().to_sys()),
		"godot_transform" => quote!(*Transform {
			basis: Basis::identity(),
			origin: Vector3::zero()
		}
		.sys()),
		"godot_vector3" => quote!(Vector3::zero().to_sys()),
		"godot_pool_vector3_array" => quote!(*TypedArray::<Vector3>::new().sys()),
		_ => panic!("Unhandled type: {}", t),
	}
}

fn convert_type_from_sys(name: &TokenStream, t: &str) -> (TokenStream, bool) {
	let (t, _) = if t.ends_with(" *") {
		(&t[..t.len() - 2], true)
	} else {
		(t, false)
	};
	let mut r = quote!(let #name =);
	let t = match t {
		"index_t" | "index_mut_t" => (
			quote!(Index::from_raw(#name).expect("Invalid index")),
			false,
		),
		"maybe_index_t" => (
			quote!(if #name == 0 { None } else { Some(Index::from_raw(#name).expect("Invalid index")) }),
			false,
		),
		"void" => (quote!(()), false),
		"size_t" | "uint32_t" | "int" | "bool" | "float" => (quote!(#name), false),
		"physics_body_state_mut_t"
		| "physics_space_state_mut_t"
		| "physics_area_monitor_event_mut_t"
		| "struct physics_ray_result" => (quote!(&mut *#name), false),
		"struct physics_ray_info" => (quote!(&*#name), false),
		"godot_variant" => (quote!(Variant::from_sys(*#name)), true),
		"godot_transform" => (quote!(&Transform::from_sys(*#name)), false),
		"godot_vector3" => (quote!(Vector3::from_sys(*#name)), false),
		"godot_pool_vector3_array" => (quote!(TypedArray::<Vector3>::from_sys(#name)), false),
		// FIXME new or new_unchecked? (NonNull)
		"godot_object" => (
			quote!(Ref::from_sys(ptr::NonNull::new_unchecked(#name))),
			false,
		),
		_ => panic!("Unhandled type: {}", t),
	};
	r.extend(t.0);
	(r, t.1)
}

fn get_type_from_sys(t: &str) -> TokenStream {
	let t = if t.ends_with(" *") {
		&t[..t.len() - 2]
	} else {
		t
	};
	let t = match t {
		"index_t" => quote!(Index),
		"index_mut_t" => quote!(Index),
		"maybe_index_t" => quote!(Option<Index>),
		"void" => quote!(()),
		"uint32_t" => quote!(u32),
		"int" => quote!(i32),
		"bool" => quote!(bool),
		"float" => quote!(f32),
		"size_t" => quote!(usize),
		"physics_body_state_mut_t" => quote!(&mut PhysicsBodyState),
		"physics_space_state_mut_t" => quote!(&mut PhysicsSpaceState),
		"physics_area_monitor_event_mut_t" => quote!(&mut AreaMonitorEvent),
		"struct physics_ray_result" => quote!(&mut PhysicsRayResult),
		"struct physics_ray_info" => quote!(&PhysicsRayInfo),
		"godot_variant" => quote!(&Variant),
		"godot_transform" => quote!(&Transform),
		"godot_vector3" => quote!(Vector3),
		"godot_pool_vector3_array" => quote!(TypedArray<Vector3>),
		"godot_object" => quote!(Ref<Object>),
		_ => panic!("Unhandled type: {}", t),
	};
	t
}
