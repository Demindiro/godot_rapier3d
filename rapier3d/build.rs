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
	bindings.extend(generate_structs(&api));

	let out = PathBuf::from(env::var("OUT_DIR").unwrap()).join("ffi.rs");
	let mut out = File::create(out).expect("Failed to create bindings");
	out.write_all(bindings.to_string().as_bytes())
		.expect("Failed to write bindings!");
}

fn generate_struct(api: &json::JsonValue) -> TokenStream {
	let mut methods_unsafe = TokenStream::new();
	let mut methods_safe = TokenStream::new();
	for info in api["methods"].members() {
		let method = info["name"].as_str().unwrap();
		let ret_type = info["return_type"].as_str().unwrap();
		let mut args_unsafe = TokenStream::new();
		let mut args_safe = TokenStream::new();
		for a in info["arguments"].members() {
			let name = a["name"].as_str().unwrap();
			assert_eq!(name.find('*'), None, "Method name: \"{}\"", name);
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
	for info in api["methods"].members() {
		let method = info["name"].as_str().unwrap();
		let ret_type = info["return_type"].as_str().unwrap();
		let mut args = TokenStream::new();
		let mut params = TokenStream::new();
		let mut params_safe = TokenStream::new();
		let mut convert_from_sys = TokenStream::new();
		let mut forget = TokenStream::new();
		for a in info["arguments"].members() {
			let name = a["name"].as_str().unwrap();
			let typ = a["type"].as_str().unwrap();
			assert_eq!(name.find('*'), None, "Method name: \"{}\"", name);
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
					let w = WRAPPERS.read().expect("Failed to read method");
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

		lazy_static::lazy_static! {
			static ref WRAPPERS: RwLock<SafeApi> = RwLock::new(SafeApi { #methods_none });
		}

		impl FFI {
			#methods
		}
	}
}

fn generate_structs(api: &json::JsonValue) -> TokenStream {
	let mut structs = quote!();
	for (name, members) in api["structs"].entries() {
		let name = format_ident!("{}", name);
		let mut fields = quote!();
		for m in members.members() {
			let name: TokenStream = m["name"].as_str().unwrap().parse().unwrap();
			let typ = m["type"].as_str().unwrap();
			let typ = map_type(typ);
			fields.extend(quote!(#name: #typ,));
		}
		let struc = quote! {
			#[repr(C)]
			pub struct #name {
				#fields
			}
		};
		structs.extend(struc);
	}
	structs
}

fn map_type(t: &str) -> TokenStream {
	match t {
		"index_t" | "maybe_index_t" => quote!(u64),
		"uint32_t" => quote!(u32),
		"int" => quote!(i32),
		"bool" => quote!(bool),
		"float" | "real_t" => quote!(f32),
		"size_t" => quote!(usize),
		"void" => quote!(()),
		"void *" => quote!(*const ()),
		"const index_t *" => quote!(*mut u64),
		_ if t.starts_with("struct ") && t.ends_with(" *") => {
			format!("*mut {}", &t["struct ".len()..t.len() - " *".len()])
				.parse()
				.unwrap()
		}
		_ if t.starts_with("const struct ") && t.ends_with(" *") => {
			format!("*const {}", &t["const struct ".len()..t.len() - " *".len()])
				.parse()
				.unwrap()
		}
		_ if t.starts_with("godot_") => {
			if let Some(t) = t.strip_suffix(" *") {
				format!("&mut sys::{}", t)
					.parse()
					.unwrap()
			} else {
				format!("sys::{}", t).parse().unwrap()
			}
		}
		_ if t.starts_with("const godot_") && t.ends_with(" *") => {
			format!("&sys::{}", &t["const ".len()..t.len() - " *".len()])
				.parse()
				.unwrap()
		}
		_ => panic!("Unhandled type: {}", t),
	}
}

fn map_type_safe(t: &str) -> TokenStream {
	match t {
		"index_t" => quote!(Index),
		"maybe_index_t" => quote!(Option<Index>),
		"void" => quote!(()),
		"uint32_t" => quote!(u32),
		"int" => quote!(i32),
		"bool" => quote!(bool),
		"float" | "real_t" => quote!(f32),
		_ if t.starts_with("godot_") => {
			if let Some(t) = t.strip_suffix(" *") {
				format!("&mut sys::{}", t)
					.parse()
					.unwrap()
			} else {
				format!("sys::{}", t).parse().unwrap()
			}
		}
		_ if t.starts_with("const godot_") && t.ends_with(" *") => {
			format!("&sys::{}", &t["const ".len()..t.len() - " *".len()])
				.parse()
				.unwrap()
		}
		_ => panic!("Unhandled type: {}", t),
	}
}

fn default_value_for_type(t: &str) -> TokenStream {
	let (t, _) = if t.ends_with(" *") {
		(&t[..t.len() - 2], true)
	} else {
		(t, false)
	};
	match t {
		"void" => quote!(()),
		"index_t" | "maybe_index_t" | "int" | "uint32_t" => quote!(0),
		"bool" => quote!(false),
		"float" | "real_t" => quote!(0.0),
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
	let mut r = quote!(let #name =);
	let t = match t {
		"index_t" => (
			quote!(Index::from_raw(#name).expect("Invalid index")),
			false,
		),
		"maybe_index_t" => (
			quote!(if #name == 0 { None } else { Some(Index::from_raw(#name).expect("Invalid index")) }),
			false,
		),
		"void" => (quote!(()), false),
		"size_t" | "uint32_t" | "int" | "bool" | "float" | "real_t" | "void *" => {
			(quote!(#name), false)
		}
		"struct physics_ray_info" => (quote!(&*#name), false),
		"godot_vector3 *" => (
			quote!(&mut *(#name as *mut sys::godot_vector3 as *mut Vector3)),
			false,
		),
		"const godot_variant *" => (
			quote!(&*(#name as *const sys::godot_variant as *const Variant)),
			false,
		),
		"const godot_transform *" => (
			quote!(&*(#name as *const sys::godot_transform as *const Transform)),
			false,
		),
		"const godot_vector3 *" => (
			quote!(&*(#name as *const sys::godot_vector3 as *const Vector3)),
			false,
		),
		"godot_pool_vector3_array" => (quote!(TypedArray::<Vector3>::from_sys(#name)), false),
		_ if t.starts_with("const struct ") && t.ends_with(" *") => (quote!(&*#name), false),
		_ if t.starts_with("struct ") && t.ends_with(" *") => (quote!(&mut *#name), false),
		_ => panic!("Unhandled type: {}", t),
	};
	r.extend(t.0);
	(r, t.1)
}

fn get_type_from_sys(t: &str) -> TokenStream {
	match t {
		"index_t" => quote!(Index),
		"maybe_index_t" => quote!(Option<Index>),
		"void *" => quote!(*const ()),
		"uint32_t" => quote!(u32),
		"int" => quote!(i32),
		"bool" => quote!(bool),
		"float" | "real_t" => quote!(f32),
		"size_t" => quote!(usize),
		"godot_vector3 *" => quote!(&mut Vector3),
		"godot_pool_vector3_array" => quote!(TypedArray<Vector3>),
		"const godot_variant *" => quote!(&Variant),
		"const godot_transform *" => quote!(&Transform),
		"const godot_vector3 *" => quote!(&Vector3),
		_ if t.starts_with("struct ") && t.ends_with(" *") => {
			format!("&mut {}", &t["struct ".len()..t.len() - " *".len()])
				.parse()
				.unwrap()
		}
		_ if t.starts_with("const struct ") && t.ends_with(" *") => {
			format!("&{}", &t["const struct ".len()..t.len() - " *".len()])
				.parse()
				.unwrap()
		}
		_ => panic!("Unhandled type: {}", t),
	}
}
