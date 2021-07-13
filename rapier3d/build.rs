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
	for info in api["methods"].members() {
		let method = info["name"].as_str().unwrap();
		let ret_type = info["return_type"].as_str().unwrap();
		let mut args_unsafe = TokenStream::new();
		for a in info["arguments"].members() {
			let name = a["name"].as_str().unwrap();
			assert_eq!(name.find('*'), None, "Method name: \"{}\"", name);
			let name: TokenStream = name.parse().unwrap();
			args_unsafe.extend(name.clone());
			args_unsafe.extend(quote!(:));
			args_unsafe.extend(map_type(a["type"].as_str().unwrap()));
			args_unsafe.extend(quote!(,));
		}

		let method = format_ident!("{}", method);
		let ret = map_type(ret_type);
		methods_unsafe.extend(quote! {
			pub #method: Option<unsafe extern "C" fn(#args_unsafe) -> #ret>,
		});
	}
	quote! {
		#[repr(C)]
		pub struct UnsafeApi {
			#methods_unsafe
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
		let method_wrapped = format_ident!("ffi_{}", method);
		let method = format_ident!("{}", method);
		let ret = map_type(ret_type);
		let (ret_wrap_pre, ret_wrap_post) = if ret_type == "maybe_index_t" {
			(quote!(if let Some(r) = ), quote!({ r.raw() } else { 0 }))
		} else if ret_type == "index_t" {
			(quote!(), quote!(.raw()))
		} else {
			(quote!(), quote!())
		};
		methods.extend(quote! {
			($ffi:ident, #method, $fn:expr) => {
				#[allow(unused_imports)]
				#[allow(non_snake_case)]
				{
					use gdnative::sys;
					use crate::server::ffi::*;
					unsafe extern "C" fn #method_wrapped(#params) -> #ret {
						#convert_from_sys
						let r = $fn(#args);
						#forget
						#ret_wrap_pre r #ret_wrap_post
					}
					// SAFETY: self.table is a valid pointer
					unsafe {
						(*$ffi.table).#method = Some(#method_wrapped);
					}
				}
			};
		});
		methods_none.extend(quote!(#method: None,));
	}
	quote! {
		macro_rules! ffi {
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
		"uint8_t" => quote!(u8),
		"uint32_t" => quote!(u32),
		"int" => quote!(i32),
		"bool" => quote!(bool),
		"float" | "real_t" => quote!(f32),
		"size_t" => quote!(usize),
		"void" => quote!(()),
		"void *" => quote!(*const ()),
		"const wchar_t *" => quote!(*const wchar::wchar_t),
		"const index_t *" => quote!(*mut u64),
		"struct physics_call_result" => quote!(physics_call_result),
		"const godot_variant **" => quote!(*const *const sys::godot_variant),
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
				format!("*mut sys::{}", t).parse().unwrap()
			} else {
				format!("sys::{}", t).parse().unwrap()
			}
		}
		_ if t.starts_with("const godot_") && t.ends_with(" *") => {
			format!("*const sys::{}", &t["const ".len()..t.len() - " *".len()])
				.parse()
				.unwrap()
		}
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
		"const wchar_t *" => (quote!(#name), false),
		"struct physics_ray_info" => (quote!(&*#name), false),
		"godot_rid" => (quote!(RID), false),
		"godot_vector3 *" => (
			quote!(&mut *(#name as *mut sys::godot_vector3 as *mut Vector3)),
			false,
		),
		//"godot_variant" => (quote!(#name), false),
		"const godot_variant *" => (
			quote!(&*(#name as *const sys::godot_variant as *const Variant)),
			false,
		),
		"const godot_variant **" => (quote!(#name as *const &Variant), false),
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
		"uint8_t" => quote!(u8),
		"uint32_t" => quote!(u32),
		"int" => quote!(i32),
		"bool" => quote!(bool),
		"float" | "real_t" => quote!(f32),
		"size_t" => quote!(usize),
		"const wchar_t *" => quote!(*const wchar::wchar_t),
		"godot_rid" => quote!(RID),
		"godot_vector3 *" => quote!(&mut Vector3),
		"godot_pool_vector3_array" => quote!(TypedArray<Vector3>),
		//"godot_variant" => quote!(Variant),
		"const godot_variant *" => quote!(&Variant),
		"const godot_variant **" => quote!(*const *const sys::godot_variant),
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
