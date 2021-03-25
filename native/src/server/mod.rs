#[macro_use]
mod ffi;
mod body;
mod shape;
mod space;

use crate::*;
use core::convert::TryInto;
use core::fmt;
use core::mem;
use core::ops::DerefMut;
use rapier3d::dynamics::{Joint, JointHandle, RigidBody, RigidBodyHandle};
use rapier3d::geometry::{Collider, ColliderHandle};
use rapier3d::math::{Isometry, Rotation, Translation, Vector};
use rapier3d::na;
use std::io;
use std::sync::RwLock;

lazy_static::lazy_static! {
	// index 0 is considered invalid
	static ref INDICES: RwLock<(Vec<Index>, Vec<ffi::Index>)> = RwLock::new((vec![Index::None], Vec::new()));
}

macro_rules! indices {
	() => {
		INDICES.read().expect("Failed to read-lock INDICES")
	};
}

macro_rules! indices_mut {
	() => {
		INDICES.write().expect("Failed to write-lock INDICES")
	};
}

type ObjectID = core::num::NonZeroU32;

enum Instance<A, L> {
	Attached(A, SpaceHandle),
	Loose(L),
}

enum Index {
	None,
	Space(SpaceHandle),
	Body(body::Body),
	Joint(Instance<JointHandle, Joint>),
	Shape(shape::Shape),
}

macro_rules! map_index_mut {
	($fn:ident, $struct:ty, $variant:path, $name:literal) => {
		fn $fn<F>(&mut self, f: F)
		where
			F: FnOnce(&mut $struct),
		{
			if let $variant(body) = self {
				f(body);
			} else {
				eprintln!("ID does not point to a {}", $name);
			}
		}
	};
}

macro_rules! map_index {
	($fn:ident, $struct:ty, $variant:path, $name:literal) => {
		fn $fn<F>(&self, f: F)
		where
			F: FnOnce(&$struct),
		{
			if let $variant(body) = self {
				f(body);
			} else {
				eprintln!("ID does not point to a {}", $name);
			}
		}
	};
}

impl Index {
	map_index_mut!(map_body_mut, body::Body, Index::Body, "Body");
	map_index_mut!(map_shape_mut, shape::Shape, Index::Shape, "Space");
	map_index_mut!(map_space_mut, SpaceHandle, Index::Space, "Space");
	map_index!(map_body, body::Body, Index::Body, "Body");
	map_index!(map_shape, shape::Shape, Index::Shape, "Space");
	map_index!(map_space, SpaceHandle, Index::Space, "Space");
}

// TODO
impl fmt::Debug for Index {
	fn fmt(&self, fmt: &mut fmt::Formatter<'_>) -> fmt::Result {
		let name = match self {
			Index::None => "None",
			Index::Space(_) => "Space",
			Index::Body(_) => "Body",
			Index::Joint(_) => "Joint",
			Index::Shape(_) => "Shape",
		};
		write!(fmt, "{}", name)
	}
}

fn init(ffi: &mut ffi::FFI) {
	ffi.init(print_init);
	ffi.flush_queries(print_flush_queries);
	ffi.step(step);
	ffi.sync(print_sync);
	ffi.free(free);
	body::init(ffi);
	space::init(ffi);
	shape::init(ffi);
}

fn add_index(index: Index) -> ffi::Index {
	let mut w = indices_mut!();
	if let Some(i) = w.1.pop() {
		assert!(std::matches!(w.0[i as usize], Index::None));
		w.0[i as usize] = index;
		i
	} else {
		let i = w.0.len().try_into().expect("Too many indices allocated");
		w.0.push(index);
		i
	}
}

fn modify_index<F>(index: ffi::Index, f: F)
where
	F: FnOnce(&mut Index),
{
	if indices_mut!().0.get_mut(index as usize).map(f).is_none() {
		eprintln!("Invalid index {}", index);
	}
}

fn read_index<F>(index: ffi::Index, f: F)
where
	F: FnOnce(&Index),
{
	if indices!().0.get(index as usize).map(f).is_none() {
		eprintln!("Invalid index {}", index);
	}
}

fn remove_index(index: ffi::Index) {
	let mut w = indices_mut!();
	let (ref mut indices, ref mut stack) = w.deref_mut();
	indices
		.get_mut(index as usize)
		.and_then(|v| {
			match v {
				Index::None => return None,
				Index::Space(h) => (),
				Index::Shape(h) => (),
				Index::Joint(h) => (),
				Index::Body(h) => (),
			}
			*v = Index::None;
			stack.push(index);
			Some(())
		})
		.or_else(|| Some(eprintln!("Attempt to remove invalid index")));
}

/// SAFETY: transform must be a valid pointer and must not be freed by the caller
unsafe fn conv_transform(transform: ffi::godot_transform) -> Isometry<f32> {
	use gdnative::{core_types::Transform, sys};
	// SAFETY: gdnative_sys::godot_transform is the exact same as ffi::godot_transform
	let transform: sys::godot_transform = mem::transmute(transform);
	let transform = Transform::from_sys(transform);
	transform_to_isometry(transform)
}

gdphysics_init!(init);

unsafe extern "C" fn print_init() {
	println!("RUST MODULES LIVE! *stomp stomp*");
}

unsafe extern "C" fn step(delta: f32) {
	crate::step_all_spaces(delta);
}

unsafe extern "C" fn print_sync() {}

unsafe extern "C" fn print_flush_queries() {}

unsafe extern "C" fn free(index: ffi::Index) {
	remove_index(index);
}
