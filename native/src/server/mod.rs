#[macro_use]
mod ffi;
mod body;
mod joint;
mod shape;
mod space;

use crate::*;
use body::Body;
use core::convert::TryInto;
use core::fmt;
use core::mem;
use core::num::NonZeroU32;
use core::ops::DerefMut;
use joint::Joint;
use rapier3d::math::{Isometry, Rotation, Translation, Vector};
use rapier3d::na;
use shape::Shape;
use std::io;
use std::sync::RwLock;

lazy_static::lazy_static! {
	static ref SPACE_INDICES: RwLock<SparseVec<SpaceHandle>> = RwLock::new(SparseVec::new());
	static ref BODY_INDICES: RwLock<SparseVec<Body>> = RwLock::new(SparseVec::new());
	static ref JOINT_INDICES: RwLock<SparseVec<Joint>> = RwLock::new(SparseVec::new());
	static ref SHAPE_INDICES: RwLock<SparseVec<Shape>> = RwLock::new(SparseVec::new());
}

struct SparseVec<T> {
	elements: Vec<Option<T>>,
	empty_slots: Vec<usize>,
}

enum Instance<A, L> {
	Attached(A, SpaceHandle),
	Loose(L),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Index {
	Space(usize),
	Body(usize),
	Joint(usize),
	Shape(usize),
}

/// Used primarily for cleanup, as there is only one generic `free()` function
enum Type {
	Space(SpaceHandle),
	Body(Body),
	Joint(Joint),
	Shape(Shape),
}

#[derive(Debug)]
enum IndexError {
	WrongType,
	NoElement,
}

struct ObjectID(NonZeroU32);

impl<T> SparseVec<T> {
	fn new() -> Self {
		Self {
			elements: Vec::new(),
			empty_slots: Vec::new(),
		}
	}

	fn add(&mut self, element: T) -> usize {
		if let Some(index) = self.empty_slots.pop() {
			self.elements[index] = Some(element);
			index
		} else {
			self.elements.push(Some(element));
			self.elements.len() - 1
		}
	}

	fn get(&self, index: usize) -> Option<&T> {
		self.elements.get(index).map(Option::as_ref).flatten()
	}

	fn get_mut(&mut self, index: usize) -> Option<&mut T> {
		self.elements.get_mut(index).map(Option::as_mut).flatten()
	}

	fn remove(&mut self, index: usize) -> Option<T> {
		self.elements.get_mut(index).map(Option::take).flatten()
	}
}

macro_rules! map_index {
	($fn:ident, $fn_mut:ident, $fn_add:ident, $fn_remove:ident, $array:ident, $variant:ident, $struct:ty) => {
		fn $fn<F, R>(self, f: F) -> Result<R, IndexError>
		where
			F: FnOnce(&$struct) -> R,
		{
			const NAME: &str = stringify!($variant);
			if let Index::$variant(index) = self {
				let w = $array
					.read()
					.expect(&format!("Failed to read-lock {} array", NAME));
				if let Some(e) = w.get(index) {
					Ok(f(e))
				} else {
					Err(IndexError::NoElement)
				}
			} else {
				Err(IndexError::WrongType)
			}
		}

		fn $fn_mut<F, R>(self, f: F) -> Result<R, IndexError>
		where
			F: FnOnce(&mut $struct) -> R,
		{
			const NAME: &str = stringify!($variant);
			if let Index::$variant(index) = self {
				let mut w = $array
					.write()
					.expect(&format!("Failed to write-lock {} array", NAME));
				if let Some(e) = w.get_mut(index) {
					Ok(f(e))
				} else {
					Err(IndexError::NoElement)
				}
			} else {
				Err(IndexError::WrongType)
			}
		}

		fn $fn_add(element: $struct) -> Index {
			let mut w = $array.write().expect(&format!(
				"Failed to write-lock {} array",
				stringify!($variant)
			));
			let index = w.add(element);
			Self::$variant(index)
		}
	};
}

impl Index {
	map_index!(
		map_body,
		map_body_mut,
		add_body,
		remove_body,
		BODY_INDICES,
		Body,
		Body
	);
	map_index!(
		map_joint,
		map_joint_mut,
		add_joint,
		remove_joint,
		JOINT_INDICES,
		Joint,
		Joint
	);
	map_index!(
		map_shape,
		map_shape_mut,
		add_shape,
		remove_shape,
		SHAPE_INDICES,
		Shape,
		Shape
	);
	map_index!(
		map_space,
		map_space_mut,
		add_space,
		remove_space,
		SPACE_INDICES,
		Space,
		SpaceHandle
	);

	// TODO this shouldn't return (), as additional cleanup will be necessary
	fn remove(self) -> Result<Type, IndexError> {
		match self {
			Index::Body(index) => {
				let mut w = BODY_INDICES
					.write()
					.expect(&format!("Failed to write-lock {} array", stringify!(Body)));
				if let Some(element) = w.remove(index) {
					Ok(Type::Body(element))
				} else {
					Err(IndexError::NoElement)
				}
			}
			Index::Joint(index) => {
				let mut w = JOINT_INDICES
					.write()
					.expect(&format!("Failed to write-lock {} array", stringify!(Joint)));
				if let Some(element) = w.remove(index) {
					Ok(Type::Joint(element))
				} else {
					Err(IndexError::NoElement)
				}
			}
			Index::Shape(index) => {
				let mut w = SHAPE_INDICES
					.write()
					.expect(&format!("Failed to write-lock {} array", stringify!(Shape)));
				if let Some(element) = w.remove(index) {
					Ok(Type::Shape(element))
				} else {
					Err(IndexError::NoElement)
				}
			}
			Index::Space(index) => {
				let mut w = SPACE_INDICES
					.write()
					.expect(&format!("Failed to write-lock {} array", stringify!(Space)));
				if let Some(element) = w.remove(index) {
					Ok(Type::Space(element))
				} else {
					Err(IndexError::NoElement)
				}
			}
		}
	}

	/// Passes a shape to a closure if it exists, otherwise returns an error
	fn read_shape<F, R>(index: usize, f: F) -> Result<R, IndexError>
	where
		F: FnOnce(&Shape) -> R,
	{
		let w = SHAPE_INDICES
			.read()
			.expect(&format!("Failed to read-lock {} array", stringify!(Shape)));
		if let Some(element) = w.get(index) {
			Ok(f(element))
		} else {
			Err(IndexError::NoElement)
		}
	}

	/// Passes a body to a closure if it exists, otherwise returns an error
	fn read_body<F, R>(index: usize, f: F) -> Result<R, IndexError>
	where
		F: FnOnce(&Body) -> R,
	{
		let w = BODY_INDICES
			.read()
			.expect(&format!("Failed to read-lock {} array", stringify!(Body)));
		if let Some(element) = w.get(index) {
			Ok(f(element))
		} else {
			Err(IndexError::NoElement)
		}
	}

	/// Returns a raw pointer to a boxed index. Care must be taken not to leak it
	fn raw(self) -> ffi::IndexMut {
		Box::into_raw(Box::new(self)) as ffi::IndexMut
	}

	/// Converts a raw pointer into it's boxed equivalent. This is used when freeing resources
	///
	/// SAFETY: The pointer must point to a valid boxed Index.
	unsafe fn from_raw(from: ffi::IndexMut) -> Box<Self> {
		Box::from_raw(from as *mut Self)
	}

	/// Copies the Index behind a raw pointer. This is used when accessing resources
	///
	/// SAFETY: The pointer must point to a valid boxed Index.
	unsafe fn copy_raw(from: ffi::Index) -> Self {
		*(from as *const Self)
	}
}

impl ObjectID {
	fn new(n: u32) -> Option<Self> {
		NonZeroU32::new(n).map(Self)
	}
}

#[macro_export]
macro_rules! map_or_err {
	($index:expr, $func:ident) => {
		let index = $index;
		if let Err(e) = index.$func() {
			use gdnative::prelude::godot_error;
			match e {
				IndexError::WrongType => godot_error!("ID is of wrong type {:?}", $index),
				IndexError::NoElement => godot_error!("No element at ID {:?}", $index),
			}
		}
	};
	($index:expr, $func:ident, $($args:expr),*) => {
		let index = $index;
		if let Err(e) = index.$func($($args)*) {
			use gdnative::prelude::godot_error;
			match e {
				IndexError::WrongType => godot_error!("ID is of wrong type {:?}", $index),
				IndexError::NoElement => godot_error!("No element at ID {:?}", $index),
			}
		}
	};
}

fn init(ffi: &mut ffi::FFI) {
	ffi.init(print_init);
	ffi.flush_queries(print_flush_queries);
	ffi.step(step);
	ffi.sync(print_sync);
	ffi.free(free);
	body::init(ffi);
	joint::init(ffi);
	space::init(ffi);
	shape::init(ffi);
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

unsafe extern "C" fn free(index: ffi::IndexMut) {
	map_or_err!(Index::from_raw(index), remove);
}
