#[macro_use]
mod ffi;
mod body;
mod index;
mod joint;
mod shape;
mod space;

use crate::space::Space;
use crate::sparse_vec::SparseVec;
use crate::*;
pub use body::Body;
use core::num::NonZeroU32;
use gdnative::godot_error;
pub use index::*;
use joint::Joint;
use rapier3d::na;
use shape::Shape;
use std::sync::RwLock;

lazy_static::lazy_static! {
	static ref SPACE_INDICES: RwLock<SparseVec<Space>> = RwLock::new(SparseVec::new());
	static ref BODY_INDICES: RwLock<SparseVec<Body>> = RwLock::new(SparseVec::new());
	static ref JOINT_INDICES: RwLock<SparseVec<Joint>> = RwLock::new(SparseVec::new());
	static ref SHAPE_INDICES: RwLock<SparseVec<Shape>> = RwLock::new(SparseVec::new());
	static ref ACTIVE: RwLock<bool> = RwLock::new(true);
}

enum Instance<A, L> {
	Attached(A, SpaceIndex),
	Loose(L),
}

#[derive(Debug)]
enum IndexError {
	WrongType,
	NoElement,
}

#[derive(Clone, Copy)]
pub struct ObjectID(NonZeroU32);

trait MapIndex<T> {
	fn map<F, R>(&self, f: F) -> Result<R, IndexError>
	where
		F: FnOnce(&T) -> R;

	fn map_mut<F, R>(&self, f: F) -> Result<R, IndexError>
	where
		F: FnOnce(&mut T) -> R;

	fn add(element: T) -> Self;

	fn remove(self) -> Result<T, IndexError>;
}

trait MapEnumIndex {
	fn map_body<F, R>(&self, f: F) -> Result<R, IndexError>
	where
		F: FnOnce(&Body, BodyIndex) -> R;

	fn map_body_mut<F, R>(&self, f: F) -> Result<R, IndexError>
	where
		F: FnOnce(&mut Body, BodyIndex) -> R;

	fn map_joint<F, R>(&self, f: F) -> Result<R, IndexError>
	where
		F: FnOnce(&Joint, JointIndex) -> R;

	fn map_joint_mut<F, R>(&self, f: F) -> Result<R, IndexError>
	where
		F: FnOnce(&mut Joint, JointIndex) -> R;

	fn map_shape<F, R>(&self, f: F) -> Result<R, IndexError>
	where
		F: FnOnce(&Shape, ShapeIndex) -> R;

	fn map_shape_mut<F, R>(&self, f: F) -> Result<R, IndexError>
	where
		F: FnOnce(&mut Shape, ShapeIndex) -> R;

	fn map_space<F, R>(&self, f: F) -> Result<R, IndexError>
	where
		F: FnOnce(&Space, SpaceIndex) -> R;

	fn map_space_mut<F, R>(&self, f: F) -> Result<R, IndexError>
	where
		F: FnOnce(&mut Space, SpaceIndex) -> R;
}

macro_rules! map_index {
	($array:ident, $variant:ident, $index:ident,) => {
		impl MapIndex<$variant> for $index {
			fn map<F, R>(&self, f: F) -> Result<R, IndexError>
			where
				F: FnOnce(&$variant) -> R,
			{
				let w = $array.read().expect(&format!(
					"Failed to read-lock {} array",
					stringify!($variant)
				));
				if let Some(element) = w.get(self.index() as usize) {
					Ok(f(element))
				} else {
					Err(IndexError::NoElement)
				}
			}

			fn map_mut<F, R>(&self, f: F) -> Result<R, IndexError>
			where
				F: FnOnce(&mut $variant) -> R,
			{
				let mut w = $array.write().expect(&format!(
					"Failed to write-lock {} array",
					stringify!($variant)
				));
				if let Some(element) = w.get_mut(self.index() as usize) {
					Ok(f(element))
				} else {
					Err(IndexError::NoElement)
				}
			}

			fn add(element: $variant) -> Self {
				let mut w = $array.write().expect(&format!(
					"Failed to write-lock {} array",
					stringify!($variant)
				));
				Self::new(w.add(element) as u32)
			}

			fn remove(self) -> Result<$variant, IndexError> {
				let mut w = $array.write().expect(&format!(
					"Failed to write-lock {} array",
					stringify!($variant)
				));
				if let Some(element) = w.remove(self.index() as usize) {
					Ok(element)
				} else {
					Err(IndexError::NoElement)
				}
			}
		}
	};
}

macro_rules! map_enum_index {
	($fn_map:ident, $fn_map_mut:ident, $array:ident, $variant:ident, $index:ident,) => {
		#[allow(dead_code)]
		fn $fn_map<F, R>(&self, f: F) -> Result<R, IndexError>
		where
			F: FnOnce(&$variant, $index) -> R,
		{
			if let Index::$variant(index) = self {
				index.map(|e| f(e, *index))
			} else {
				Err(IndexError::WrongType)
			}
		}

		#[allow(dead_code)]
		fn $fn_map_mut<F, R>(&self, f: F) -> Result<R, IndexError>
		where
			F: FnOnce(&mut $variant, $index) -> R,
		{
			if let Index::$variant(index) = self {
				index.map_mut(|e| f(e, *index))
			} else {
				Err(IndexError::WrongType)
			}
		}
	};
}

map_index!(BODY_INDICES, Body, BodyIndex,);
map_index!(JOINT_INDICES, Joint, JointIndex,);
map_index!(SHAPE_INDICES, Shape, ShapeIndex,);
map_index!(SPACE_INDICES, Space, SpaceIndex,);

impl MapEnumIndex for Index {
	map_enum_index!(map_body, map_body_mut, BODY_INDICES, Body, BodyIndex,);
	map_enum_index!(map_joint, map_joint_mut, JOINT_INDICES, Joint, JointIndex,);
	map_enum_index!(map_shape, map_shape_mut, SHAPE_INDICES, Shape, ShapeIndex,);
	map_enum_index!(map_space, map_space_mut, SPACE_INDICES, Space, SpaceIndex,);
}

impl ObjectID {
	fn new(n: u32) -> Option<Self> {
		NonZeroU32::new(n).map(Self)
	}

	fn get(self) -> u32 {
		self.0.get()
	}
}

#[macro_export]
macro_rules! map_or_err {
	($index:expr, $func:ident) => {
		let index = &$index;
		if let Err(e) = index.$func() {
			use gdnative::prelude::godot_error;
			match e {
				IndexError::WrongType => godot_error!("ID is of wrong type {:?}", index),
				IndexError::NoElement => godot_error!("No element at ID {:?}", index),
			}
		}
	};
	($index:expr, $func:ident, $($args:expr),*) => {
		let index = &$index;
		if let Err(e) = index.$func($($args)*) {
			use gdnative::prelude::godot_error;
			match e {
				IndexError::WrongType => godot_error!("ID is of wrong type {:?}", index),
				IndexError::NoElement => godot_error!("No element at ID {:?}", index),
			}
		}
	};
}

fn init(ffi: &mut ffi::FFI) {
	ffi.set_active(set_active);
	ffi.init(server_init);
	ffi.flush_queries(flush_queries);
	ffi.step(step);
	ffi.sync(sync);
	ffi.free(free);
	ffi.get_process_info(get_process_info);
	body::init(ffi);
	joint::init(ffi);
	space::init(ffi);
	shape::init(ffi);
}

gdphysics_init!(init);

fn server_init() {}

fn step(delta: f32) {
	if *ACTIVE.read().expect("Failed to check ACTIVE") {
		let mut w = SPACE_INDICES
			.write()
			.expect("Failed to write-lock SPACE_INDICES");
		for space in w.iter_mut() {
			space.step(delta);
		}
	}
}

fn sync() {}

fn flush_queries() {}

fn free(index: Index) {
	let rem = || -> Result<(), IndexError> {
		Ok(match index {
			Index::Body(index) => body::free(index.remove()?),
			Index::Joint(index) => joint::free(index.remove()?),
			Index::Shape(index) => shape::free(index.remove()?),
			Index::Space(index) => space::free(index.remove()?),
		})
	};
	if let Err(e) = rem() {
		godot_error!("Failed to remove index: {:?}", e);
	}
}

fn get_process_info(info: i32) -> i32 {
	godot_error!("TODO {}", info);
	0
}

fn set_active(active: bool) {
	*ACTIVE.write().expect("Failed to modify ACTIVE") = active;
}
