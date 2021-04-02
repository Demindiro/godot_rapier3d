#[macro_use]
mod ffi;
mod body;
mod joint;
mod shape;
mod space;

use crate::*;
pub use body::Body;
use core::num::NonZeroU32;
use joint::Joint;
use rapier3d::na;
use shape::Shape;
use std::sync::RwLock;

lazy_static::lazy_static! {
	static ref SPACE_INDICES: RwLock<SparseVec<SpaceHandle>> = RwLock::new(SparseVec::new());
	static ref BODY_INDICES: RwLock<SparseVec<Body>> = RwLock::new(SparseVec::new());
	static ref JOINT_INDICES: RwLock<SparseVec<Joint>> = RwLock::new(SparseVec::new());
	static ref SHAPE_INDICES: RwLock<SparseVec<Shape>> = RwLock::new(SparseVec::new());
	static ref ACTIVE: RwLock<bool> = RwLock::new(true);
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
pub enum Index {
	Space(u32),
	Body(u32),
	Joint(u32),
	Shape(u32),
}

#[derive(Debug)]
enum IndexError {
	Invalid,
	WrongType,
	NoElement,
}

#[derive(Clone, Copy)]
pub struct ObjectID(NonZeroU32);

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
		if index < self.elements.len() {
			if let Some(element) = self.elements[index].take() {
				self.empty_slots.push(index);
				return Some(element);
			}
		}
		None
	}
}

macro_rules! map_index {
	($fn:ident, $fn_mut:ident, $fn_add:ident, $fn_remove:ident, $array:ident, $variant:ident, $struct:ty) => {
		#[allow(dead_code)]
		fn $fn<F, R>(self, f: F) -> Result<R, IndexError>
		where
			F: FnOnce(&$struct, u32) -> R,
		{
			const NAME: &str = stringify!($variant);
			if let Index::$variant(index) = self {
				let w = $array
					.read()
					.expect(&format!("Failed to read-lock {} array", NAME));
				if let Some(e) = w.get(index as usize) {
					Ok(f(e, index))
				} else {
					Err(IndexError::NoElement)
				}
			} else {
				Err(IndexError::WrongType)
			}
		}

		#[allow(dead_code)]
		fn $fn_mut<F, R>(self, f: F) -> Result<R, IndexError>
		where
			F: FnOnce(&mut $struct, u32) -> R,
		{
			const NAME: &str = stringify!($variant);
			if let Index::$variant(index) = self {
				let mut w = $array
					.write()
					.expect(&format!("Failed to write-lock {} array", NAME));
				if let Some(e) = w.get_mut(index as usize) {
					Ok(f(e, index))
				} else {
					Err(IndexError::NoElement)
				}
			} else {
				Err(IndexError::WrongType)
			}
		}

		#[allow(dead_code)]
		fn $fn_add(element: $struct) -> Index {
			let mut w = $array.write().expect(&format!(
				"Failed to write-lock {} array",
				stringify!($variant)
			));
			let index = w.add(element);
			Self::$variant(index as u32)
		}

		#[allow(dead_code)]
		fn $fn_remove(self) -> Result<$struct, IndexError> {
			if let Index::$variant(index) = self {
				let mut w = $array.write().expect(&format!(
					"Failed to write-lock {} array",
					stringify!($variant)
				));
				if let Some(element) = w.remove(index as usize) {
					Ok(element)
				} else {
					Err(IndexError::NoElement)
				}
			} else {
				Err(IndexError::WrongType)
			}
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

	fn remove(self) -> Result<(), IndexError> {
		match self {
			Index::Body(_) => self.remove_body().map(Body::free),
			Index::Joint(_) => self.remove_joint().map(Joint::free),
			Index::Shape(_) => self.remove_shape().map(Shape::free),
			// TODO the space <-> "world" mapping is a mess, so skip for now.
			Index::Space(_) => self.remove_space().map(|_| ()), //.map(Space::free),
		}
	}

	/// Passes a shape to a closure if it exists, otherwise returns an error
	fn read_shape<F, R>(index: u32, f: F) -> Result<R, IndexError>
	where
		F: FnOnce(&Shape) -> R,
	{
		let w = SHAPE_INDICES
			.read()
			.expect(&format!("Failed to read-lock {} array", stringify!(Shape)));
		if let Some(element) = w.get(index as usize) {
			Ok(f(element))
		} else {
			Err(IndexError::NoElement)
		}
	}

	/// Passes a body to a closure if it exists, otherwise returns an error
	fn read_body<F, R>(index: u32, f: F) -> Result<R, IndexError>
	where
		F: FnOnce(&Body) -> R,
	{
		let w = BODY_INDICES
			.read()
			.expect(&format!("Failed to read-lock {} array", stringify!(Body)));
		if let Some(element) = w.get(index as usize) {
			Ok(f(element))
		} else {
			Err(IndexError::NoElement)
		}
	}

	/// Converts an Index into an u64
	fn raw(self) -> u64 {
		let f = |a, i| ((a as u64) << 32) | i as u64;
		match self {
			// Reserve 0 for invalid indices
			Self::Body(i) => f(1, i),
			Self::Joint(i) => f(2, i),
			Self::Shape(i) => f(3, i),
			Self::Space(i) => f(4, i),
		}
	}

	/// Converts an u64 to an Index. Returns an error if the index is not valid
	fn from_raw(from: u64) -> Result<Self, IndexError> {
		let i = from as u32;
		Ok(match from >> 32 {
			// Reserve 0 for invalid indices
			1 => Self::Body(i),
			2 => Self::Joint(i),
			3 => Self::Shape(i),
			4 => Self::Space(i),
			_ => return Err(IndexError::Invalid),
		})
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
		let index = &$index;
		if let Err(e) = index.$func() {
			use gdnative::prelude::godot_error;
			match e {
				IndexError::Invalid => godot_error!("ID is invalid {:?}", index),
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
				IndexError::Invalid => godot_error!("ID is invalid {:?}", index),
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
		crate::step_all_spaces(delta);
	}
}

fn sync() {}

fn flush_queries() {}

fn free(index: Index) {
	map_or_err!(index, remove);
}

fn get_process_info(info: i32) -> i32 {
	godot_error!("TODO {}", info);
	0
}

fn set_active(active: bool) {
	*ACTIVE.write().expect("Failed to modify ACTIVE") = active;
}
