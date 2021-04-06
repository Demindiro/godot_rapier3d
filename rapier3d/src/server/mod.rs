#[macro_use]
mod ffi;
mod area;
mod body;
mod index;
mod joint;
mod shape;
mod space;

use crate::area::Area;
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
use std::sync::{RwLock, RwLockReadGuard, RwLockWriteGuard};

lazy_static::lazy_static! {
	static ref ACTIVE: RwLock<bool> = RwLock::new(true);
	static ref AREA_INDICES: RwLock<Indices<Area>> = RwLock::new(Indices::new());
	static ref BODY_INDICES: RwLock<Indices<Body>> = RwLock::new(Indices::new());
	static ref JOINT_INDICES: RwLock<Indices<Joint>> = RwLock::new(Indices::new());
	static ref SHAPE_INDICES: RwLock<Indices<Shape>> = RwLock::new(Indices::new());
	static ref SPACE_INDICES: RwLock<Indices<Space>> = RwLock::new(Indices::new());
}

pub struct Indices<T> {
	elements: SparseVec<(T, u16)>,
	generation: u16,
}

pub enum Instance<A, L> {
	Attached(A, SpaceIndex),
	Loose(L),
}

#[derive(Debug)]
pub enum IndexError {
	WrongType,
	NoElement,
}

#[derive(Clone, Copy)]
pub struct ObjectID(NonZeroU32);

pub trait MapIndex<T> {
	fn map<F, R>(&self, f: F) -> Result<R, IndexError>
	where
		F: FnOnce(&T) -> R;

	fn map_mut<F, R>(&self, f: F) -> Result<R, IndexError>
	where
		F: FnOnce(&mut T) -> R;

	fn add(element: T) -> Self;

	fn remove(self) -> Result<T, IndexError>;

	fn read_all() -> RwLockReadGuard<'static, Indices<T>>;

	fn write_all() -> RwLockWriteGuard<'static, Indices<T>>;
}

trait MapEnumIndex {
	fn map_area<F, R>(&self, f: F) -> Result<R, IndexError>
	where
		F: FnOnce(&Area, AreaIndex) -> R;

	fn map_area_mut<F, R>(&self, f: F) -> Result<R, IndexError>
	where
		F: FnOnce(&mut Area, AreaIndex) -> R;

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
				let w = Self::read_all();
				if let Some(element) = w.get(self.index(), self.generation()) {
					Ok(f(element))
				} else {
					Err(IndexError::NoElement)
				}
			}

			fn map_mut<F, R>(&self, f: F) -> Result<R, IndexError>
			where
				F: FnOnce(&mut $variant) -> R,
			{
				let mut w = Self::write_all();
				if let Some(element) = w.get_mut(self.index(), self.generation()) {
					Ok(f(element))
				} else {
					Err(IndexError::NoElement)
				}
			}

			fn add(element: $variant) -> Self {
				let mut w = Self::write_all();
				let (i, g) = w.add(element);
				Self::new(i, g)
			}

			fn remove(self) -> Result<$variant, IndexError> {
				let mut w = Self::write_all();
				if let Some(element) = w.remove(self.index(), self.generation()) {
					Ok(element)
				} else {
					Err(IndexError::NoElement)
				}
			}

			fn read_all() -> RwLockReadGuard<'static, Indices<$variant>> {
				const MSG: &str = concat!("Failed to read-lock ", stringify!($variant), " array");
				$array.read().expect(MSG)
			}

			fn write_all() -> RwLockWriteGuard<'static, Indices<$variant>> {
				const MSG: &str = concat!("Failed to write-lock ", stringify!($variant), " array");
				$array.write().expect(MSG)
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

impl<T> Indices<T> {
	fn new() -> Self {
		Self {
			elements: SparseVec::new(),
			generation: 0,
		}
	}

	fn add(&mut self, element: T) -> (u32, u16) {
		let g = self.generation;
		self.generation += 1;
		let i = self.elements.add((element, g)) as u32;
		(i, g)
	}

	fn remove(&mut self, index: u32, generation: u16) -> Option<T> {
		if let Some((_, g)) = self.elements.get(index as usize) {
			if *g == generation {
				return self.elements.remove(index as usize).map(|v| v.0);
			}
		}
		None
	}

	pub fn get(&self, index: u32, generation: u16) -> Option<&T> {
		if let Some((e, g)) = self.elements.get(index as usize) {
			if *g == generation {
				return Some(e);
			}
		}
		None
	}

	pub fn get_mut(&mut self, index: u32, generation: u16) -> Option<&mut T> {
		if let Some((e, g)) = self.elements.get_mut(index as usize) {
			if *g == generation {
				return Some(e);
			}
		}
		None
	}

	pub fn get_mut2(&mut self, index_a: u32, generation_a: u16, index_b: u32, generation_b: u16) -> (Option<&mut T>, Option<&mut T>) {
		let (a, b) = self.elements.get_mut2(index_a as usize, index_b as usize);
		let a = a.and_then(|(e, g)| if *g == generation_a { Some(e) } else { None });
		let b = b.and_then(|(e, g)| if *g == generation_b { Some(e) } else { None });
		(a, b)
	}

	fn iter_mut(&mut self) -> impl Iterator<Item = &mut T> {
		self.elements.iter_mut().map(|v| &mut v.0)
	}
}

impl<A, L> Instance<A, L> {
	pub fn as_attached(&self) -> Option<(&A, SpaceIndex)> {
		if let Instance::Attached(a, i) = self {
			Some((a, *i))
		} else {
			None
		}
	}
}

map_index!(AREA_INDICES, Area, AreaIndex,);
map_index!(BODY_INDICES, Body, BodyIndex,);
map_index!(JOINT_INDICES, Joint, JointIndex,);
map_index!(SHAPE_INDICES, Shape, ShapeIndex,);
map_index!(SPACE_INDICES, Space, SpaceIndex,);

impl MapEnumIndex for Index {
	map_enum_index!(map_area, map_area_mut, AREA_INDICES, Area, AreaIndex,);
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
	($index:expr, $func:ident, $closure:expr) => {
		match $index.$func($closure) {
			Ok(v) => Some(v),
			Err(e) => {
				use gdnative::prelude::godot_error;
				match e {
					IndexError::WrongType => godot_error!("ID is of wrong type {:?}", $index),
					IndexError::NoElement => godot_error!("No element at ID {:?}", $index),
				}
				None
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
	area::init(ffi);
	body::init(ffi);
	joint::init(ffi);
	space::init(ffi);
	shape::init(ffi);
}

gdphysics_init!(init);

fn server_init() {}

fn step(delta: f32) {
	if *ACTIVE.read().expect("Failed to check ACTIVE") {
		for area in AreaIndex::write_all().iter_mut() {
			area.clear_events();
		}
		for space in SpaceIndex::write_all().iter_mut() {
			space.step(delta);
		}
	}
}

fn sync() {}

fn flush_queries() {}

fn free(index: Index) {
	let rem = || -> Result<(), IndexError> {
		Ok(match index {
			Index::Area(index) => area::free(index.remove()?),
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
