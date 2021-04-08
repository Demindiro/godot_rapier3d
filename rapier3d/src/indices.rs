//! Based on https://github.com/fitzgen/generational-arena/blob/master/src/lib.rs but optimized for
//! smaller indices (u32 for index, u16 for generation) so that it's suitable for FFI.

use core::mem;

pub struct Indices<T> {
	elements: Vec<Entry<T>>,
	free_slot: Option<u32>,
	generation: u16,
}

pub struct Index {
	index: u32,
	generation: u16,
}

enum Entry<T> {
	Free { next: Option<u32> },
	Occupied { item: T, generation: u16 },
}

impl<T> Indices<T> {
	pub fn new() -> Self {
		Self {
			elements: Vec::new(),
			free_slot: None,
			generation: 0,
		}
	}

	pub fn add(&mut self, item: T) -> Index {
		let generation = self.generation;
		let entry = Entry::Occupied { item, generation };
		self.generation = self.generation.wrapping_add(1);
		if let Some(index) = self.free_slot {
			let e = mem::replace(&mut self.elements[index as usize], entry);
			if let Entry::Free { next } = e {
				self.free_slot = next;
			} else {
				panic!("Entry was occupied");
			}
			Index::new(index, generation)
		} else {
			let index = self.elements.len() as u32;
			self.elements.push(entry);
			Index::new(index, generation)
		}
	}

	pub fn remove(&mut self, index: Index) -> Option<T> {
		if let Some(e) = self.elements.get_mut(index.index as usize) {
			if let Entry::Occupied { generation, .. } = e {
				if *generation == index.generation {
					let entry = Entry::Free {
						next: self.free_slot,
					};
					self.free_slot = Some(index.index);
					if let Entry::Occupied { item, .. } = mem::replace(e, entry) {
						return Some(item);
					} else {
						unreachable!();
					}
				}
			}
		}
		None
	}

	pub fn get(&self, index: Index) -> Option<&T> {
		let (i, g) = index.split();
		if let Some(Entry::Occupied { item, generation }) = self.elements.get(i as usize) {
			if *generation == g {
				return Some(item);
			}
		}
		None
	}

	pub fn get_mut(&mut self, index: Index) -> Option<&mut T> {
		let (i, g) = index.split();
		if let Some(Entry::Occupied { item, generation }) = self.elements.get_mut(i as usize) {
			if *generation == g {
				return Some(item);
			}
		}
		None
	}

	pub fn get2_mut(&mut self, index_a: Index, index_b: Index) -> (Option<&mut T>, Option<&mut T>) {
		let (a, b) = (index_a.split(), index_b.split());
		let (a, b, swapped) = if a.0 < b.0 {
			(a, b, false)
		} else {
			(b, a, true)
		};
		let ((ai, ag), (bi, bg)) = ((a.0 as usize, a.1), (b.0 as usize, b.1));
		if ai < self.elements.len() {
			let (l, r) = self.elements.split_at_mut(ai + 1);
			let a = if let Entry::Occupied { item, generation } = &mut l[ai] {
				if *generation == ag {
					Some(item)
				} else {
					None
				}
			} else {
				None
			};
			let b = if let Some(Entry::Occupied { item, generation }) = r.get_mut(bi - ai - 1) {
				if *generation == bg {
					Some(item)
				} else {
					None
				}
			} else {
				None
			};
			if swapped {
				(b, a)
			} else {
				(a, b)
			}
		} else {
			(None, None)
		}
	}

	pub fn iter_mut(&mut self) -> impl Iterator<Item = (Index, &mut T)> {
		self.elements.iter_mut().enumerate().filter_map(|(i, e)| {
			if let Entry::Occupied { item, generation } = e {
				Some((Index::new(i as u32, *generation), item))
			} else {
				None
			}
		})
	}
}

impl Index {
	pub fn new(index: u32, generation: u16) -> Self {
		Self { index, generation }
	}

	pub fn index(&self) -> u32 {
		self.index
	}

	pub fn generation(&self) -> u16 {
		self.generation
	}

	fn split(self) -> (u32, u16) {
		(self.index, self.generation)
	}
}

#[cfg(test)]
mod test {
	use super::*;

	#[test]
	fn get2_mut() {
		let mut inds = Indices::new();
		let a = inds.add("foo");
		let b = inds.add("bar");
		let (a, b) = inds.get2_mut(a, b);
		assert_eq!(a, Some(&mut "foo"));
		assert_eq!(b, Some(&mut "bar"));

		let mut inds = Indices::new();
		let a = inds.add("bar");
		let b = inds.add("foo");
		let (a, b) = inds.get2_mut(a, b);
		assert_eq!(a, Some(&mut "bar"));
		assert_eq!(b, Some(&mut "foo"));
	}
}
