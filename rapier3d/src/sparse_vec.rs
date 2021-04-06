pub struct SparseVec<T> {
	elements: Vec<Option<T>>,
	empty_slots: Vec<usize>,
}

impl<T> SparseVec<T> {
	pub fn new() -> Self {
		Self {
			elements: Vec::new(),
			empty_slots: Vec::new(),
		}
	}

	pub fn add(&mut self, element: T) -> usize {
		if let Some(index) = self.empty_slots.pop() {
			self.elements[index] = Some(element);
			index
		} else {
			self.elements.push(Some(element));
			self.elements.len() - 1
		}
	}

	pub fn get(&self, index: usize) -> Option<&T> {
		self.elements.get(index).map(Option::as_ref).flatten()
	}

	pub fn get_mut(&mut self, index: usize) -> Option<&mut T> {
		self.elements.get_mut(index).map(Option::as_mut).flatten()
	}

	pub fn get_mut2(&mut self, index_a: usize, index_b: usize) -> (Option<&mut T>, Option<&mut T>) {
		let (a, b) = if index_a < index_b {
			(index_a, index_b)
		} else {
			(index_b, index_a)
		};
		if a < self.elements.len() {
			let (l, r) = self.elements.split_at_mut(a + 1);
			(l.get_mut(a).map(Option::as_mut).flatten(), r.get_mut(b - a).map(Option::as_mut).flatten())
		} else {
			(None, None)
		}
	}

	pub fn remove(&mut self, index: usize) -> Option<T> {
		if index < self.elements.len() {
			if let Some(element) = self.elements[index].take() {
				self.empty_slots.push(index);
				return Some(element);
			}
		}
		None
	}

	#[allow(dead_code)]
	pub fn iter(&self) -> impl Iterator<Item = &T> {
		self.elements.iter().filter_map(Option::as_ref)
	}

	pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut T> {
		self.elements.iter_mut().filter_map(Option::as_mut)
	}
}
