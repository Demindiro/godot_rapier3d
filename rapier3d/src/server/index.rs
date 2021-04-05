#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BodyIndex(u32, u16);
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct JointIndex(u32, u16);
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ShapeIndex(u32, u16);
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SpaceIndex(u32, u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Index {
	Body(BodyIndex),
	Joint(JointIndex),
	Shape(ShapeIndex),
	Space(SpaceIndex),
}

#[derive(Debug)]
pub struct InvalidIndex;

impl BodyIndex {
	/// Creates a new BodyIndex with the given Index
	pub fn new(i: u32, generation: u16) -> Self {
		Self(i, generation)
	}

	/// Returns the inner index value
	pub fn index(&self) -> u32 {
		self.0
	}

	/// Returns the generation of this BodyIndex
	pub fn generation(&self) -> u16 {
		self.1
	}
}

impl JointIndex {
	/// Creates a new BodyIndex with the given Index
	pub fn new(i: u32, generation: u16) -> Self {
		Self(i, generation)
	}

	/// Returns the inner index value
	pub fn index(&self) -> u32 {
		self.0
	}

	/// Returns the generation of this JointIndex
	pub fn generation(&self) -> u16 {
		self.1
	}
}

impl ShapeIndex {
	/// Creates a new BodyIndex with the given Index
	pub fn new(i: u32, generation: u16) -> Self {
		Self(i, generation)
	}

	/// Returns the inner index value
	pub fn index(&self) -> u32 {
		self.0
	}

	/// Returns the generation of this ShapeIndex
	pub fn generation(&self) -> u16 {
		self.1
	}
}

impl SpaceIndex {
	/// Creates a new BodyIndex with the given Index
	pub fn new(i: u32, generation: u16) -> Self {
		Self(i, generation)
	}

	/// Returns the inner index value
	pub fn index(&self) -> u32 {
		self.0
	}

	/// Returns the generation of this SpaceIndex
	pub fn generation(&self) -> u16 {
		self.1
	}
}

impl Index {
	/// A value that represents an invalid index as a u64.
	#[allow(unused)]
	const INVALID_RAW: u64 = 0;

	/// Returns the given index as a `BodyIndex` if it is one
	pub fn as_body(&self) -> Option<BodyIndex> {
		if let Self::Body(i) = self {
			Some(*i)
		} else {
			None
		}
	}

	/// Returns the given index as a `JointIndex` if it is one
	pub fn as_joint(&self) -> Option<JointIndex> {
		if let Self::Joint(i) = self {
			Some(*i)
		} else {
			None
		}
	}

	/// Returns the given index as a `ShapeIndex` if it is one
	pub fn as_shape(&self) -> Option<ShapeIndex> {
		if let Self::Shape(i) = self {
			Some(*i)
		} else {
			None
		}
	}

	/// Returns the given index as a `SpaceIndex` if it is one
	pub fn as_space(&self) -> Option<SpaceIndex> {
		if let Self::Space(i) = self {
			Some(*i)
		} else {
			None
		}
	}

	/// Converts an Index into an u64
	pub fn raw(self) -> u64 {
		let f = |a, i, gen| ((a as u64) << 48) | ((gen as u64) << 32) | i as u64;
		match self {
			// Reserve 0 for invalid indices
			Self::Body(i) => f(1, i.index(), i.generation()),
			Self::Joint(i) => f(2, i.index(), i.generation()),
			Self::Shape(i) => f(3, i.index(), i.generation()),
			Self::Space(i) => f(4, i.index(), i.generation()),
		}
	}

	/// Converts an u64 to an Index. Returns an error if the index is not valid
	pub fn from_raw(from: u64) -> Result<Self, InvalidIndex> {
		let i = from as u32;
		let gen = (from >> 32) as u16;
		Ok(match from >> 48 {
			// Reserve 0 for invalid indices
			1 => Self::Body(BodyIndex::new(i, gen)),
			2 => Self::Joint(JointIndex::new(i, gen)),
			3 => Self::Shape(ShapeIndex::new(i, gen)),
			4 => Self::Space(SpaceIndex::new(i, gen)),
			_ => return Err(InvalidIndex),
		})
	}
}
