use crate::indices;

pub const AREA_ID: u8 = 1;
pub const BODY_ID: u8 = 2;
pub const JOINT_ID: u8 = 3;
pub const SHAPE_ID: u8 = 4;
pub const SPACE_ID: u8 = 5;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Index {
	Area(AreaIndex),
	Body(BodyIndex),
	Joint(JointIndex),
	Shape(ShapeIndex),
	Space(SpaceIndex),
}

#[derive(Debug)]
pub struct InvalidIndex;

impl AreaIndex {
	/// Creates a new AreaIndex with the given Index
	pub fn new(i: u32, generation: u16) -> Self {
		Self(i, generation)
	}

	/// Returns the inner index value
	pub fn index(&self) -> u32 {
		self.0
	}

	/// Returns the generation of this AreaIndex
	pub fn generation(&self) -> u16 {
		self.1
	}
}

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
	/// Creates a new JointIndex with the given Index
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
	/// Creates a new ShapeIndex with the given Index
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
	/// Creates a new SpaceIndex with the given Index
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

	/// Converts an Index into an u64
	pub fn raw(self) -> u64 {
		let f = |a, i, gen| ((a as u64) << 48) | ((gen as u64) << 32) | i as u64;
		match self {
			// Reserve 0 for invalid indices
			Self::Area(i) => f(AREA_ID, i.index(), i.generation()),
			Self::Body(i) => f(BODY_ID, i.index(), i.generation()),
			Self::Joint(i) => f(JOINT_ID, i.index(), i.generation()),
			Self::Shape(i) => f(SHAPE_ID, i.index(), i.generation()),
			Self::Space(i) => f(SPACE_ID, i.index(), i.generation()),
		}
	}

	/// Converts an u64 to an Index. Returns an error if the index is not valid
	pub fn from_raw(from: u64) -> Result<Self, InvalidIndex> {
		let i = from as u32;
		let gen = (from >> 32) as u16;
		Ok(match (from >> 48) as u8 {
			// Reserve 0 for invalid indices
			AREA_ID => Self::Area(AreaIndex::new(i, gen)),
			BODY_ID => Self::Body(BodyIndex::new(i, gen)),
			JOINT_ID => Self::Joint(JointIndex::new(i, gen)),
			SHAPE_ID => Self::Shape(ShapeIndex::new(i, gen)),
			SPACE_ID => Self::Space(SpaceIndex::new(i, gen)),
			_ => return Err(InvalidIndex),
		})
	}
}

macro_rules! generate {
	($name:ident, $index:ident, $as:ident) => {
		#[derive(Clone, Copy, Debug, PartialEq, Eq)]
		pub struct $index(u32, u16);

		impl Index {
			/// Returns the given index as a [`$index`] if it is one
			pub fn $as(self) -> Option<$index> {
				if let Self::$name(i) = self {
					Some(i)
				} else {
					None
				}
			}
		}

		impl From<$index> for indices::Index {
			/// Converts a [`$index`] into a [`Index`](crate::indices::Index)
			fn from(index: $index) -> Self {
				Self::new(index.0, index.1)
			}
		}

		impl From<&$index> for indices::Index {
			fn from(index: &$index) -> Self {
				(*index).into()
			}
		}

		impl From<indices::Index> for $index {
			/// Converts a [`$index`] into a [`Index`](crate::indices::Index)
			fn from(index: indices::Index) -> Self {
				Self(index.index(), index.generation())
			}
		}
	};
}

generate!(Area, AreaIndex, as_area);
generate!(Body, BodyIndex, as_body);
generate!(Joint, JointIndex, as_joint);
generate!(Shape, ShapeIndex, as_shape);
generate!(Space, SpaceIndex, as_space);
