use super::*;
use core::mem;
use gdnative::core_types::*;
use gdnative::sys;
use rapier3d::geometry::SharedShape;
use rapier3d::math::Point;
use rapier3d::na::{Dynamic, Matrix, Matrix3x1};

enum Type {
	Plane,
	Ray,
	Sphere,
	Box,
	Capsule,
	Cylinder,
	Convex,
	Concave,
	Heightmap,
	// Add custom types below
}

#[derive(Debug)]
enum TypeError {
	InvalidType,
}

impl Type {
	fn new(shape: i32) -> Result<Self, TypeError> {
		Ok(match shape {
			0 => Type::Plane,
			1 => Type::Ray,
			2 => Type::Sphere,
			3 => Type::Box,
			4 => Type::Capsule,
			5 => Type::Cylinder,
			6 => Type::Convex,
			7 => Type::Concave,
			8 => Type::Heightmap,
			_ => return Err(TypeError::InvalidType),
		})
	}
}

pub struct Shape {
	r#type: Type,
	shape: SharedShape,
}

#[derive(Debug)]
enum ShapeError {
	InvalidData,
}

impl Shape {
	fn new(r#type: Type) -> Self {
		let shape = match r#type {
			// TODO will this work as expected?
			Type::Plane => SharedShape::cuboid(1.0, 0.0, 1.0),
			Type::Ray => {
				// TODO ditto?
				SharedShape::capsule(Point::new(0.0, 1.0, 0.0), Point::new(0.0, -1.0, 0.0), 0.0)
			}
			Type::Sphere => SharedShape::ball(1.0),
			Type::Box => SharedShape::cuboid(1.0, 1.0, 1.0),
			Type::Capsule => {
				SharedShape::capsule(Point::new(0.0, 1.0, 0.0), Point::new(0.0, -1.0, 0.0), 1.0)
			}
			Type::Cylinder => SharedShape::cylinder(1.0, 1.0),
			// TODO do this panic-free
			Type::Convex => {
				SharedShape::convex_mesh(Vec::new(), &[]).expect("Failed to create convex mesh")
			}
			// TODO check if this is the correct equivalent
			Type::Concave => SharedShape::trimesh(Vec::new(), Vec::new()),
			Type::Heightmap => SharedShape::heightfield(
				Matrix::<_, Dynamic, Dynamic, _>::zeros(1, 1),
				Matrix3x1::new(1.0, 1.0, 1.0),
			),
		};
		Self { r#type, shape }
	}

	fn apply_data(&mut self, data: &Variant) -> Result<(), ShapeError> {
		match self.r#type {
			Type::Box => {
				if let Some(extents) = data.try_to_vector3() {
					self.shape = SharedShape::cuboid(extents.x, extents.y, extents.z);
					Ok(())
				} else {
					Err(ShapeError::InvalidData)
				}
			}
			_ => todo!(),
		}
	}

	pub fn shape(&self) -> &SharedShape {
		&self.shape
	}
}

pub fn init(ffi: &mut ffi::FFI) {
	ffi.shape_create(create);
	ffi.shape_set_data(set_data);
}

fn create(shape: i32) -> *const Index {
	match Type::new(shape) {
		Ok(shape) => {
			let shape = Shape::new(shape);
			let w = Index::add_shape(shape).raw();
			w
		}
		Err(e) => {
			eprintln!("Invalid shape: {:?}", e);
			std::ptr::null()
		}
	}
}

fn set_data(shape: Index, data: &Variant) {
	map_or_err!(shape, map_shape_mut, |shape| {
		if let Err(e) = shape.apply_data(&data) {
			eprintln!("Failed to apply data: {:?}", e);
		}
	});
}
