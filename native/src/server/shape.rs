use super::*;
use core::mem;
use gdnative::core_types::*;
use gdnative::sys;
use rapier3d::geometry::SharedShape;
use rapier3d::math::Point;
use rapier3d::na::{Dynamic, Matrix, Matrix3x1};

#[derive(Debug)]
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
		let p = |x, y, z| Point::new(x, y, z);
		// We need something for triangle shapes I suppose, so have a pyramid
		let pyramid = (
			[
				p(1.0, 0.0, 1.0),
				p(1.0, 0.0, -1.0),
				p(-1.0, 0.0, -1.0),
				p(-1.0, 0.0, 1.0),
				p(0.0, 1.0, 0.0),
			],
			[
				[0, 1, 2],
				[2, 3, 0],
				[0, 1, 4],
				[1, 2, 4],
				[2, 3, 4],
				[3, 0, 4],
			],
		);
		let shape = match r#type {
			// TODO will this work as expected?
			Type::Plane => SharedShape::cuboid(1.0, 0.0, 1.0),
			Type::Ray => {
				// TODO ditto?
				SharedShape::capsule(p(0.0, 1.0, 0.0), p(0.0, 0.0, 0.0), 0.0)
			}
			Type::Sphere => SharedShape::ball(1.0),
			Type::Box => SharedShape::cuboid(1.0, 1.0, 1.0),
			Type::Capsule => SharedShape::capsule(p(0.0, 1.0, 0.0), p(0.0, -1.0, 0.0), 1.0),
			Type::Cylinder => SharedShape::cylinder(1.0, 1.0),
			Type::Convex => SharedShape::convex_mesh(Vec::from(pyramid.0), &pyramid.1)
				.expect("Failed to create convex mesh"),
			// TODO check if this is the correct equivalent
			Type::Concave => SharedShape::trimesh(Vec::from(pyramid.0), Vec::from(pyramid.1)),
			Type::Heightmap => SharedShape::heightfield(
				Matrix::<_, Dynamic, Dynamic, _>::zeros(2, 2),
				Matrix3x1::new(1.0, 1.0, 1.0),
			),
		};
		Self { r#type, shape }
	}

	fn apply_data(&mut self, data: &Variant) -> Result<(), ShapeError> {
		let p = |x, y, z| Point::new(x, y, z);
		fn e<T>(r: Option<T>) -> Result<T, ShapeError> {
			r.ok_or(ShapeError::InvalidData)
		}
		let dict = || e(data.try_to_dictionary());
		let get_f = |d: &Dictionary, k| Ok(e(d.get(k).try_to_f64())? as f32);
		let get_i = |d: &Dictionary, k| e(d.get(k).try_to_i64());
		self.shape = match self.r#type {
			Type::Box => {
				let extents = e(data.try_to_vector3())?;
				SharedShape::cuboid(extents.x, extents.y, extents.z)
			}
			Type::Ray => {
				let data = dict()?;
				let length = get_f(&data, "length")?;
				// TODO Ditto
				// TODO There is a second paramater in the data that is currently unused
				SharedShape::capsule(p(0.0, length, 0.0), p(0.0, 0.0, 0.0), 0.0)
			}
			Type::Capsule => {
				let data = dict()?;
				let height = get_f(&data, "height")?;
				let radius = get_f(&data, "radius")?;
				// TODO Ditto
				SharedShape::capsule(p(0.0, height, 0.0), p(0.0, 0.0, 0.0), radius)
			}
			Type::Cylinder => {
				let data = dict()?;
				let height = get_f(&data, "height")?;
				let radius = get_f(&data, "radius")?;
				SharedShape::cylinder(height, radius)
			}
			Type::Heightmap => {
				let data = dict()?;
				let depth = get_i(&data, "depth")? as usize;
				let width = get_i(&data, "width")? as usize;
				let heights = e(data.get("heights").try_to_float32_array())?;
				let heights = heights.read();
				// TODO there are max_height and min_height, what are they for?
				let mut map = Matrix::<_, Dynamic, Dynamic, _>::zeros(depth, width);
				for x in 0..width {
					for z in 0..depth {
						map[(x, z)] = heights[x * depth + z];
					}
				}
				SharedShape::heightfield(map, na::Vector3::new(1.0, 1.0, 1.0))
			}
			Type::Plane => {
				// TODO figure out a way to implement planes
				let plane = e(data.try_to_plane())?;
				let _ = plane;
				godot_error!("Planes are not implemented yet");
				SharedShape::cuboid(1.0, 0.0, 1.0)
			}
			Type::Sphere => {
				let radius = e(data.try_to_f64())? as f32;
				SharedShape::ball(radius)
			}
			_ => panic!("Handle {:?} - {:?}", self.r#type, data),
		};
		Ok(())
	}

	pub fn shape(&self) -> &SharedShape {
		&self.shape
	}
}

pub fn init(ffi: &mut ffi::FFI) {
	ffi.shape_create(create);
	ffi.shape_set_data(set_data);
}

fn create(shape: i32) -> Option<Index> {
	match Type::new(shape) {
		Ok(shape) => {
			let shape = Shape::new(shape);
			Some(Index::add_shape(shape))
		}
		Err(e) => {
			eprintln!("Invalid shape: {:?}", e);
			None
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
