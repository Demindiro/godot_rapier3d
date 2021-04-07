use super::*;
use crate::util::*;
use gdnative::core_types::*;
use rapier3d::geometry::{Collider, ColliderBuilder, SharedShape};
use rapier3d::math::Point;
use rapier3d::na::{DMatrix, Dynamic, Isometry3, Matrix, Matrix3x1, Point3};

#[derive(Copy, Clone, Debug)]
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
	IncompleteTriangle,
	ConvexNotManifold,
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
				let mut map = DMatrix::zeros(width, depth);
				for x in 0..width {
					for z in 0..depth {
						map[(x, z)] = heights[x * depth + z];
					}
				}
				SharedShape::heightfield(
					map,
					na::Vector3::new(depth as f32 - 1.0, 1.0, width as f32 - 1.0),
				)
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
			Type::Concave | Type::Convex => {
				let array = e(data.try_to_vector3_array())?;
				let array = array.read();
				if array.len() % 3 != 0 {
					return Err(ShapeError::IncompleteTriangle);
				}
				let mut verts = Vec::with_capacity(array.len());
				for v in array.iter() {
					verts.push(p(v.x, v.y, v.z));
				}
				let mut indices = Vec::with_capacity(array.len());
				// It may be worth to perform some sort of deduplication to reduce the
				// amount of vertices. For now, the simple, dumb way will do.
				for i in 0..array.len() / 3 {
					let i = i * 3;
					indices.push([i as u32, (i + 1) as u32, (i + 2) as u32]);
				}
				if let Type::Concave = self.r#type {
					SharedShape::trimesh(verts, indices)
				} else {
					SharedShape::convex_mesh(verts, &indices)
						.ok_or(ShapeError::ConvexNotManifold)?
				}
			}
		};
		Ok(())
	}

	pub fn shape(&self) -> &SharedShape {
		&self.shape
	}

	/// Do a best effort to scale a collider appropriately
	pub fn scaled(&self, scale: Vector3) -> SharedShape {
		let scale = vec_gd_to_na(scale);
		// TODO figure out the exact way each collider is scaled in Godot for consistency
		// The colliders where the scale is not certain are left empty for now
		match self.r#type {
			Type::Heightmap => {
				let hf = self.shape.as_heightfield().unwrap();
				SharedShape::heightfield(hf.heights().clone(), hf.scale().component_mul(&scale))
			}
			Type::Convex => {
				let cp = self.shape.as_convex_polyhedron().unwrap();
				let cp_points = cp.points();
				let mut verts = Vec::with_capacity(cp_points.len());
				for v in cp_points.iter() {
					verts.push(Point3::new(v.x * scale.x, v.y * scale.y, v.z * scale.z));
				}
				SharedShape::convex_hull(&verts[..]).expect("Failed to scale convex hull")
			}
			Type::Concave => {
				let tm = self.shape.as_trimesh().unwrap();
				let tm_verts = tm.vertices();
				let mut verts = Vec::with_capacity(tm_verts.len());
				for v in tm_verts.iter() {
					verts.push(Point3::new(v.x * scale.x, v.y * scale.y, v.z * scale.z));
				}
				SharedShape::trimesh(verts, tm.indices().iter().copied().collect())
			}
			_ => self.shape.clone(),
		}
	}

	/// Frees this shape, removing it from any attached rigidbodies
	pub fn free(self) {
		// FIXME we need to track attached bodies to remove the corresponding shapes
		godot_error!("TODO free shape");
	}

	/// Creates a new collider based on the given position and scale
	pub fn build(&self, position: Isometry3<f32>, scale: Vector3, sensor: bool) -> Collider {
		let shape_scale = position.rotation * vec_gd_to_na(scale);
		let shape_scale = vec_gd_to_na(scale).component_mul(&shape_scale);
		let shape_scale = vec_na_to_gd(shape_scale);
		ColliderBuilder::new(self.scaled(shape_scale))
			.position_wrt_parent(position)
			.sensor(sensor)
			.build()
	}
}

pub fn init(ffi: &mut ffi::FFI) {
	ffi.shape_create(create);
	ffi.shape_get_margin(|_| 0.0);
	ffi.shape_set_margin(|_, _| ());
	ffi.shape_set_data(set_data);
}

/// Frees the given shape, removing it from any attached rigidbodies
pub fn free(shape: Shape) {
	shape.free();
}

fn create(shape: i32) -> Option<Index> {
	match Type::new(shape) {
		Ok(shape) => {
			let shape = Shape::new(shape);
			Some(Index::Shape(ShapeIndex::add(shape)))
		}
		Err(e) => {
			godot_error!("Invalid shape: {:?}", e);
			None
		}
	}
}

fn set_data(shape: Index, data: &Variant) {
	map_or_err!(shape, map_shape_mut, |shape, _| {
		if let Err(e) = shape.apply_data(&data) {
			godot_error!("Failed to apply data: {:?}", e);
		}
	});
}
