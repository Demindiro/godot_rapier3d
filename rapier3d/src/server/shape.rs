use super::*;
use crate::util::*;
use core::mem;
use core::convert::TryFrom;
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
	index: Option<ShapeIndex>,
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
		let index = None;
		Self {
			r#type,
			shape,
			index,
		}
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
				let mut verts = Vec::with_capacity(array.len());
				for v in array.iter() {
					verts.push(p(v.x, v.y, v.z));
				}
				if let Type::Concave = self.r#type {
					if array.len() % 3 != 0 {
						return Err(ShapeError::IncompleteTriangle);
					}
					let mut indices = Vec::with_capacity(array.len());
					// It may be worth to perform some sort of deduplication to reduce the
					// amount of vertices. For now, the simple, dumb way will do.
					for i in 0..array.len() / 3 {
						let i = i * 3;
						indices.push([i as u32, (i + 1) as u32, (i + 2) as u32]);
					}
					SharedShape::trimesh(verts, indices)
				} else {
					SharedShape::convex_hull(&verts).ok_or(ShapeError::ConvexNotManifold)?
				}
			}
		};
		Ok(())
	}

	fn data(&self) -> Variant {
		match self.r#type {
			Type::Box => {
				let shape = self.shape.as_cuboid().unwrap();
				vec_na_to_gd(shape.half_extents).owned_to_variant()
			}
			Type::Ray => {
				let shape = self.shape.as_capsule().unwrap();
				let dict = Dictionary::new();
				dict.insert("length", shape.segment.a.y.owned_to_variant());
				dict.owned_to_variant()
			}
			Type::Capsule => {
				let shape = self.shape.as_capsule().unwrap();
				let dict = Dictionary::new();
				dict.insert("length", shape.segment.a.y.owned_to_variant());
				dict.insert("radius", shape.radius.owned_to_variant());
				dict.owned_to_variant()
			}
			Type::Cylinder => {
				let shape = self.shape.as_cylinder().unwrap();
				let dict = Dictionary::new();
				dict.insert("height", shape.half_height);
				dict.insert("radius", shape.radius);
				dict.owned_to_variant()
			}
			Type::Heightmap => {
				let shape = self.shape.as_heightfield().unwrap();
				let depth = shape.nrows();
				let width = shape.ncols();
				let mut heights = TypedArray::<f32>::new();
				heights.resize((depth * width).try_into().unwrap());
				let mut wr_heights = heights.write();
				for x in 0..width {
					for z in 0..depth {
						wr_heights[x * depth + z] = shape.heights()[(x, z)];
					}
				}
				mem::drop(wr_heights);
				let dict = Dictionary::new();
				dict.insert("depth", depth.owned_to_variant());
				dict.insert("width", width.owned_to_variant());
				dict.insert("heights", heights.owned_to_variant());
				dict.owned_to_variant()
			}
			Type::Plane => {
				let _shape = self.shape.as_cuboid().unwrap();
				let plane = Plane::new(Vector3::new(0.0, 0.0, 0.0), 1.0);
				plane.owned_to_variant()
			}
			Type::Sphere => {
				let shape = self.shape.as_ball().unwrap();
				shape.radius.owned_to_variant()
			}
			Type::Convex => {
				let shape = self.shape.as_convex_polyhedron().unwrap();
				let mut array = TypedArray::<Vector3>::new();
				array.resize(shape.points().len().try_into().unwrap());
				let mut wr_array = array.write();
				for (s, d) in shape.points().iter().zip(wr_array.iter_mut()) {
					*d = vec_na_to_gd(s.coords);
				}
				mem::drop(wr_array);
				array.owned_to_variant()
			}
			Type::Concave => {
				let shape = self.shape.as_trimesh().unwrap();
				let mut array = TypedArray::<Vector3>::new();
				array.resize(shape.flat_indices().len().try_into().unwrap());
				let mut wr_array = array.write();
				for (s, d) in shape.flat_indices().iter().zip(wr_array.iter_mut()) {
					*d = vec_na_to_gd(shape.vertices()[usize::try_from(*s).unwrap()].coords);
				}
				mem::drop(wr_array);
				array.owned_to_variant()
			}
		}
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

	/// Creates a new shape based on the given position and scale
	pub fn build_shape(&self, position: Isometry3<f32>, scale: Vector3) -> SharedShape {
		let shape_scale = position.rotation * vec_gd_to_na(scale);
		let shape_scale = vec_gd_to_na(scale).component_mul(&shape_scale);
		let shape_scale = vec_na_to_gd(shape_scale);
		self.scaled(shape_scale)
	}

	/// Creates a new collider based on the given position and scale
	pub fn build(&self, position: Isometry3<f32>, scale: Vector3, sensor: bool) -> Collider {
		ColliderBuilder::new(self.build_shape(position, scale))
			.position_wrt_parent(position)
			.sensor(sensor)
			.build()
	}

	/// Sets the index of this shape
	///
	/// # Panics
	///
	/// Panics if the index is already set
	pub fn set_index(&mut self, index: ShapeIndex) {
		assert_eq!(self.index, None, "Index is already set");
		self.index = Some(index);
	}

	/// Returns the index of this shape
	///
	/// # Panics
	///
	/// Panics if the index isn't set
	pub fn index(&self) -> ShapeIndex {
		self.index.expect("Index isn't set")
	}
}

pub fn init(ffi: &mut ffi::FFI) {
	ffi!(ffi, shape_create, create);
	ffi!(ffi, shape_get_margin, |_| 0.0);
	ffi!(ffi, shape_set_margin, |_, _| ());
	ffi!(ffi, shape_get_data, get_data);
	ffi!(ffi, shape_set_data, set_data);
}

/// Frees the given shape, removing it from any attached rigidbodies
pub fn free(shape: Shape) {
	shape.free();
}

fn create(shape: i32) -> Option<Index> {
	match Type::new(shape) {
		Ok(shape) => {
			let shape = Shape::new(shape);
			let index = ShapeIndex::add(shape);
			index.map_mut(|s| s.set_index(index)).unwrap();
			Some(Index::Shape(index))
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

fn get_data(shape: Index) -> gdnative::sys::godot_variant {
	map_or_err!(shape, map_shape, |shape, _| {
		shape.data()
	}).unwrap_or(Variant::new()).forget()
}
