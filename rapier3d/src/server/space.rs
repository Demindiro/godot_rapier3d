use super::*;
use crate::space::{BodyOrAreaIndex, Space};
use gdnative::core_types::Vector3;

pub fn init(ffi: &mut ffi::FFI) {
	ffi!(ffi, space_create, create);
	ffi!(ffi, space_get_contact, get_contact);
	ffi!(ffi, space_get_contact_count, get_contact_count);
	ffi!(ffi, space_intersect_ray, intersect_ray);
	ffi!(ffi, space_intersect_shape, intersect_shape);
	ffi!(ffi, space_is_active, is_active);
	ffi!(ffi, space_set_active, set_active);
	ffi!(ffi, space_set_debug_contacts, set_debug_contacts);
}

pub fn free(_space: Space) {}

fn create() -> Option<Index> {
	let index = SpaceIndex::add(Space::new());
	index.map_mut(|space| space.set_index(index)).unwrap();
	Some(Index::Space(index))
}

fn intersect_ray(
	space: Index,
	info: &ffi::PhysicsRayInfo,
	result: &mut ffi::PhysicsRayResult,
) -> bool {
	space
		.map_space_mut(|space, _| {
			let exclude_raw = info.exclude_raw();
			let mut exclude_bodies = if info.collide_with_bodies() {
				Some(Vec::with_capacity(exclude_raw.len()))
			} else {
				None
			};
			let mut exclude_areas = if info.collide_with_areas() {
				Some(Vec::with_capacity(exclude_raw.len()))
			} else {
				None
			};
			for &e in exclude_raw {
				if let Ok(i) = Index::from_raw(e) {
					match i {
						Index::Body(i) => {
							exclude_bodies.as_mut().map(|v| v.push(i));
						}
						Index::Area(i) => {
							exclude_areas.as_mut().map(|v| v.push(i));
						}
						_ => godot_error!("One of the indices does not point to a body"),
					}
				} else {
					godot_error!("One of the indices is invalid");
				}
			}
			space
				.cast_ray(
					info.from(),
					info.to(),
					info.collision_mask(),
					exclude_bodies.as_deref(),
					exclude_areas.as_deref(),
					info.pick_ray(),
				)
				.map(|res| {
					let (object_id, index) = match res.index {
						BodyOrAreaIndex::Body(body) => (
							body.map(|body| body.object_id()).expect("Invalid body"),
							Index::Body(body),
						),
						BodyOrAreaIndex::Area(area) => (
							area.map(|area| area.object_id()).expect("Invalid area"),
							Index::Area(area),
						),
					};
					result.set_position(res.position);
					result.set_normal(res.normal);
					result.set_object_id(object_id);
					result.set_index(index);
					result.set_shape(res.shape);
				})
				.is_some()
		})
		.unwrap_or_else(|_| {
			godot_error!("Invalid space index");
			false
		})
}

fn intersect_shape(
	space: Index,
	info: &ffi::PhysicsShapeInfo,
	results: *mut ffi::PhysicsShapeResult,
	max_results: usize,
) -> usize {
	// SAFETY: We'll have to trust the Godot side that the results array is large enough.
	let results = unsafe { core::slice::from_raw_parts_mut(results, max_results) };
	map_or_err!(space, map_space_mut, |space, _| {
		let exclude_raw = info.exclude_raw();
		let mut exclude_bodies = if info.collide_with_bodies() {
			Some(Vec::with_capacity(exclude_raw.len()))
		} else {
			None
		};
		let mut exclude_areas = if info.collide_with_areas() {
			Some(Vec::with_capacity(exclude_raw.len()))
		} else {
			None
		};
		for &e in exclude_raw {
			if let Ok(i) = Index::from_raw(e) {
				match i {
					Index::Body(i) => {
						exclude_bodies.as_mut().map(|v| v.push(i));
					}
					Index::Area(i) => {
						exclude_areas.as_mut().map(|v| v.push(i));
					}
					_ => godot_error!("One of the indices does not point to a body"),
				}
			} else {
				godot_error!("One of the indices is invalid");
			}
		}

		info.shape()
			.map_err(|e| godot_error!("Invalid shape index: {:?}", e))
			.map(|shape| {
				shape
					.map(|shape| {
						let mut total = 0;
						for r in space.intersect_shape(
							shape.shape().as_ref(),
							info.transform(),
							info.collision_mask(),
							exclude_bodies.as_deref(),
							exclude_areas.as_deref(),
							max_results,
						) {
							let (object_id, index) = match r.index {
								BodyOrAreaIndex::Body(body) => (
									body.map(|body| body.object_id()).expect("Invalid body"),
									Index::Body(body),
								),
								BodyOrAreaIndex::Area(area) => (
									area.map(|area| area.object_id()).expect("Invalid area"),
									Index::Area(area),
								),
							};
							results[total].set_index(index);
							results[total].set_object_id(object_id);
							results[total].set_shape(r.shape);
							total += 1;
						}
						total
					})
					.map_err(|_| godot_error!("Invalid shape"))
					.unwrap_or(0)
			})
			.unwrap_or(0)
	})
	.unwrap_or(0)
}

fn set_active(space: Index, active: bool) {
	map_or_err!(space, map_space_mut, |space, _| space.enabled = active);
}

fn is_active(space: Index) -> bool {
	let result = space.map_space(|space, _| space.enabled);
	if let Ok(active) = result {
		active
	} else {
		godot_error!("RID does not point to a space");
		false
	}
}

fn set_debug_contacts(space: Index, contacts: i32) {
	map_or_err!(space, map_space_mut, |space, _| space
		.set_debug_contact_count(contacts as usize));
}

fn get_contact_count(space: Index) -> i32 {
	map_or_err!(space, map_space_mut, |space, _| space
		.debug_contacts()
		.len())
	.unwrap_or(0) as i32
}

fn get_contact(space: Index, contact: i32, out: &mut Vector3) {
	*out = map_or_err!(space, map_space_mut, |space, _| {
		if let Some(contact) = space.debug_contacts().get(contact as usize) {
			*contact
		} else {
			godot_error!("Invalid contact index");
			Vector3::zero()
		}
	})
	.unwrap_or(Vector3::zero());
}

/// Extra methods exposed through the "call" function.
mod call {
	use super::super::call;
	use super::*;
	use crate::util::*;
	use ffi::{PhysicsCallError, VariantType};
	use gdnative::prelude::*;

	/// Return *all* colliders that intersect with a ray. A `VariantArray` will be returned with a
	/// number of `Dictionary` entries. Each entry has the following fields:
	///
	/// * `position`: The location where the ray hit.
	///
	/// * `normal`: The normal at the intersection point.
	///
	/// * `rid`: The RID of the parent object.
	///
	/// * `object_id`: The `ObjectID` of the parent, which is a Body or an `Area´.
	///
	/// * `shape`: The index of the shape in the body/area.
	///
	/// * `time_of_impact`: The time of impact, where
	///   `position = from + (to - from) * time_of_impact`
	///
	/// There is no order guarantee.
	///
	/// `solid` indicates whether it should detect colliders that encapsulate the origin of the
	/// ray.
	///
	/// `exclude` is a `VariantArray` of `Rid`s pointing to bodies that should be ignored.
	pub fn intersections_with_ray(args: &[&Variant]) -> call::Result {
		call_check_arg_count!(args in 2..9)?;
		let space = call_get_arg!(args[0] => Rid)?;
		let from = call_get_arg!(args[1] => Vector3)?;
		let to = call_get_arg!(args[2] => Vector3)?;
		let solid = call_get_arg!(args[3] => bool || false)?;
		let mask = call_get_arg!(args[4] => u32 || 0xffff_ffff)?;
		let max_results = call_get_arg!(args[5] => i32 || 32)?;
		let exclude_bodies = !call_get_arg!(args[6] => bool || false)?;
		let exclude_areas = !call_get_arg!(args[7] => bool || false)?;
		let exclude = call_get_arg!(args[8] => VariantArray || VariantArray::new().into_shared())?;
		if let Ok(space) = super::get_index(space) {
			map_or_err!(space, map_space_mut, |space, _| {
				let array = VariantArray::new();
				let mut exclude_bodies =
					exclude_bodies.then(|| Vec::with_capacity(exclude.len() as usize));
				let mut exclude_areas =
					exclude_areas.then(|| Vec::with_capacity(exclude.len() as usize));
				for (i, e) in exclude.iter().enumerate() {
					if let Some(e) = e.try_to_rid() {
						if let Ok(e) = super::get_index(e) {
							if let Some(e) = e.as_body() {
								exclude_bodies.as_mut().map(|v| v.push(e));
							} else if let Some(e) = e.as_area() {
								exclude_areas.as_mut().map(|v| v.push(e));
							} else {
								godot_error!("RID {:?} is not a body", e);
							}
						} else {
							godot_error!("RID {:?} is invalid", e);
						}
					} else {
						godot_error!("Element {} is not a RID", i);
					}
				}
				// Creating the Godot strings before hand should be slightly more efficient
				// (less allocations, more ref counting). Should probably be done as a global
				// lazy static or similar but w/e.
				let position_key = "position".to_variant();
				let normal_key = "normal".to_variant();
				let rid_key = "rid".to_variant();
				let object_id_key = "object_id".to_variant();
				let shape_key = "shape".to_variant();
				let toi_key = "time_of_impact".to_variant();

				let bodies = BodyIndex::read_all();
				let areas = AreaIndex::read_all();
				space.intersections_with_ray(
					from,
					to - from,
					solid,
					mask,
					exclude_bodies.as_ref().map(|v| &v[..]),
					exclude_areas.as_ref().map(|v| &v[..]),
					|index, shape_index, ri| {
						let dict = Dictionary::new();
						let pos = (to - from).normalize() * ri.toi + from;
						dict.insert(position_key.clone(), pos);
						dict.insert(normal_key.clone(), vec_na_to_gd(ri.normal));
						let (object_id, index) = match index {
							BodyOrAreaIndex::Body(body) => (
								bodies
									.get(body.into())
									.map(|b| b.object_id())
									.expect("Invalid body"),
								Index::Body(body),
							),
							BodyOrAreaIndex::Area(area) => (
								areas
									.get(area.into())
									.map(|a| a.object_id())
									.expect("Invalid area"),
								Index::Area(area),
							),
						};
						let rid = super::get_rid(index.into());
						dict.insert(
							object_id_key.clone(),
							object_id.map(ObjectID::get).unwrap_or(0),
						);
						dict.insert(shape_key.clone(), shape_index);
						dict.insert(rid_key.clone(), rid);
						dict.insert(toi_key.clone(), ri.toi);
						array.push(dict);
						array.len() < max_results
					},
				);
				Ok(array.owned_to_variant())
			})
			.unwrap_or(Ok(Variant::new()))
		} else {
			godot_error!("Invalid index");
			Ok(Variant::new())
		}
	}
}

pub(super) use call::*;
