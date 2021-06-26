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
