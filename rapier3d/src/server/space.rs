use super::*;
use crate::space::{BodyOrAreaIndex, Space};

pub fn init(ffi: &mut ffi::FFI) {
	ffi!(ffi, space_create, create);
	ffi!(ffi, space_is_active, is_active);
	ffi!(ffi, space_set_active, set_active);
	ffi!(ffi, space_intersect_ray, intersect_ray);
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
						Index::Body(i) => { exclude_bodies.as_mut().map(|v| v.push(i)); }
						Index::Area(i) => { exclude_areas.as_mut().map(|v| v.push(i)); }
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
						BodyOrAreaIndex::Body(body) => {
							(body.map(|body| body.object_id()).expect("Invalid body"), Index::Body(body))
						}
						BodyOrAreaIndex::Area(area) => {
							(area.map(|area| area.object_id()).expect("Invalid area"), Index::Area(area))
						}
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
