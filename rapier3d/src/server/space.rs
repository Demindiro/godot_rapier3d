use super::*;
use crate::space::Space;

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
			let mut exclude = Vec::with_capacity(exclude_raw.len());
			for &e in exclude_raw {
				if let Ok(i) = Index::from_raw(e) {
					if let Index::Body(i) = i {
						exclude.push(i);
					} else {
						godot_error!("One of the indices does not point to a body");
						// TODO should we continue on regardless?
					}
				} else {
					godot_error!("One of the indices is invalid");
					// TODO ditto?
				}
			}
			space
				.cast_ray(
					info.from(),
					info.to(),
					info.collision_mask(),
					&exclude[..],
				)
				.map(|res| {
					let object_id = res.body.map(|body| body.object_id()).expect("Invalid body");
					result.set_position(res.position);
					result.set_normal(res.normal);
					result.set_object_id(object_id);
					result.set_index(Index::Body(res.body));
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
