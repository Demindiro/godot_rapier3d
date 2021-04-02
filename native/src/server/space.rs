use super::*;
use rapier3d::geometry::{InteractionGroups, Ray};
use rapier3d::na::Point3;

pub fn init(ffi: &mut ffi::FFI) {
	ffi.space_create(create);
	ffi.space_is_active(is_active);
	ffi.space_set_active(set_active);
	ffi.space_intersect_ray(intersect_ray);
}

fn create() -> Option<Index> {
	let index = Index::add_space(crate::create_space());
	index
		.map_space(|space, _| space.set_index(Some(index)))
		.unwrap()
		.unwrap();
	Some(index)
}

fn intersect_ray(
	space: Index,
	info: &ffi::PhysicsRayInfo,
	result: &mut ffi::PhysicsRayResult,
) -> bool {
	// TODO account for excluded colliders
	// There are two ways to approach this
	// - Make a separate ColliderSet without the excluded colliders (very likely very slow)
	// - Use `intersections_with_ray`
	// It probably makes sense to use `cast_ray_and_get_normal` if there are no excluded colliders,
	// it is likely at least as fast as `intersections_with_ray`.
	let mut collided = false;
	map_or_err!(space, map_space, |space, _| space.modify(|space| {
		space.update_query_pipeline();
		let from = info.from();
		let dir = info.to() - from;
		let (dir, max_toi) = (dir.normalize(), dir.length());
		let intersection = space.query_pipeline.cast_ray_and_get_normal(
			&space.colliders,
			&Ray::new(Point3::new(from.x, from.y, from.z), vec_gd_to_na(dir)),
			max_toi,
			// TODO what is the solid parameter for?
			false,
			// FIXME document the fact that bitmasks are limited to 16 bits (which honestly should be plenty
			// for all games, but people may still be using the upper 4 bits of the 20(?) available)
			InteractionGroups::new(u16::MAX, info.collision_mask() as u16),
		);
		if let Some((collider, intersection)) = intersection {
			//let point = dir.mul_add(intersection.toi, from); // Faster & more accurate
			let point = dir * intersection.toi + from; // MulAdd not implemented for the above :(
			result.set_position(point);
			result.set_normal(vec_na_to_gd(intersection.normal));
			let collider = space.colliders.get(collider).unwrap();
			if let Some((body, shape)) = body::Body::get_shape_userdata(collider) {
				let mut object_id = None;
				map_or_err!(Index::Body(body), map_body, |body, _| {
					object_id = body.object_id()
				});
				result.set_object_id(object_id);
				result.set_index(Index::Body(body));
				result.set_shape(shape);
			}
			collided = true;
		}
	}));
	collided
}

fn set_active(space: Index, active: bool) {
	map_or_err!(space, map_space, |space, _| space
		.modify(|space| space.enabled = active)
		.expect("Invalid space"));
}

fn is_active(space: Index) -> bool {
	let result =
		space.map_space(|space, _| space.read(|space| space.enabled).expect("Invalid space"));
	if let Ok(active) = result {
		active
	} else {
		godot_error!("RID does not point to a space");
		false
	}
}
