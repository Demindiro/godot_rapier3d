use super::*;
use rapier3d::geometry::{InteractionGroups, Ray};
use rapier3d::na::Point3;

pub fn init(ffi: &mut ffi::FFI) {
	ffi.space_create(create);
	ffi.space_set_active(set_active);
	ffi.space_intersect_ray(intersect_ray);
}

fn create() -> Option<Index> {
	Some(Index::add_space(crate::create_space()))
}

fn intersect_ray(space: Index, info: &ffi::PhysicsRayInfo, result: &mut ffi::PhysicsRayResult) -> bool {
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
			result.set_position(dir * intersection.toi + from);
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

fn set_active(index: Index, active: bool) {
	println!(
		"Then God actived the World, even though it is a no-op {:?} {}",
		index, active
	);
}
