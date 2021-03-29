#include <stddef.h>
#include "core/error_macros.h"
#include "space_state.h"
#include "server.h"
#include "core/os/memory.h"


/*
void PluggablePhysicsDirectSpaceState::_bind_methods() {

}
*/

int PluggablePhysicsDirectSpaceState::intersect_point(const Vector3 &point, ShapeResult *r_results, int result_max, const Set<RID> &exclude, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas) {
	ERR_FAIL_V_MSG(0, "TODO");
}

bool PluggablePhysicsDirectSpaceState::intersect_ray(const Vector3 &from, const Vector3 &to, PhysicsDirectSpaceState::RayResult &result, const Set<RID> &exclude, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, bool pick_ray) {
	ERR_FAIL_COND_V_MSG(this->server->fn_table.space_intersect_ray == nullptr, false, "Not implemented");

	index_t *e_list = memnew_arr_template<index_t>(exclude.size());
	size_t i = 0;
	for (typename Set<RID>::Element *rid = exclude.front(); rid != nullptr; rid = rid->next()) {
		e_list[i++] = this->server->get_index(rid->get());
	}

	struct physics_ray_info info = {
		.from = from,
		.to = to,
		.exclude = e_list,
		.exclude_count = (size_t)exclude.size(),
		.collision_mask = collision_mask,
		.collide_with_bodies = collide_with_bodies,
		.collide_with_areas = collide_with_areas,
		.pick_ray = pick_ray,
	};
	struct physics_ray_result prr = {};
	bool collided = (*this->server->fn_table.space_intersect_ray)(this->space, &info, &prr);

	// Apparently this function doesn't like it if e_list is null (which can happen if exclude.size() == 0)
	if (e_list != nullptr) {
		memdelete_arr(e_list);
	}

	if (collided) {
		result.position = prr.position;
		result.normal = prr.normal;
		result.rid = this->server->get_rid(prr.id);
		result.collider_id = prr.object_id;
		result.collider = prr.object_id == 0 ? nullptr : ObjectDB::get_instance(prr.object_id);
	}
	return collided;
}

int PluggablePhysicsDirectSpaceState::intersect_shape(const RID &shape, const Transform &xform, float margin, PhysicsDirectSpaceState::ShapeResult *r_results, int result_max, const Set<RID> &exclude, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas) {
	ERR_FAIL_V_MSG(0, "TODO");
}

bool PluggablePhysicsDirectSpaceState::cast_motion(const RID &shape, const Transform &xform, const Vector3 &motion, float margin, float &closest_safe, float &closest_unsafe, const Set<RID> &exclude, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas, PhysicsDirectSpaceState::ShapeRestInfo *r_info) {
	ERR_FAIL_V_MSG(false, "TODO");
}

bool PluggablePhysicsDirectSpaceState::collide_shape(RID shape, const Transform &shape_xform, float margin, Vector3 *r_results, int result_max, int &r_result_count, const Set<RID> &exclude, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas) {
	ERR_FAIL_V_MSG(false, "TODO");
}

bool PluggablePhysicsDirectSpaceState::rest_info(RID shape, const Transform &shape_xform, float margin, PhysicsDirectSpaceState::ShapeRestInfo *r_info, const Set<RID> &exclude, uint32_t collision_mask, bool collide_with_bodies, bool collide_with_areas) {
	ERR_FAIL_V_MSG(false, "TODO");
}

Vector3 PluggablePhysicsDirectSpaceState::get_closest_point_to_object_volume(RID object, const Vector3 point) const {
	ERR_FAIL_V_MSG(Vector3(), "TODO");
}
