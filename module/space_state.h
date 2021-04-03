#ifndef PLUGGABLE_PHYSICS_DIRECT_SPACE_STATE
#define PLUGGABLE_PHYSICS_DIRECT_SPACE_STATE

#include "index.h"
#include "api.gen.h"
#include "typedef.h"
#include "servers/physics_server.h"

class PluggablePhysicsServer;

class PluggablePhysicsDirectSpaceState : public PhysicsDirectSpaceState {
	GDCLASS(PluggablePhysicsDirectSpaceState, PhysicsDirectSpaceState);

	friend class PluggablePhysicsServer;
	friend class PluggablePhysicsDirectBodyState;

	PluggablePhysicsServer *server;
	index_t space;

	_FORCE_INLINE_ PluggablePhysicsDirectSpaceState(PluggablePhysicsServer *p_server) : PhysicsDirectSpaceState() {
		this->server = p_server;
		this->space = 0;
	}

protected:
	//static void _bind_methods();

public:

	virtual int intersect_point(const Vector3 &p_point, ShapeResult *r_results, int p_result_max, const Set<RID> &p_exclude = Set<RID>(), uint32_t p_collision_mask = 0xFFFFFFFF, bool p_collide_with_bodies = true, bool p_collide_with_areas = false);

	virtual bool intersect_ray(const Vector3 &p_from, const Vector3 &p_to, RayResult &r_result, const Set<RID> &p_exclude = Set<RID>(), uint32_t p_collision_mask = 0xFFFFFFFF, bool p_collide_with_bodies = true, bool p_collide_with_areas = false, bool p_pick_ray = false);

	virtual int intersect_shape(const RID &p_shape, const Transform &p_xform, float p_margin, ShapeResult *r_results, int p_result_max, const Set<RID> &p_exclude = Set<RID>(), uint32_t p_collision_mask = 0xFFFFFFFF, bool p_collide_with_bodies = true, bool p_collide_with_areas = false);

	virtual bool cast_motion(const RID &p_shape, const Transform &p_xform, const Vector3 &p_motion, float p_margin, float &p_closest_safe, float &p_closest_unsafe, const Set<RID> &p_exclude = Set<RID>(), uint32_t p_collision_mask = 0xFFFFFFFF, bool p_collide_with_bodies = true, bool p_collide_with_areas = false, ShapeRestInfo *r_info = NULL);

	virtual bool collide_shape(RID p_shape, const Transform &p_shape_xform, float p_margin, Vector3 *r_results, int p_result_max, int &r_result_count, const Set<RID> &p_exclude = Set<RID>(), uint32_t p_collision_mask = 0xFFFFFFFF, bool p_collide_with_bodies = true, bool p_collide_with_areas = false);

	virtual bool rest_info(RID p_shape, const Transform &p_shape_xform, float p_margin, ShapeRestInfo *r_info, const Set<RID> &p_exclude = Set<RID>(), uint32_t p_collision_mask = 0xFFFFFFFF, bool p_collide_with_bodies = true, bool p_collide_with_areas = false);

	virtual Vector3 get_closest_point_to_object_volume(RID p_object, const Vector3 p_point) const;
};

#endif
