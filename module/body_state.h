#ifndef PLUGGABLE_PHYSICS_BODY_STATE
#define PLUGGABLE_PHYSICS_BODY_STATE

#include "servers/physics_server.h"
#include "typedef.h"
#include "index.h"
#include "api.gen.h"

class PluggablePhysicsServer;
class PluggablePhysicsDirectSpaceState;

class PluggablePhysicsDirectBodyState : public PhysicsDirectBodyState {
	GDCLASS(PluggablePhysicsDirectBodyState, PhysicsDirectBodyState);

	friend class PluggablePhysicsServer;
	_FORCE_INLINE_ PluggablePhysicsDirectBodyState(PluggablePhysicsServer *p_server) {
		this->server = p_server;
	}
	_FORCE_INLINE_ ~PluggablePhysicsDirectBodyState() {}

	struct physics_body_state state;
	mutable struct physics_body_contact contact;
	PluggablePhysicsServer *server;
	PluggablePhysicsDirectSpaceState *space_state_singleton;
	index_t body;
	real_t delta;
	mutable uint32_t contact_index;

	_FORCE_INLINE_ const struct physics_body_contact *_select_contact(int id) const;

public:
	virtual Vector3 get_total_gravity() const;
	virtual float get_total_angular_damp() const;
	virtual float get_total_linear_damp() const;

	virtual Vector3 get_center_of_mass() const;
	virtual Basis get_principal_inertia_axes() const;
	virtual float get_inverse_mass() const; // get the mass
	virtual Vector3 get_inverse_inertia() const; // get density of this body space
	virtual Basis get_inverse_inertia_tensor() const; // get density of this body space

	virtual void set_linear_velocity(const Vector3 &p_velocity);
	virtual Vector3 get_linear_velocity() const;

	virtual void set_angular_velocity(const Vector3 &p_velocity);
	virtual Vector3 get_angular_velocity() const;

	virtual void set_transform(const Transform &p_transform);
	virtual Transform get_transform() const;

	virtual void add_central_force(const Vector3 &p_force);
	virtual void add_force(const Vector3 &p_force, const Vector3 &p_pos);
	virtual void add_torque(const Vector3 &p_torque);
	virtual void apply_central_impulse(const Vector3 &p_j);
	virtual void apply_impulse(const Vector3 &p_pos, const Vector3 &p_j);
	virtual void apply_torque_impulse(const Vector3 &p_j);

	virtual void set_sleep_state(bool p_enable);
	virtual bool is_sleeping() const;

	virtual int get_contact_count() const;

	virtual Vector3 get_contact_local_position(int p_contact_idx) const;
	virtual Vector3 get_contact_local_normal(int p_contact_idx) const;
	virtual float get_contact_impulse(int p_contact_idx) const;
	virtual int get_contact_local_shape(int p_contact_idx) const;

	virtual RID get_contact_collider(int p_contact_idx) const;
	virtual Vector3 get_contact_collider_position(int p_contact_idx) const;
	virtual ObjectID get_contact_collider_id(int p_contact_idx) const;
	virtual Object *get_contact_collider_object(int p_contact_idx) const;
	virtual int get_contact_collider_shape(int p_contact_idx) const;
	virtual Vector3 get_contact_collider_velocity_at_position(int p_contact_idx) const;

	virtual real_t get_step() const;
	virtual void integrate_forces();

	virtual PhysicsDirectSpaceState *get_space_state();
};

#endif
