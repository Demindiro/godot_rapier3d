#include "body_state.h"
#include "space_state.h"
#include "server.h"


Vector3 PluggablePhysicsDirectBodyState::get_total_gravity() const {
	return Vector3();	
};
float PluggablePhysicsDirectBodyState::get_total_angular_damp() const {
	return 0.0;	
};
float PluggablePhysicsDirectBodyState::get_total_linear_damp() const {
	return 0.0;	
};

Vector3 PluggablePhysicsDirectBodyState::get_center_of_mass() const {
	return this->state.center_of_mass;
};
Basis PluggablePhysicsDirectBodyState::get_principal_inertia_axes() const {
	return Basis();	
};
float PluggablePhysicsDirectBodyState::get_inverse_mass() const {
	return this->state.inv_mass;
}
Vector3 PluggablePhysicsDirectBodyState::get_inverse_inertia() const {
	return Vector3();
}
Basis PluggablePhysicsDirectBodyState::get_inverse_inertia_tensor() const {
	return Basis();
}

void PluggablePhysicsDirectBodyState::set_linear_velocity(const Vector3 &p_velocity) {	
	ERR_FAIL_MSG("TODO");
}
Vector3 PluggablePhysicsDirectBodyState::get_linear_velocity() const {
	return this->state.linear_velocity;
}

void PluggablePhysicsDirectBodyState::set_angular_velocity(const Vector3 &p_velocity) {	
	ERR_FAIL_MSG("TODO");
}
Vector3 PluggablePhysicsDirectBodyState::get_angular_velocity() const {
	return this->state.angular_velocity;
}

void PluggablePhysicsDirectBodyState::set_transform(const Transform &p_transform) {
}
Transform PluggablePhysicsDirectBodyState::get_transform() const {
	return this->state.transform;
}

void PluggablePhysicsDirectBodyState::add_central_force(const Vector3 &p_force) {
	WARN_PRINT("AAA");
}
void PluggablePhysicsDirectBodyState::add_force(const Vector3 &p_force, const Vector3 &p_pos) {
	WARN_PRINT("BBB");
};
void PluggablePhysicsDirectBodyState::add_torque(const Vector3 &p_torque) {
	WARN_PRINT("CCC");
};
void PluggablePhysicsDirectBodyState::apply_central_impulse(const Vector3 &p_j) {
	WARN_PRINT("DDD");
};
void PluggablePhysicsDirectBodyState::apply_impulse(const Vector3 &position, const Vector3 &impulse) {
	ERR_FAIL_COND_MSG(this->server->fn_table.body_apply_impulse == nullptr, "Not implemented");
	(*this->server->fn_table.body_apply_impulse)(this->body, &position, &impulse);
};
void PluggablePhysicsDirectBodyState::apply_torque_impulse(const Vector3 &p_j) {};

void PluggablePhysicsDirectBodyState::set_sleep_state(bool p_enable) {};
bool PluggablePhysicsDirectBodyState::is_sleeping() const {
	return false;
};

int PluggablePhysicsDirectBodyState::get_contact_count() const {
	return 0;
}

Vector3 PluggablePhysicsDirectBodyState::get_contact_local_position(int p_contact_idx) const {
	return Vector3();
}
Vector3 PluggablePhysicsDirectBodyState::get_contact_local_normal(int p_contact_idx) const {
	return Vector3();	
}
float PluggablePhysicsDirectBodyState::get_contact_impulse(int p_contact_idx) const {
	return 0.0;	
}
int PluggablePhysicsDirectBodyState::get_contact_local_shape(int p_contact_idx) const {
	return 0;	
};

RID PluggablePhysicsDirectBodyState::get_contact_collider(int p_contact_idx) const {
	return RID();	
};
Vector3 PluggablePhysicsDirectBodyState::get_contact_collider_position(int p_contact_idx) const {
	return Vector3();
}
ObjectID PluggablePhysicsDirectBodyState::get_contact_collider_id(int p_contact_idx) const {
	return ObjectID();
}
Object *PluggablePhysicsDirectBodyState::get_contact_collider_object(int p_contact_idx) const {
	return nullptr;
}
int PluggablePhysicsDirectBodyState::get_contact_collider_shape(int p_contact_idx) const {
	return 0;	
};
Vector3 PluggablePhysicsDirectBodyState::get_contact_collider_velocity_at_position(int p_contact_idx) const {
	return Vector3();	
};

real_t PluggablePhysicsDirectBodyState::get_step() const {
	return this->delta;
};
void PluggablePhysicsDirectBodyState::integrate_forces() {
	WARN_PRINT("EEE");
};

PhysicsDirectSpaceState *PluggablePhysicsDirectBodyState::get_space_state() {
	this->space_state_singleton->space = this->state.space;
	return this->space_state_singleton;
};
