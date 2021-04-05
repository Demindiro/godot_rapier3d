#include "body_state.h"
#include "space_state.h"
#include "server.h"


Vector3 PluggablePhysicsDirectBodyState::get_total_gravity() const {
	return this->state.gravity;
}

float PluggablePhysicsDirectBodyState::get_total_angular_damp() const {
	return this->state.angular_damp;	
}

float PluggablePhysicsDirectBodyState::get_total_linear_damp() const {
	return this->state.linear_damp;
}

Vector3 PluggablePhysicsDirectBodyState::get_center_of_mass() const {
	return this->state.center_of_mass;
}

Basis PluggablePhysicsDirectBodyState::get_principal_inertia_axes() const {
	return Basis();	
}

float PluggablePhysicsDirectBodyState::get_inverse_mass() const {
	return this->state.inv_mass;
}

Vector3 PluggablePhysicsDirectBodyState::get_inverse_inertia() const {
	return this->state.inv_inertia;
}

Basis PluggablePhysicsDirectBodyState::get_inverse_inertia_tensor() const {
	return this->state.inv_inertia_tensor;
}

void PluggablePhysicsDirectBodyState::set_linear_velocity(const Vector3 &velocity) {	
	Variant v(velocity);
	EXEC_FFI_FN(this->server, body_set_state, this->body, PhysicsServer::BODY_STATE_LINEAR_VELOCITY, &v);
	this->state.linear_velocity = velocity;
}

Vector3 PluggablePhysicsDirectBodyState::get_linear_velocity() const {
	return this->state.linear_velocity;
}

void PluggablePhysicsDirectBodyState::set_angular_velocity(const Vector3 &velocity) {	
	Variant v(velocity);
	EXEC_FFI_FN(this->server, body_set_state, this->body, PhysicsServer::BODY_STATE_ANGULAR_VELOCITY, &v);
	this->state.angular_velocity = velocity;
}

Vector3 PluggablePhysicsDirectBodyState::get_angular_velocity() const {
	return this->state.angular_velocity;
}

void PluggablePhysicsDirectBodyState::set_transform(const Transform &transform) {
	Variant t(transform);
	EXEC_FFI_FN(this->server, body_set_state, this->body, PhysicsServer::BODY_STATE_TRANSFORM, &t);
	this->state.transform = transform;
}

Transform PluggablePhysicsDirectBodyState::get_transform() const {
	return this->state.transform;
}

void PluggablePhysicsDirectBodyState::add_central_force(const Vector3 &force) {
	EXEC_FFI_FN(this->server, body_add_central_force, this->body, &force);
}

void PluggablePhysicsDirectBodyState::add_force(const Vector3 &force, const Vector3 &pos) {
	EXEC_FFI_FN(this->server, body_add_force, this->body, &force, &pos);
};

void PluggablePhysicsDirectBodyState::add_torque(const Vector3 &torque) {
	EXEC_FFI_FN(this->server, body_add_torque, this->body, &torque);
};

void PluggablePhysicsDirectBodyState::apply_central_impulse(const Vector3 &impulse) {
	EXEC_FFI_FN(this->server, body_apply_central_impulse, this->body, &impulse);
};

void PluggablePhysicsDirectBodyState::apply_impulse(const Vector3 &position, const Vector3 &impulse) {
	EXEC_FFI_FN(this->server, body_apply_impulse, this->body, &position, &impulse);
};

void PluggablePhysicsDirectBodyState::apply_torque_impulse(const Vector3 &impulse) {
	EXEC_FFI_FN(this->server, body_apply_torque_impulse, this->body, &impulse);
};

void PluggablePhysicsDirectBodyState::set_sleep_state(bool enable) {
	Variant e(enable);
	EXEC_FFI_FN(this->server, body_set_state, this->body, PhysicsServer::BODY_STATE_SLEEPING, &e);
};

bool PluggablePhysicsDirectBodyState::is_sleeping() const {
	return this->state.sleeping;
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
	// Don't need to do anything here
};

PhysicsDirectSpaceState *PluggablePhysicsDirectBodyState::get_space_state() {
	this->space_state_singleton->space = this->state.space;
	return this->space_state_singleton;
};
