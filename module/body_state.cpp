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
	return this->state.contact_count;
}

Vector3 PluggablePhysicsDirectBodyState::get_contact_local_position(int id) const {
	return this->_select_contact(id)->local_position;
}

Vector3 PluggablePhysicsDirectBodyState::get_contact_local_normal(int id) const {
	return this->_select_contact(id)->local_normal;
}

float PluggablePhysicsDirectBodyState::get_contact_impulse(int id) const {
	return this->_select_contact(id)->impulse;
}

int PluggablePhysicsDirectBodyState::get_contact_local_shape(int id) const {
	return this->_select_contact(id)->local_shape;
};

RID PluggablePhysicsDirectBodyState::get_contact_collider(int id) const {
	return this->server->get_rid(this->_select_contact(id)->index);
};

Vector3 PluggablePhysicsDirectBodyState::get_contact_collider_position(int id) const {
	return this->_select_contact(id)->position;
}

ObjectID PluggablePhysicsDirectBodyState::get_contact_collider_id(int id) const {
	return this->_select_contact(id)->object_id;
}

Object *PluggablePhysicsDirectBodyState::get_contact_collider_object(int id) const {
	int oid = this->_select_contact(id)->object_id;
	return oid != 0 ? ObjectDB::get_instance(oid) : nullptr;
}

int PluggablePhysicsDirectBodyState::get_contact_collider_shape(int id) const {
	return this->_select_contact(id)->shape;
};

Vector3 PluggablePhysicsDirectBodyState::get_contact_collider_velocity_at_position(int id) const {
	return this->_select_contact(id)->velocity;
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

_FORCE_INLINE_ const struct physics_body_contact *PluggablePhysicsDirectBodyState::_select_contact(int id) const {
	uint32_t uid = (uint32_t)id;
	if (this->contact_index != uid) {
		if (uid < this->state.contact_count) {
			EXEC_V_FFI_FN(&this->contact, this->server, body_get_contact, this->body, uid, &this->contact);
			this->contact_index = uid;
		} else {
			this->contact = {};
			this->contact_index = (uint32_t)-1;
		}
	}
	return &this->contact;
}

