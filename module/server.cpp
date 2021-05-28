#include "core/error_macros.h"
#include "server.h"
#include "api.gen.h"
#include "body_state.h"
#include "core/project_settings.h"
#include "core/os/os.h"
#include "core/io/resource_loader.h"
// TODO figure out how to include the gdnative header properly
//#include "modules/gdnative/gdnative.h"
#include "gdnative.h"


PluggablePhysicsServer::PluggablePhysicsServer() {
	memset(&this->fn_table, 0, sizeof(this->fn_table));
	this->body_state_singleton = memnew(PluggablePhysicsDirectBodyState(this));
	this->space_state_singleton = memnew(PluggablePhysicsDirectSpaceState(this));
	this->body_state_singleton->space_state_singleton = this->space_state_singleton;
}

PluggablePhysicsServer::~PluggablePhysicsServer() {
	this->library->terminate();
}

void PluggablePhysicsServer::_bind_methods() {

}

void PluggablePhysicsServer::area_set_monitor_callback(RID area, Object* receiver, const StringName &method) {
	index_t id = this->get_index(area);
	ERR_FAIL_COND_MSG(id == 0, "Invalid RID");
	AreaCallback callback(receiver, method);
	this->area_body_monitor_callbacks.set(id, callback);
}

void PluggablePhysicsServer::area_set_area_monitor_callback(RID area, Object* receiver, const StringName &method) {
	index_t id = this->get_index(area);
	ERR_FAIL_COND_MSG(id == 0, "Invalid RID");
	AreaCallback callback(receiver, method);
	this->area_area_monitor_callbacks.set(id, callback);
}

void PluggablePhysicsServer::body_get_collision_exceptions(RID body, List<RID> *list) {
	ERR_FAIL_MSG("TODO");
}

void PluggablePhysicsServer::init() {
    Variant lib_path_variant = ProjectSettings::get_singleton()->get_setting("physics/3d/custom_library_path");
    String lib_path = String(lib_path_variant);

    if (lib_path != "") {
		Error err;
		RES lib = ResourceLoader::load(lib_path, "", false, &err);
		ERR_FAIL_COND_MSG(err, "Failed to load physics server library");

		void *handle;
		String init_symbol = "gdphysics_init";
		this->library.instance();
		this->library->set_library(lib);
		this->library->initialize();
		err = this->library->get_symbol(init_symbol, handle);
		ERR_FAIL_COND_MSG(err, "Failed to get init handle");

		// SAFETY: the callee must have the exact same signature
		void (*init_func)(struct fn_table *) = reinterpret_cast<void (*)(struct fn_table *)>(handle);
		init_func(&this->fn_table);
    }
}

void PluggablePhysicsServer::step(float delta) {

	EXEC_FFI_FN(this, step, delta);

	const index_t *id = nullptr;
	Vector<index_t> invalid_callbacks;

	// Execute force integration callbacks
	ERR_FAIL_COND_MSG(this->fn_table.body_get_direct_state == nullptr, "Not implemented");
	while ((id = this->body_force_integration_callbacks.next(id)) != nullptr) {

		(*this->fn_table.body_get_direct_state)(*id, &this->body_state_singleton->state);

		this->body_state_singleton->delta = delta;
		this->body_state_singleton->body = *id;

		Callback *callback = this->body_force_integration_callbacks.getptr(*id);
		Object *object = ObjectDB::get_instance(callback->object_id);

		if (object == nullptr) {
			invalid_callbacks.push_back(*id);
		} else {
			Variant variant_body_direct = this->body_state_singleton;
			const Variant *argv[2] = { &variant_body_direct, &callback->userdata };
			int argc = (callback->userdata.get_type() == Variant::NIL) ? 1 : 2;

			Variant::CallError error = {};
			object->call(callback->method, argv, argc, error);
		}
	}

	// Remove any invalid callbacks
	for (int i = 0; i < invalid_callbacks.size(); i++) {
		this->body_force_integration_callbacks.erase(invalid_callbacks[i]);
	}
	invalid_callbacks.resize(0);

	// Execute area <-> body monitor callbacks
	ERR_FAIL_COND_MSG(this->fn_table.area_get_body_event == nullptr, "Not implemented");
	while ((id = this->area_body_monitor_callbacks.next(id)) != nullptr) {
		
		struct physics_area_monitor_event event;

		AreaCallback *callback = this->area_body_monitor_callbacks.getptr(*id);
		Object *object = ObjectDB::get_instance(callback->object_id);

		if (object == nullptr) {
			invalid_callbacks.push_back(*id);
		} else {
			while ((*this->fn_table.area_get_body_event)(*id, &event)) {
				Variant event_type = event.added ? 0 : 1;
				Variant rid = this->reverse_rids.get(event.id);
				Variant object_id = event.object_id;
				// Bullets skip this, so shall we I suppose (less work for me)
				Variant body_shape = 0;
				Variant area_shape = 0;

				const Variant *argv[5] = { &event_type, &rid, &object_id, &body_shape, &area_shape };

				Variant::CallError error = {};
				object->call(callback->method, argv, 5, error);
			}
		}
	}

	// Remove any invalid callbacks
	for (int i = 0; i < invalid_callbacks.size(); i++) {
		this->area_body_monitor_callbacks.erase(invalid_callbacks[i]);
	}
	invalid_callbacks.resize(0);

	// Execute area <-> area monitor callbacks
	ERR_FAIL_COND_MSG(this->fn_table.area_get_area_event == nullptr, "Not implemented");
	while ((id = this->area_area_monitor_callbacks.next(id)) != nullptr) {
		
		struct physics_area_monitor_event event;

		AreaCallback *callback = this->area_area_monitor_callbacks.getptr(*id);
		Object *object = ObjectDB::get_instance(callback->object_id);

		if (object == nullptr) {
			invalid_callbacks.push_back(*id);
		} else {
			while ((*this->fn_table.area_get_area_event)(*id, &event)) {
				Variant event_type = event.added ? 0 : 1;
				Variant rid = this->reverse_rids.get(event.id);
				Variant object_id = event.object_id;
				// Bullets skips this, so shall we I suppose (less work for me)
				Variant body_shape = 0;
				Variant area_shape = 0;

				const Variant *argv[5] = { &event_type, &rid, &object_id, &body_shape, &area_shape };

				Variant::CallError error = {};
				object->call(callback->method, argv, 5, error);
			}
		}
	}

	// Remove any invalid callbacks
	for (int i = 0; i < invalid_callbacks.size(); i++) {
		this->area_body_monitor_callbacks.erase(invalid_callbacks[i]);
	}
}

PhysicsDirectBodyState *PluggablePhysicsServer::body_get_direct_state(RID rid) {
	ERR_FAIL_COND_V_MSG(this->fn_table.body_get_direct_state == nullptr, nullptr, "Not implemented");
	index_t id = this->get_index(rid);
	ERR_FAIL_COND_V_MSG(id == 0, nullptr, "Invalid RID");
	(*this->fn_table.body_get_direct_state)(id, &this->body_state_singleton->state);
	return this->body_state_singleton;
}

void PluggablePhysicsServer::body_set_force_integration_callback(RID body, Object *receiver, const StringName &method, const Variant &userdata) {
	index_t id = this->get_index(body);
	Callback callback(receiver, method, userdata);
	this->body_force_integration_callbacks.set(id, callback);
}

void PluggablePhysicsServer::free(RID rid) {
	ERR_FAIL_COND_MSG(this->fn_table.free == nullptr, "Not implemented");
	index_t id = this->get_index(rid);
	ERR_FAIL_COND_MSG(id == 0, "Invalid RID");
	this->rids.free(rid);
	this->reverse_rids.erase(id);
	this->body_force_integration_callbacks.erase(id);
	this->area_body_monitor_callbacks.erase(id);
	this->area_area_monitor_callbacks.erase(id);
	(*this->fn_table.free)(id);
}

PhysicsDirectSpaceState *PluggablePhysicsServer::space_get_direct_state(RID space) {
	index_t id = this->get_index(space);
	ERR_FAIL_COND_V_MSG(id == 0, nullptr, "Space doesn't exist");
	this->space_state_singleton->space = id;
	return this->space_state_singleton;
}

Vector<Vector3> PluggablePhysicsServer::space_get_contacts(RID space) const {
	ERR_FAIL_COND_V_MSG(this->fn_table.space_get_contact_count == nullptr, Vector<Vector3>(), "Not implemented");
	ERR_FAIL_COND_V_MSG(this->fn_table.space_get_contact == nullptr, Vector<Vector3>(), "Not implemented");

	index_t id = this->get_index(space);
	ERR_FAIL_COND_V_MSG(id == 0, Vector<Vector3>(), "Invalid RID");

	size_t count = (*this->fn_table.space_get_contact_count)(id);
	Vector<Vector3> contacts;
	contacts.resize(count);
	Vector3 *contacts_ptr = contacts.ptrw();
	for (size_t i = 0; i < count; i++) {
		(*this->fn_table.space_get_contact)(id, i, &contacts_ptr[i]);
	}
	return contacts;
}

void PluggablePhysicsServer::soft_body_update_visual_server(RID soft_body, SoftBodyVisualServerHandler *handler) {
	ERR_FAIL_MSG("TODO");
}

void PluggablePhysicsServer::soft_body_get_collision_exceptions(RID soft_body, List<RID> *list) {
	ERR_FAIL_MSG("TODO");
}

void PluggablePhysicsServer::soft_body_set_mesh(RID soft_body, const REF &mesh) {
	ERR_FAIL_MSG("TODO");
}
