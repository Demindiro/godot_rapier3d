#include "core/error_macros.h"
#include "server.h"
#include "fn_table.h"
#include "body_state.h"
#include "core/project_settings.h"
#include "core/os/os.h"
#include "core/io/resource_loader.h"
// TODO figure out how to include the gdnative header properly
//#include "modules/gdnative/gdnative.h"
#include "gdnative.h"




PluggablePhysicsServer::PluggablePhysicsServer() {
	zeromem(&this->fn_table, sizeof(this->fn_table));
}
PluggablePhysicsServer::~PluggablePhysicsServer() {
	// TODO
	/*
    if (this->library != nullptr) {
		DEREF(this->library);
        ERR_FAIL_COND_MSG(err, "Failed to close physics server library");
    }
	*/
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

void PluggablePhysicsServer::_bind_methods() {
}

void PluggablePhysicsServer::step(float delta) {

	ERR_FAIL_COND_MSG(this->fn_table.step == nullptr, "Not implemented");
	(*this->fn_table.step)(delta);

	ERR_FAIL_COND_MSG(this->fn_table.body_get_direct_state == nullptr, "Not implemented");

	const index_t *id = nullptr;
	while ((id = this->callbacks.next(id)) != nullptr) {

		(*this->fn_table.body_get_direct_state)(*id, &this->body_state_singleton.state);
		Variant variant_body_direct = &this->body_state_singleton;

		Callback *callback = this->callbacks.getptr(*id);
		const Variant *argv[2] = { &variant_body_direct, &callback->userdata };
		int argc = (callback->userdata.get_type() == Variant::NIL) ? 1 : 2;

		Variant::CallError error;
		callback->object->call(callback->method, argv, argc, error);
	}
}

PhysicsDirectBodyState *PluggablePhysicsServer::body_get_direct_state(RID rid) {
	ERR_FAIL_COND_V_MSG(this->fn_table.body_get_direct_state == nullptr, nullptr, "Not implemented");
	index_t id = this->get_index(rid);
	(*this->fn_table.body_get_direct_state)(id, &this->body_state_singleton.state);
	return &this->body_state_singleton;
}

void PluggablePhysicsServer::body_set_force_integration_callback(RID body, Object *receiver, const StringName &method, const Variant &userdata) {
	index_t id = this->get_index(body);
	Callback callback(receiver, method, userdata);
	this->callbacks.set(id, callback);
}

void PluggablePhysicsServer::free(RID rid) {
	ERR_FAIL_COND_MSG(this->fn_table.free == nullptr, "Not implemented");
	index_t id = this->get_index(rid);
	this->rids.free(rid);
	this->reverse_rids.erase(id);
	this->callbacks.erase(id);
	// SAFETY: the RID is removed
	index_mut_t mut_id = (index_mut_t)id;
	(*this->fn_table.free)(mut_id);
}
