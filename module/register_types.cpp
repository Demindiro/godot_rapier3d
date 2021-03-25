#include "register_types.h"

#include "core/class_db.h"
#include "core/project_settings.h"
#include "servers/physics_server.h"

#include "server.h"

#ifndef _3D_DISABLED
PhysicsServer *_createPluggablePhysicsCallback() {
	return memnew(PluggablePhysicsServer);
}
#endif

void register_pluggable_physics_types() {
#ifndef _3D_DISABLED
	PhysicsServerManager::register_server("Custom", &_createPluggablePhysicsCallback);

	GLOBAL_DEF("physics/3d/custom_library_path", "");
	String lib_path_prop = "physics/3d/custom_library_path";
	PropertyInfo prop_info(Variant::STRING, lib_path_prop, PROPERTY_HINT_FILE, "*.gdnlib");
    ProjectSettings::get_singleton()->set_custom_property_info(lib_path_prop, prop_info);
#endif
}

void unregister_pluggable_physics_types() {
}
