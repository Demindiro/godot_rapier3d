#ifndef PLUGGABLE_PHYSICS_TYPEDEF_H
#define PLUGGABLE_PHYSICS_TYPEDEF_H

#include "core/variant.h"

typedef Transform godot_transform;
typedef Vector3 godot_vector3;
typedef Variant godot_variant;
typedef StringName godot_string_name;
typedef Object godot_object;
typedef Vector<Vector3> godot_pool_vector3_array;

typedef const struct physics_body_state * physics_body_state_t;
typedef struct physics_body_state * physics_body_state_mut_t;

typedef const struct physics_space_state * physics_space_state_t;
typedef struct physics_space_state * physics_space_state_mut_t;

typedef const struct physics_area_monitor_event * physics_area_monitor_event_t;
typedef struct physics_area_monitor_event * physics_area_monitor_event_mut_t;


#endif
