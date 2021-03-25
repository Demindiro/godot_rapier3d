#ifndef NATIVE_GDNATIVE_H
#define NATIVE_GDNATIVE_H

#include "../godot-headers/gdnative/variant.h"

typedef godot_vector3 Vector3;
typedef godot_variant Variant;
typedef godot_rid RID;
typedef godot_transform Transform;
typedef godot_object Object;
typedef godot_string_name StringName;
typedef godot_pool_vector3_array Vector_Vector3;

typedef godot_object PhysicsDirectBodyState;
typedef godot_object PhysicsDirectSpaceState;

typedef float real_t;
typedef uint64_t ObjectID;
//typedef _Bool bool;
#define class struct
typedef void REF;

#endif
