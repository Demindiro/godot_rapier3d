#!/usr/bin/env python3

"""
This script generates the method overrides and the FFI struct based on the
contents of `server.h`. It also dumps a JSON file with all functions for use
with other languages (i.e. Rust).
"""


# Functions to exclude from server.gen.cpp
SERVER_EXCLUDE = {
    'area_set_area_monitor_callback',
    'area_set_monitor_callback',
    'body_set_force_integration_callback',
    'body_get_direct_state',
    'body_get_collision_exceptions',
    'free',
    'init',
    'soft_body_get_collision_exceptions',
    'soft_body_update_visual_server',
    'space_get_direct_state',
    'step',
}

# Functions to exclude from the API
API_EXCLUDE = {
    'area_set_area_monitor_callback',
    'area_set_monitor_callback',
    'body_set_force_integration_callback',
    'body_get_direct_state',
    'body_get_collision_exceptions',
    'free',
    'soft_body_get_collision_exceptions',
    'soft_body_update_visual_server',
    'space_get_direct_state',
}

# Functions where some RIDs may be null
API_MAYBE_RIDS = {
    'body_set_space': {'space'},
}

# Extra functions to add to the API
API_CUSTOM_FUNCTIONS = {
    'area_get_monitor_event': ('void', [('uint32_t', 'index', True), ('physics_area_monitor_event_mut_t', 'event', False)], True),
    'area_get_monitor_event_count': ('uint32_t', [], True),
    'body_get_direct_state': ('void', [('index_t', 'body', True), ('physics_body_state_mut_t', 'state', False)], True),
    'body_get_collision_exception': ('index_t', [('index_t', 'body', True), ('int', 'index', True)], True),
    'body_get_collision_exception_count': ('int', [('index_t', 'body', True)], True),
    'free': ('void', [('index_mut_t', 'id', False)], False),
    'space_intersect_ray': ('bool', [
        ('index_t', 'space', True),
        ('struct physics_ray_info *', 'info', True),
        ('struct physics_ray_result *', 'result', False),
    ], False),
    'soft_body_get_collision_exception': ('index_t', [('index_t', 'body', True), ('int', 'index', True)], True),
    'soft_body_get_collision_exception_count': ('int', [('index_t', 'body', True)], True),
}

# Functions for which to validate all RIDs
VALIDATE_ALL_RIDS = {
    'body_add_collision_exception',
    'body_remove_collision_exception',
    'body_add_shape',
    'joint_create_cone_twist',
    'joint_create_generic_6dof',
    'joint_create_hinge',
    'joint_create_pin',
    'joint_create_slider',
    'joint_create_hinge_simple',
}

# C++ to C type map
TYPE_CPP_TO_C = {
    'Transform': 'godot_transform',
    'RID': 'godot_rid',
    'Vector3': 'godot_vector3',
    'Object': 'godot_object',
    'real_t': 'float',
}


CLASS_NAME = 'PluggablePhysicsServer'
RID_DATA_NAME = 'PluggablePhysicsRID_Data'
FN_STRUCT_NAME = 'fn_table'

SERVER_CPP_TAIL = ''


# Create a mapping for various types
physics_type_map = [
    'ShapeType',
    'SpaceParameter',
    'AreaParameter',
    'AreaSpaceOverrideMode',
    'BodyMode',
    'BodyParameter',
    'BodyState',
    'BodyAxis',
    'PinJointParam',
    'JointType',
    'HingeJointParam',
    'HingeJointFlag',
    'SliderJointParam',
    'ConeTwistJointParam',
    'G6DOFJointAxisParam',
    'G6DOFJointAxisFlag',
    'AreaBodyStatus',
    'ProcessInfo',
    'SeparationResult',
    'MotionResult',
]

physics_structs = {
    'MotionResult',
    'SeparationResult',
    'SoftBodyVisualServerHandler',
}

c_to_cpp = {
    'godot_variant': 'Variant',
    'godot_vector3': 'Vector3',
    'godot_transform': 'Transform',
    'godot_string_name': 'StringName',
    'godot_object': 'Object',
    'godot_pool_vector3_array': 'Vector<Vector3>',
}
cpp_to_c = {v: k for k, v in c_to_cpp.items()}
cpp_to_c['REF'] = 'godot_object'


physics_type_map_cpp = {}
physics_type_map_c = {}

for i, t in enumerate(physics_type_map):
    cpp_t = CLASS_NAME + '::' + t
    physics_type_map[i] = cpp_t
    physics_type_map_cpp[t] = cpp_t
    physics_type_map_c[cpp_t] = 'void' if t in physics_structs else 'int'
    physics_type_map_c[cpp_t + ' *'] = 'void *' if t in physics_structs else 'int *'

physics_type_map_c['Vector<Vector3>'] = 'godot_pool_vector3_array'
physics_type_map_c['Transform *'] = 'godot_transform *'
physics_type_map_c['Transform'] = 'godot_transform'
physics_type_map_c['Vector3 *'] = 'godot_vector3 *'
physics_type_map_c['Vector3'] = 'godot_vector3'
physics_type_map_c['Vector3::Axis'] = 'int'
physics_type_map_c['Variant *'] = 'godot_variant *'
physics_type_map_c['Variant'] = 'godot_variant'
physics_type_map_c['StringName *'] = 'godot_string_name *'
physics_type_map_c['RID'] = 'index_t'
physics_type_map_c['ObjectID'] = 'int'
physics_type_map_c['real_t'] = 'float'
physics_type_map_c['REF *'] = 'godot_object *'


def generate_method_table(header_file):
    """
    Creates a table with all virtual methods from the given header file.
    """
    method_table = {}
    for line in server_h.readlines():
        line = line.strip()
        if line.startswith('virtual '):
            line = line.split(' ', 1)[1]
            if line[-1] != ';':
                # The method is presumably already defined
                continue
            line = line[:-1]

            # Strip const at end
            is_const = line.endswith('const')
            if is_const:
                line = line[:-len('const')]

            # Split return type, method & arguments
            (typ, line) = line.split(' ', 1)
            (method, line) = line.split('(', 1)
            args = '(' + line

            # Check if method needs to be skipped
            if method in API_EXCLUDE or (method[0] == '*' and method[1:] in API_EXCLUDE):
                continue

            # Strip default args
            for k in ('false', 'true', 'Variant()', 'Transform()', 'NULL', '0.001', 'BODY_MODE_RIGID'):
                args = args.replace(' = ' + k, '')

            # Split args
            args = list(filter(bool, args.strip()[1:-1].split(',')))
            for i, a in enumerate(args):
                a = a.strip()
                a = tuple(a.split(' '))
                if len(a) == 2:
                    a = (*a[0:2], False)
                else:
                    a = (*a[1:3], True)
                assert len(a) == 3
                args[i] = a

            # Fix method return type and name if it is a pointer
            if method[0] == '*':
                method = method[1:]
                typ += ' *'

            # Make sure C++ types are fully qualified
            typ = physics_type_map_cpp.get(typ, typ)
            for i, (t, n, c) in enumerate(args):
                t = physics_type_map_cpp.get(t, t)
                args[i] = (t, n, c)

            method_table[method] = (typ, args, is_const)

    return method_table


def clean_method_table(table):
    for method, (ret, args, const) in table.items():
        ret = physics_type_map_c.get(ret, ret)
        for i, (t, n, c) in enumerate(args):
            if i > 0 and method not in VALIDATE_ALL_RIDS:
                if t == 'RID':
                    t = 'maybe_index_t'
            if n[0] in "&*":
                t += ' *'
                n = n[1:]
            t = physics_type_map_c.get(t, t)
            args[i] = (t, n, c)
        # Presumably all API methods can fail somehow, thus RID can be null
        if ret == 'index_t':
            ret = 'maybe_index_t'
        table[method] = (ret, args, const)


def generate_api_h(method_table, api_h):
    api_h.write(
        '#ifndef PLUGGABLE_PHYSICS_SERVER_FN_TABLE_H\n'
        '#define PLUGGABLE_PHYSICS_SERVER_FN_TABLE_H\n'
        '\n'
        '#include "index.h"\n'
        '#include "body_state.h"\n'
        '#include "space_state.h"\n'
        '\n'
        '#include "core/rid.h"\n'
        '#include "servers/physics_server.h"\n'
        '\n'
        '#ifdef __cplusplus\n'
        'extern "C" {\n'
        '#endif\n'
        '\n'
        '\tstruct fn_table {\n'
    )

    for method, (ret, args, is_const) in method_table.items():
        args = ', '.join(f'{"const " if c else ""}{t} {n}' for t, n, c in args)
        line = f'\t\t{ret} (*{method}) ({args})'
        if line.endswith('const'):
            line = line[:-len('const')]
        line += ';\n'
        api_h.write(line)

    api_h.write(
        '\t};\n'
        '\n'
        '#ifdef __cplusplus\n'
        '}\n'
        '#endif\n'
        '\n'
        '#endif\n'
    )


def generate_api_json(method_table, api_json):
    api_json.write('{')
    skip_comma = True # JSON doesn't allow trailing commas :(
    for method, (ret, args, is_const) in method_table.items():
        if skip_comma:
            skip_comma = False
        else:
            api_json.write(',')
        args = '[' + ', '.join([f'{{"type": "{t}", "name": "{n}", "const": {"true" if c else "false"}}}' for t, n, c in args]) + ']'
        ret = f'{{"type": "{ret}", "const": {"true" if is_const else "false"}}}'
        line = f'\n\t"{method}": {{"return": {ret}, "args": {args}}}'
        api_json.write(line)
    api_json.write('\n}')


def generate_server_cpp(method_table, server_cpp):
    def add_line(tabs, text, end = ';'):
        for _ in range(tabs):
            server_cpp.write('\t')
        server_cpp.write(text)
        server_cpp.write(end)
        server_cpp.write('\n')

    add_line(0, '#include "server.h"', '')
    add_line(0, '', '')
    add_line(0, '', '')

    for method, (ret, args, is_const) in method_table.items():

        args = args[:]

        # Check if it needs to be skipped
        if method in SERVER_EXCLUDE:
            continue

        # Add definition
        definition_args = ', '.join([('const ' if c else '') + t + ' ' + n for (t, n, c) in args])
        add_line(0, f'{ret} {CLASS_NAME}::{method}({definition_args}) {"const " if is_const else ""}', '{')

        # Determine correct error macro and return value
        if ret == 'void':
            add_line(1, f'ERR_FAIL_COND_MSG({FN_STRUCT_NAME}.{method} == nullptr, "Not implemented")')
        else:
            tbl = {
                'bool': 'false',
                'int': '0',
                'long': '0',
                'float': '0.0f',
                'double': '0.0',
                'real_t': '0.0f',
                'uint32_t': '0',
                'ObjectID': '0',
                'Variant': 'Variant()',
                'Vector3': 'Vector3()',
                'RID': 'RID()',
                'Transform': 'Transform()',
                f'{CLASS_NAME}::AreaSpaceOverrideMode': f'{CLASS_NAME}::AREA_SPACE_OVERRIDE_DISABLED',
                f'{CLASS_NAME}::BodyMode': f'{CLASS_NAME}::BODY_MODE_RIGID',
                f'{CLASS_NAME}::JointType': f'{CLASS_NAME}::JOINT_PIN',
                f'{CLASS_NAME}::ShapeType': f'{CLASS_NAME}::SHAPE_CUSTOM',
                'Vector<Vector3>': 'Vector<Vector3>()',
            }
            err_ret = tbl.get(ret, 'nullptr')
            add_line(1, f'ERR_FAIL_COND_V_MSG({FN_STRUCT_NAME}.{method} == nullptr, {err_ret}, "Not implemented")')

        # Cast parameters as needed
        for i, (t, n, c) in enumerate(args):
            if t in cpp_to_c:
                ref = n[0] == '&'
                if ref:
                    nn = 'ffi_' + n[1:]
                else:
                    nn = 'ffi_' + n
                tt = cpp_to_c[t]
                add_line(1, f'{tt} {"*" if ref else ""}{nn} = {"" if ref else "*"}({tt} *){n}')
                args[i] = (tt, nn, c)

        # Create argument list for fn_table function
        params = []
        for a in args:
            (typ, name, is_const) = a
            if typ == 'RID':
                # Make sure the RID is valid. Only necessary for the first argument, others
                # may actually be invalid (e.g. unset space for body). If all RIDs need to be
                # valid, add the method in VALIDATE_ALL_RIDS
                if len(params) == 0 or method in VALIDATE_ALL_RIDS:
                    if ret == 'void':
                        add_line(1, f'ERR_FAIL_COND_MSG(!{name}.is_valid(), "Invalid RID")')
                    else:
                        add_line(1, f'ERR_FAIL_COND_V_MSG(!{name}.is_valid(), {err_ret}, "Invalid RID")')
                # Get the actual ID
                new_name = name + '_index'
                add_line(1, f'index_t {new_name} = this->get_index({name})')
                name = new_name
            else:
                if name[0] == '*':
                    name = name[1:]
            params.append(name)
        params = ', '.join(params)

        ret_s = f'(*{FN_STRUCT_NAME}.{method})({params})'
        # Check if we're returning an enum and if so, cast the int to the enum
        if ret in physics_type_map:
            ret_s = f'{ret}({ret_s})'
        # Check if we're returing a RID. If so, get or create one
        elif ret == 'RID':
            add_line(1, f'index_t index = {ret_s}')
            # If the method name ends with '_create' we most likely need to create a new RID
            if method.endswith('_create') or method.startswith('joint_create_'):
                ret_s = 'this->make_rid(index)'
            # Otherwise, we need to look up an existing one
            else:
                ret_s = 'this->reverse_rids.get(index)'

        add_line(1, f'return {ret_s}')
        add_line(0, '', '}')
        add_line(0, '', '')


# Get the method table
with open('server.h') as server_h:
    method_table = generate_method_table(server_h)

# Generate server.gen.cpp
with open('server.gen.cpp', 'w') as server_cpp:
    generate_server_cpp(method_table, server_cpp)

# Convert method table to "pure" C
clean_method_table(method_table)

# Add custom API functions
for k, v in API_CUSTOM_FUNCTIONS.items():
    method_table[k] = v

# Generate api.gen.h and api.json
with open('api.gen.h', 'w') as api_h:
    generate_api_h(method_table, api_h)
with open('api.json', 'w') as api_json:
    generate_api_json(method_table, api_json)
