#!/usr/bin/env python3

"""
This script generates the method overrides and the FFI struct based on the
contents of `server.h`
"""


# Functions to exclude from server_gen.cpp
SERVER_EXCLUDE = {
    'body_set_force_integration_callback',
    'body_get_direct_state',
    'step',
    'free',
    'init',
}
# Functions to exclude from fn_table.h
FN_TABLE_EXCLUDE = {
    'body_set_force_integration_callback',
    'body_get_direct_state',
    'free',
}

# Extra functions to add to fn_table.h
FN_TABLE_CUSTOM_FUNCTIONS = (
    'void (*body_get_direct_state)(index_t id, struct body_state *state)',
    'void (*free)(index_t id)',
)


CLASS_NAME = 'PluggablePhysicsServer'
FN_STRUCT_NAME = 'fn_table'

SERVER_CPP_TAIL = ''


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
}

physics_type_map_cpp = physics_type_map[:]
physics_type_map_c = physics_type_map[:]

for i, t in enumerate(physics_type_map):
    physics_type_map[i] = CLASS_NAME + '::' + t
    physics_type_map_cpp[i] = (t, CLASS_NAME + '::' + t)
    physics_type_map_c[i] = (CLASS_NAME + '::' + t, 'void' if t in physics_structs else 'int')

physics_type_map_c.append(('List<RID>', 'void')) # TODO
physics_type_map_c.append(('Vector<Vector3>', 'Vector_Vector3')) # TODO


def create_fn_table_function(method, ret, args):
    for (k, v) in physics_type_map_c:
        if ret == 'RID':
            # The FFI interface uses indices, which are then "converted" to RIDs
            ret = 'index_t';
        elif ret == k:
            ret = v
        args = args.replace(k, v)
    args = args \
        .replace('&', '*') \
        .replace('Vector3::Axis', 'int') \
        .replace('RID', 'index_t')
    line = f'\t\t{ret} (*{method}) {args}'
    if line.endswith('const'):
        line = line[:-len('const')]
    line += ';\n'
    return line


def create_server_function(method, ret, args):
    line = f'{ret} {CLASS_NAME}::{method}{args} {{\n'

    # Determine correct error macro and return value
    if ret == 'void':
        err = f'ERR_FAIL_COND_MSG(!{FN_STRUCT_NAME}.{method}, "Not implemented")'
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
        err = f'ERR_FAIL_COND_V_MSG(!{FN_STRUCT_NAME}.{method}, {err_ret}, "Not implemented")'
    line += f'\t{err};\n'

    # Create argument list for fn_table function
    params = []
    for a in filter(bool, args[1:-len(') const') if args.endswith('const') else -1].split(',', )):
        a = a.split()
        (typ, name) = a[-2:]
        if typ == 'RID':
            # Get the actual ID
            new_name = name + '_index'
            line += f'\tindex_t {new_name} = {name}.get_id();\n'
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
        line += f'\tindex_t index = {ret_s};\n'
        # If the method name ends with '_create' we most likely need to create a new RID
        if method.endswith('_create') or method.startswith('joint_create_'):
            ret_s = 'this->rids.create(index)'
        # Otherwise, we need to look up an existing one
        else:
            ret_s = 'this->rids.get(index)'

    # Create wrapper
    line += f'\treturn {ret_s};\n'
    line += '}\n\n'
    return line


def generate(server_h, fn_table_h, server_cpp):
    fn_table_h.write(
        '#ifndef PLUGGABLE_PHYSICS_SERVER_FN_TABLE_H\n'
        '#define PLUGGABLE_PHYSICS_SERVER_FN_TABLE_H\n'
        '\n'
        '#include "index.h"\n'
        '#include "body_state.h"\n'
        '\n'
        '#ifdef USE_GDNATIVE_HEADERS\n'
        '# include "../native/gdnative.h"\n'
        '#else\n'
        '# include "core/rid.h"\n'
        '# include "servers/physics_server.h"\n'
        'typedef Vector<Vector3> Vector_Vector3;\n'
        '#endif\n'
        '\n'
        '#ifdef __cplusplus\n'
        'extern "C" {\n'
        '#endif\n'
        '\n'
        '\tstruct fn_table {\n'
    )
    server_cpp.write('#include "server.h"\n')

    for line in server_h.readlines():
        line = line.strip()
        if line.startswith('virtual '):
            line = line.split(' ', 1)[1]
            if line[-1] != ';':
                # The method is presumably already defined
                continue
            line = line[:-1]

            # Split return type, method & arguments
            (typ, line) = line.split(' ', 1)
            (method, line) = line.split('(', 1)
            args = '(' + line

            # Strip default args
            for k in ('false', 'true', 'Variant()', 'Transform()', 'NULL', '0.001', 'BODY_MODE_RIGID'):
                args = args.replace(' = ' + k, '')

            # Fix method return type and name if it is a pointer
            if method[0] == '*':
                method = method[1:]
                typ += ' *'

            # Make PhysicsServer types fully qualified
            for (k, v) in physics_type_map_cpp:
                if typ == k:
                    typ = v
                for pre in (' ', '\t', '('):
                    args = args.replace(pre + k, pre + v)

            # Add fn_table function pointer
            if method not in FN_TABLE_EXCLUDE:
                fn_table_h.write(create_fn_table_function(method, typ, args))

            # Add PhysicsServer wrapper function
            if method not in SERVER_EXCLUDE:
                server_cpp.write(create_server_function(method, typ, args))

    # Add custom fn_table functions
    for fn in FN_TABLE_CUSTOM_FUNCTIONS:
        fn_table_h.write(f'\t\t{fn};\n')

    fn_table_h.write(
        '\t};\n'
        '\n'
        '#ifdef __cplusplus\n'
        '}\n'
        '#endif\n'
        '\n'
        '#endif\n'
    )


with open('server.h') as server_h:
    with open('fn_table.h', 'w') as fn_table_h:
        with open('server_gen.cpp', 'w') as server_cpp:
            generate(server_h, fn_table_h, server_cpp)

