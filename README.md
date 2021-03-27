# Godot bindings for the Rapier3D physics library

## How to use

There are two parts:

* A Godot module which must be compiled with the engine. This is necessary to
  communicate with the library through the PhysicsServer
* A library with the actual bindings

1. Run `module/generate.py` first
2. Then, either make a symlink or copy the contents of `module` to the engine source
3. Build the engine
4. Build the library with the provided `Makefile` using `make`
5. Set `3d/physics_engine` to `Custom`

Note that these instructions have only been tested on Linux. If you have
difficulties, please contact me or open an issue.


# The editor may crash if you keep `3d/physics_engine` set to `Custom`. If it does, uncomment it in `project.godot`
