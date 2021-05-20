# Godot bindings for the Rapier3D physics library

## How to use
  There are two parts:

  * A Godot module which must be compiled with the engine. This is necessary to
    communicate with the library through the PhysicsServer
  * A library with the actual bindings

## Linux:

  - Run `module/generate.py` first
  - Then, either make a symlink or copy the contents of `module` to the engine source `modules/pluggable_physics`
  - Build the engine
  - Build the library with the provided `Makefile` using `make`
  - Set `3d/physics_engine` to `Custom`

## Windows:
  - Run `python module/generate.py`
  - Copy `module/api.json` to `rapier3d/api.json`
  - Then, either make a symlink or copy the contents of `module` to the engine source `modules/pluggable_physics`
  - Build the engine
  - Build the library following these steps:
    - ensure `LIBCLANG_PATH` environment variable is set to llvm's bin folder containing `libclang.dll`
    - `rustup default nightly-msvc`
      - If you get an error try: `cargo update`
    - run `build_win.bat`
  - Set `3d/physics_engine` to `Custom`

If you have difficulties, please contact me or open an issue.

# The editor may crash if you keep `3d/physics_engine` set to `Custom`. If it does, uncomment it in `project.godot`
