use gdnative::{godot_nativescript_init, godot_gdnative_init, godot_gdnative_terminate};
use godot_rapier3d::init;


godot_gdnative_init!(_ as gd_rapier3d_gdnative_init);
godot_nativescript_init!(init as gd_rapier3d_nativescript_init);
godot_gdnative_terminate!(_ as gd_rapier3d_gdnative_terminate);
