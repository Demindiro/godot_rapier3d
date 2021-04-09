extends Label


var sample_time := OS.get_ticks_usec()
var samples := []

const SAMPLE_COUNT := 60


func _physics_process(_delta: float) -> void:
	var draw_calls = get_tree() \
		.root \
		.get_render_info(Viewport.RENDER_INFO_DRAW_CALLS_IN_FRAME)

	var delta: float
	if Engine.has_method("get_physics_step_time_usec"):
		delta = Engine.get_physics_step_time_usec()
	else:
		var time := OS.get_ticks_usec()
		delta = time - sample_time
		sample_time = time

	samples.push_back(delta)
	if len(samples) > SAMPLE_COUNT:
		samples.pop_front()

	var d := 0
	for s in samples:
		d += s
	d /= len(samples)
	if d == 0:
		d = 1

	text = "Engine: %s (%s)" % [ProjectSettings.get("physics/3d/physics_engine"), "Debug" if OS.is_debug_build() else "Release"]
	text += "\nReciprocal step time: %f" % (1000000.0 / d)
	text += "\nStep time: %f" % (d / 1000000.0)
	text += "\nFrame: %d" % Engine.get_physics_frames()
	text += "\nDraw calls: %d" % draw_calls
