extends Label


var sample_time := OS.get_ticks_msec()


func _physics_process(_delta: float) -> void:
	var draw_calls = get_tree() \
		.root \
		.get_render_info(Viewport.RENDER_INFO_DRAW_CALLS_IN_FRAME)
	var time := OS.get_ticks_msec()
	var delta := time - sample_time
	sample_time = time
	text = ""
	text += "\nFPS: %f" % (1000.0 / delta)
	text += "\nTime per frame: %f" % (delta / 1000.0)
	text += "\nFrame: %d" % Engine.get_physics_frames()
	text += "\nDraw calls: %d" % draw_calls
