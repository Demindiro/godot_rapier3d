extends Label


var file := File.new()
var sample_time := OS.get_ticks_msec()


func _enter_tree():
	var date := OS.get_datetime()
	var eng := "rapier3d" if $"../../".use_rapier3d else "godot3d"
	var name := "%s_%d-%d-%d_%d-%d-%d" % [eng, date.year, date.month, date.day, date.hour, date.minute, date.second]
	var e := file.open("user://%s.txt" % name, File.WRITE)
	assert(e == OK, "Failed to open stats file: %d" % e)


func _exit_tree():
	file.close()


func _physics_process(_delta: float) -> void:
	var draw_calls = get_tree() \
		.root \
		.get_render_info(Viewport.RENDER_INFO_DRAW_CALLS_IN_FRAME)
	var time := OS.get_ticks_msec()
	var delta := time - sample_time
	sample_time = time
	text = ""
	text += "\nFPS: %f" % (delta / 1000.0)
	text += "\nTime per frame: %f" % (1000.0 / delta)
	text += "\nFrame: %d" % Engine.get_physics_frames()
	text += "\nDraw calls: %d" % draw_calls
	file.store_line(str(delta))
