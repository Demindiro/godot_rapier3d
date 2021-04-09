extends Node


export var boxes_scene: PackedScene
export var spheres_scene: PackedScene

export var frames_per_test := 1000


func _enter_tree():
	var file := File.new()

	if not Engine.has_method("get_physics_step_time_usec"):
		print("Engine::get_physics_step_time_usec not found, this may affect measurement accuracy")
		print("Use a Godot build with the provided patch to ensure only the time spent inside")
		print("PhysicServer::step is measured")

	var eng: String = ProjectSettings.get("physics/3d/physics_engine").to_lower()
	for m in get_method_list():
		m = m["name"]
		if not m.begins_with("test_"):
			continue

		printt(eng, m)
		var node: Node = funcref(self, m).call_func()
		var e := file.open("user://%s_%s.txt" % [eng, m], File.WRITE)
		assert(e == OK, "Failed to open stats file: %d" % e)

		var time: float
		for _i in frames_per_test:
			if Engine.has_method("get_physics_step_time_usec"):
				yield(get_tree(), "physics_frame")
				file.store_line(str(Engine.get_physics_step_time_usec()))
			else:
				time = OS.get_ticks_usec()
				yield(get_tree(), "physics_frame")
				var t := OS.get_ticks_usec()
				file.store_line(str(t - time))
				time = t

		file.close()
		node.queue_free()
		# Remove the node too to ensure the removal of boxes doesn't stall the next benchmark
		# (This should be fixed internally eventually, I guess...)
		remove_child(node)

	get_tree().quit()


func test_boxes_dense():
	var n := boxes_scene.instance()
	n.offset = Vector3.ONE
	add_child(n)
	return n


func test_boxes_spaced():
	var n := boxes_scene.instance()
	n.offset = Vector3.ONE * 1.1
	add_child(n)
	return n
