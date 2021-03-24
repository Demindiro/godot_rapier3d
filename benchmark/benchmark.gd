extends Node


export var boxes_scene: PackedScene
export var spheres_scene: PackedScene

export var frames_per_test := 1000

var use_rapier := false


func _enter_tree():
	var file := File.new()

	for i in 2:
		use_rapier = i % 2 > 0
		var eng := "rapier3d" if use_rapier else "godot3d"
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
				time = OS.get_ticks_msec()
				yield(get_tree(), "physics_frame")
				var t := OS.get_ticks_msec()
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
	n.use_rapier3d = use_rapier
	add_child(n)
	return n


func test_boxes_spaced():
	var n := boxes_scene.instance()
	n.offset = Vector3.ONE * 1.1
	n.use_rapier3d = use_rapier
	add_child(n)
	return n
