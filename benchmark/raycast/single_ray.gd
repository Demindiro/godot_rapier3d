extends Spatial

export var from := Vector3(0, 0.5, 0)
export var to := Vector3(0, -0.5, 0)


func _ready():
	#yield(get_tree(), "idle_frame")
	var state = get_world().direct_space_state
	var result = state.intersect_ray(from, to)
	print(result)
