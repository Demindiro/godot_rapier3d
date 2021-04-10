extends Node


export var ball: PackedScene
export var count := 1000
export var enable_ccd := true
export var spawn := NodePath()
export var initial_velocity := Vector3()


func _ready():
	var pos: Vector3 = get_node(spawn).translation
	for _i in count:
		var b: Spatial = ball.instance()
		b.continuous_cd = enable_ccd
		b.collision_layer = 2
		b.translation = pos
		b.linear_velocity = initial_velocity
		add_child(b)
