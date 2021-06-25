extends Node


export var box: PackedScene
export var count := 1000
export var spawn := Vector3(0, 1, 0)


func _ready():
	var list := []
	for _i in 1000:
		var b: RigidBody = box;
		for e in list:
			b.add_collision_exception_with(e)
		b.translation = spawn
		add_child(b)
		list.push(b)
