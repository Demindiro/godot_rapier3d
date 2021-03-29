extends Node

export var count_x := 100
export var count_z := 100
export var height := 0.5


func _ready():
	for x in count_x:
		for z in count_z:
			var r := RayCast.new()
			r.translation = Vector3(x, height, z)
			r.enabled = true
			add_child(r)
