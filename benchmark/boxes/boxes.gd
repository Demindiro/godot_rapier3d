extends Node


export var x := 10
export var y := 10
export var z := 10
export var origin := Vector3.ONE
export var offset := Vector3.ONE
export var box_godot: PackedScene
export var colors := PoolColorArray([Color.white, Color.red, Color.green, Color.blue, Color.yellow, Color.magenta])


func _ready():
	BatchedMeshManager.enable_culling = false # Should reduce rendering impact a little more
	var i := 0
	for a in x:
		for b in y:
			for c in z:
				var n: Spatial = box_godot.instance()
				n.get_node("Mesh").color = colors[i % len(colors)]
				i += 1
				n.translation = Vector3(a, b, c) * offset + origin
				add_child(n)
