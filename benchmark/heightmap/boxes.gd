extends Node


export var box: PackedScene
export var count_x := 50
export var count_y := 50
export var start := Vector3(-45, 10, -45)
export var end := Vector3(45, 10, 45)
export var colors := [
	Color.white,
	Color.red,
	Color.green,
	Color.blue,
	Color.cyan,
	Color.pink,
]

func _ready():
	var d := (end - start) / Vector3(count_x, 1.0, count_y)
	var i := 0
	for x in count_x:
		for y in count_y:
			var p := start + d * Vector3(x, 0.0, y)
			var b: Spatial = box.instance()
			b.translation = p
			b.can_sleep = false
			b.get_node("Mesh").color = colors[i % len(colors)]
			add_child(b)
			i += 1
