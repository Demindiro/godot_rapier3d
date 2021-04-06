extends Node

export var ball: PackedScene

export var start := Vector3()
export var distance := Vector3()
export var count_x := 100
export var count_y := 100

export var colors := PoolColorArray([
	Color.white,
	Color.yellow,
	Color.green,
	Color.blue,
	Color.red,
	Color.cyan,
	Color.orange,
	Color.purple,
	Color.mistyrose,
])


func _ready() -> void:
	var i := 0
	for x in count_x:
		for y in count_y:
			var pos := start + distance * Vector3(x, 0, y)
			var node: Spatial = ball.instance()
			node.translation = pos
			node.get_node("Mesh").color = colors[i % len(colors)]
			add_child(node)
			i += 1
