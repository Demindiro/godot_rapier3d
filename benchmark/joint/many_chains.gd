extends Node

export var chain: PackedScene
export var chain_count := 1000
export var chain_offset := 10.0

func _ready():
	for i in chain_count:
		var c: Spatial = chain.instance()
		c.translation = Vector3(0, 0, i * chain_offset)
		add_child(c)
