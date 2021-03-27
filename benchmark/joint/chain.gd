extends Node

export var box_scene: PackedScene
export var length := 10
export var colors := PoolColorArray([Color.white, Color.green, Color.blue, Color.red, Color.cyan, Color.yellow])
export var offset := 0.0

func _ready():
	var anchor_path := "Anchor"
	for i in length:
		var joint := HingeJoint.new()
		joint.set("nodes/node_a", "../%s" % anchor_path)
		joint.set("nodes/node_b", "../Box %d" % i)
		joint.translation = Vector3((i + 0.5) * (1 + offset), 0, 0)
		joint.rotate(Vector3.RIGHT, PI / 2)
		var box := box_scene.instance()
		box.get_node("Mesh").color = colors[i % len(colors)]
		box.name = "Box %d" % i
		box.translation = Vector3((i + 1) * (1 + offset), 0, 0)
		add_child(box)
		add_child(joint)
		anchor_path = box.name
