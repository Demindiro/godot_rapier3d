extends Node


func _ready():
	yield(get_tree().create_timer(1.5), "timeout")
	var cs = CollisionShape.new()
	var bs = BoxShape.new()
	cs.shape = bs
	$RigidBody.add_child(cs)
