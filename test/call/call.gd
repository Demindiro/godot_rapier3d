extends Node


var acc = 0.0
var si = -1.0
func _physics_process(delta):
	acc += delta
	if acc >= 0.5:
		var rid = $RigidBody.get_rid()
		PhysicsServer.body_set_local_com(rid, Vector3(si * 10, 0, 0))
		si = -si
		acc = 0.0
