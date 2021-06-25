extends Spatial

func _physics_process(_d):
	for _i in 1000 * 1000 * 1000:
		var param := PhysicsShapeQueryParameters.new()
		var shape := SphereShape.new()
		shape.radius = 10.0
		param.set_shape(shape)
		param.transform.origin = Vector3()
		param.collision_mask = 0xffffffff
		get_world().direct_space_state.intersect_shape(param)
