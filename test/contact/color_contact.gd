extends RigidBody

var node_count := 0

func enter(n):
	node_count += 1
	get_node("Mesh").color = Color.red


func exit(n):
	get_node("Mesh").color = Color.green
	node_count -= 1


func _integrate_forces(state):
	print("Node count: ", node_count)
	print("Contact count: ", state.get_contact_count())
	for i in state.get_contact_count():
		var pos = state.get_contact_collider_position(i)
		var pos_l = state.get_contact_local_position(i)
		var norm_l = state.get_contact_local_normal(i)
		var shape_self = state.get_contact_local_shape(i)
		var shape_other = state.get_contact_collider_shape(i)
		var other = state.get_contact_collider_object(i)
		print("GLOBAL POS  ", pos, "  |  LOCAL POS  ", pos_l, "  |  LOCAL NORMAL  ", norm_l, "  |  SHAPES  ", shape_self, " ", shape_other, "  |  OBJECT  ", other)
	print()
