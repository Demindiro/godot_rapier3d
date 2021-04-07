extends Area

export var color_enter := Color(1.0, 0.0, 0.0, 1.0)
export var color_exit := Color(0.0, 1.0, 0.0, 1.0)


func enter(n):
	n = n.get_node_or_null("Mesh")
	if n:
		n.color = color_enter


func exit(n):
	n = n.get_node_or_null("Mesh")
	if n:
		n.color = color_exit
