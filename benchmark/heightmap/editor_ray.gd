tool
extends RayCast


onready var marker: Spatial = get_child(0)


func _process(_delta: float) -> void:
	force_raycast_update()
	marker.global_transform.origin = get_collision_point()
