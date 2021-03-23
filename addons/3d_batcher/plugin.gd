tool
extends EditorPlugin


const AUTOLOAD_SCRIPT := "batched_mesh_manager.gdns"

var autoload = null


func _enter_tree() -> void:
	autoload = preload(AUTOLOAD_SCRIPT).new()
	add_child(autoload)


func _exit_tree() -> void:
	autoload.queue_free()
	autoload = null
