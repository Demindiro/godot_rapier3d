extends Node


func _ready():
	PhysicsServer.set_active(false)
	print("Press 'Space' to advance the simulation")


func _input(event):
	if event.is_action_pressed("advance_simulation"):
		PhysicsServer.set_active(true)
		PhysicsServer.step(1 / 30.0)
		PhysicsServer.set_active(false)
