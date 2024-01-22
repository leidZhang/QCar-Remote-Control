from logidrivepy import LogitechController 

controller = LogitechController()

print(f"steering_initialize: {controller.steering_initialize()}")
print(f"logi_update: {controller.logi_update()}")
print(f"is_connected: {controller.is_connected(0)}")

controller.steering_shutdown()