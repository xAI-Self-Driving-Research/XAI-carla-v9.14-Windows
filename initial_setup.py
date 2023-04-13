import carla
import random

# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
client.load_world('Town07')
world = client.get_world()
level =  world.get_map()
weather = world.get_weather()
blueprint_library = world.get_blueprint_library()

settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)