import carla
import time

client = carla.Client('127.0.0.1', 2000)
client.set_timeout(2.0)
world = client.get_world()
actors = world.get_actors()

print(actors)

for i in range(1000):

    world.tick()

    time.sleep(0.01)