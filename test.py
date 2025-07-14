import carla
import time
import cv2
import numpy as np

client = carla.Client('127.0.0.1', 2000)
client.set_timeout(2.0)
world = client.get_world()
actors = world.get_actors()

blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
blueprint.set_attribute('image_size_x', '640')
blueprint.set_attribute('image_size_y', '480')
blueprint.set_attribute('fov', '80')
blueprint.set_attribute('gamma', '1.6')
blueprint.set_attribute('bloom_intensity', '1')
blueprint.set_attribute('slope', '0.88')
blueprint.set_attribute('toe', '0.2')
blueprint.set_attribute('shoulder', '0.8')
blueprint.set_attribute('black_clip', '0.025')
blueprint.set_attribute('white_clip', '0.5')
blueprint.set_attribute('exposure_mode', 'histogram')
blueprint.set_attribute('chromatic_aberration_intensity', '1.0')
blueprint.set_attribute('lens_circle_falloff', '5.0')
blueprint.set_attribute('lens_k', '-1.5')
blueprint.set_attribute('lens_kcube', '0.5')

sensor_location = carla.Transform(carla.Location(x=6.30,y=-6.30,z=7.80), carla.Rotation(pitch=-18.993288,yaw=134.792679,roll=0.000395))
sensor = world.spawn_actor(blueprint, sensor_location)

image_queue = []

def process_image(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    image_queue.append(array)

print(sensor)
sensor.listen(process_image)

for i in range(1000):

    world.tick()
    if image_queue:
        img = image_queue.pop(0)
        cv2.imshow('CARLA Camera', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    time.sleep(0.01)

cv2.destroyAllWindows()
sensor.destroy()