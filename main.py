import argparse
import json
import logging
import tempfile
import time

import glob
import os
import re
import sys
import uuid
import numpy as np
import base64
import paho.mqtt.client as mqttClient
import subprocess
from tracemalloc import start

import carla  # pylint: disable=import-error

try:
    sys.path.append(
        glob.glob('../../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

from modules.bridge_helper import BridgeHelper
from modules.carla_handler import CarlaSimulation, CarlaActor

global server, config, sensor_config
global sensors; sensors = {}

script_dir = os.path.dirname(os.path.abspath(__file__))
config_path = os.path.join(script_dir, 'configuration.json')
sensor_config_path = os.path.join(script_dir, 'sensors.json')

with open(config_path, 'r') as f:
    config = json.load(f)

with open(sensor_config_path, 'r') as f:
    sensor_config = json.load(f)

class CARLASensor:
    def __init__(self, configuration, mqtt_client, world):
        self.configuration = configuration
        self.mqtt_client = mqtt_client
        self.id = uuid.uuid4()
        self.name = configuration.get('name', str(self.id))
        self.data = None

        blueprint = world.get_blueprint_library().find(configuration['type'])
        for key, value in configuration['attributes'].items():
            blueprint.set_attribute(key, str(value))

        position = carla.Location(x=configuration['transform']['position'][0], y=configuration['transform']['position'][1], z=configuration['transform']['position'][2])
        rotation = carla.Rotation(pitch=configuration['transform']['rotation'][0], yaw=configuration['transform']['rotation'][1], roll=configuration['transform']['rotation'][2])
        transform = carla.Transform(position, rotation)

        self.sensor = world.spawn_actor(blueprint, transform)
        self.sensor.listen(self.update)
        sensors[str(self.id)] = self

    def update(self, output):
        if self.configuration["type"] == "sensor.camera.rgb":
            image_array = np.frombuffer(output.raw_data, dtype=np.uint8)
            image_array = image_array.reshape((output.height, output.width, 4))
            rgb_array = image_array[:, :, :3]
            formatted_data = rgb_array.tobytes()

            self.data = {
                'type': self.configuration['type'],
                'data': base64.b64encode(formatted_data).decode('utf-8'),
                'metadata': {
                    'width': output.width if hasattr(output, 'width') else 0,
                    'height': output.height if hasattr(output, 'height') else 0,
                    'timestamp': time.time()
                }
            }

        else:
            self.data = {
                'type': self.configuration['type'],
                'data': base64.b64encode(output.raw_data).decode('utf-8'),
                'metadata': {
                    'timestamp': time.time()
                }
            }

        payload = json.dumps(self.data)
        self.mqtt_client.publish(f"Stream/{self.name}", payload=payload, qos=0, retain=False)

    def destroy(self):
        sensors[str(self.id)] = None
        self.sensor.destroy()

class Simulation(object):
    def __init__(self, carla_simulation, args):
        self.args = args

        self.vissim = os.path.join(tempfile.gettempdir(), 'vissim_data.json')
        if os.path.exists(self.vissim):
            os.remove(self.vissim)
            
        self.carla = carla_simulation

        self.data = {'vehicles': {}, 'pedestrians': {}, 'signals': {}}

        BridgeHelper.blueprint_library = self.carla.world.get_blueprint_library()
        dir_path = os.path.dirname(os.path.realpath(__file__))
        with open(os.path.join(dir_path, 'vtypes.json')) as f:
            BridgeHelper.vtypes = json.load(f)

        settings = self.carla.world.get_settings()
        #settings.no_rendering_mode = True
        settings.synchronous_mode = True
        #settings.fixed_delta_seconds = None
        settings.substepping = False

        self.carla.world.apply_settings(settings)
        
    def tick(self):
        if os.path.exists(self.vissim):
            try:
                with open(self.vissim, 'r') as f:
                    self.data = json.load(f)
            except (OSError, json.JSONDecodeError):
                pass

            # Spawn vehicles.
            for vissim_actor_id, vissim_actor_data in self.data['vehicles'].items():
                if self.carla.vehicle_ids.get(vissim_actor_id, None) is not None:
                    continue
                
                print(f'Spawning carla actor for vissim vehicle {vissim_actor_id}')
                carla_actor = CarlaActor(self.carla, "vehicle", vissim_actor_data)
                if carla_actor.carla_id != -1:
                    self.carla.actors[carla_actor.carla_id] = carla_actor

            if not self.args.no_pedestrians:
                for vissim_actor_id, vissim_actor_data in self.data['pedestrians'].items():
                    if self.carla.walker_ids.get(vissim_actor_id, None) is not None:
                        continue
                    
                    print(f'Spawning carla actor for vissim pedestrian {vissim_actor_id}')
                    carla_actor = CarlaActor(self.carla, "walker", vissim_actor_data)
                    if carla_actor.carla_id != -1:
                        self.carla.actors[carla_actor.carla_id] = carla_actor

            # Synchronize traffic lights.
            for name, state in self.data['signals'].items():
                if name is b'':
                    continue

                pattern = r'X=([+-]?\d*\.?\d+),Y=([+-]?\d*\.?\d+),Z=([+-]?\d*\.?\d+)'
                match = re.search(pattern, name)
                location = None

                if match:
                    x = float(match.group(1))
                    y = float(match.group(2))
                    z = float(match.group(3))
                        
                    location = carla.Location(x, y, z)

                if location is None:
                    continue

                for actor in self.carla.world.get_actors().filter('traffic.*'):
                    transform = actor._transform
                    if (transform.location*100).distance(location) < 1:
                            actor.set_state(state)
        
        tick_start = time.time()

        # Synchronize actors.
        for carla_id, carla_actor in list(self.carla.actors.items()):

            if getattr(self.carla, carla_actor.type + '_ids').get(carla_actor.vissim_id, None) is None:
                continue

            list_name = "vehicles" if carla_actor.type == "vehicle" else "pedestrians"
            vissim_list = self.data[list_name]

            if list_name == "pedestrians" and self.args.no_pedestrians:
                continue

            if vissim_list.get(str(carla_actor.vissim_id), None) is None:
                print(f'Destroying carla actor {carla_id} for missing vissim actor {carla_actor.vissim_id}')
                carla_actor.destroy()
            else:
                vissim_actor_data = vissim_list[str(carla_actor.vissim_id)]
                carla_actor.synchronize(vissim_actor_data)
            

        print("Time to synchronize actors: " + str(time.time() - tick_start))
        
    def close(self):

        for carla_actor in list(self.carla.actors.values()):
            carla_actor.destroy()

def main(args):

    network_folder = os.path.join(script_dir, 'network')
    inpx_file = glob.glob(os.path.join(network_folder, '*.inpx'))
    
    inpx_file_path = inpx_file[0]
    map_name = os.path.splitext(os.path.basename(inpx_file_path))[0]

    carla_simulation = CarlaSimulation(config["carla"], map_name)
    vissim_process = os.path.join(script_dir, 'modules/vissim_handler.py')

    terminate_flag = os.path.join(tempfile.gettempdir(), 'vissim_terminate.flag')
    if os.path.exists(terminate_flag):
        os.remove(terminate_flag)

    subprocess.Popen(['python', vissim_process], creationflags=subprocess.CREATE_NO_WINDOW)

    client = mqttClient.Client(mqttClient.CallbackAPIVersion.VERSION1, "CARLA_Sensors", clean_session=True)
    client.connect(config["mqtt"]["broker"], config["mqtt"]["port"])

    if not arguments.no_sensors:
        for sensor_cfg in sensor_config:
            try:
                sensor = CARLASensor(sensor_cfg, client, carla_simulation.world)
                print(f"Spawned sensor: {str(sensor.name)}")
            except Exception as e:
                print(f"Error spawning sensor: {e}")

    try:
        synchronization = Simulation(carla_simulation, args)
        last_tick = time.time()
        while True:
            synchronization.tick()
            carla_simulation.tick()
            print("Time to solve frame: " + str(time.time() - last_tick))
            last_tick = time.time()

    except KeyboardInterrupt:
        logging.info('Cancelled by user.')

    finally:
        logging.info('Cleaning synchronization')
        for sensor in sensors.values():
            sensor.sensor.destroy()

        open(os.path.join(tempfile.gettempdir(), 'vissim_terminate.flag'), 'w').close()
        synchronization.close()

if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--no-sensors', action='store_true', help='disable sensors')
    argparser.add_argument('--no-pedestrians', action='store_true', help='disable pedestrians')
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    arguments = argparser.parse_args()

    if arguments.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    main(arguments)
