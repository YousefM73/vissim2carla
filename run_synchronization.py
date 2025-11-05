#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import argparse
import json
import logging
import time

import glob
import os
import re
import sys
import uuid
import numpy as np
import base64
import paho.mqtt.client as mqttClient
from tracemalloc import start

import carla  # pylint: disable=import-error

try:
    sys.path.append(
        glob.glob('../../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

from vissim_integration.bridge_helper import BridgeHelper
from vissim_integration.carla_simulation import CarlaSimulation, CarlaActor
from vissim_integration.vissim_simulation import PTVVissimSimulation
from vissim_integration.constants import INVALID_ACTOR_ID

brokerIP = None  # Broker address
brokerPort = 1883

if not brokerIP:
    raise ValueError("Broker IP address is not set. Please set the brokerIP variable.")

global server, config
global sensors

script_dir = os.path.dirname(os.path.abspath(__file__))
config_path = os.path.join(script_dir, 'sensor_config.json')

with open(config_path, 'r') as f:
    config = json.load(f)

sensors = {}

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

class SimulationSynchronization(object):
    def __init__(self, vissim_simulation, carla_simulation, args):
        self.args = args

        self.vissim = vissim_simulation
        self.carla = carla_simulation

        BridgeHelper.blueprint_library = self.carla.world.get_blueprint_library()
        dir_path = os.path.dirname(os.path.realpath(__file__))
        with open(os.path.join(dir_path, 'data', 'vtypes.json')) as f:
            BridgeHelper.vtypes = json.load(f)

        settings = self.carla.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = None
        self.carla.world.apply_settings(settings)
        
    def tick(self):
        # Spawn vehicles.
        for vissim_actor_id, vissim_actor in self.vissim.vehicles.items():
            if self.carla.vehicle_ids.get(vissim_actor_id, None) is not None:
                continue
            
            print(f'Spawning carla actor for vissim vehicle {vissim_actor_id}')
            carla_actor = CarlaActor(self.carla, "vehicle", vissim_actor, vissim_actor_id)
            if carla_actor.carla_id != INVALID_ACTOR_ID:
                self.carla.actors[carla_actor.carla_id] = carla_actor

        # Spawn pedestrians.
        for vissim_actor_id, vissim_actor in self.vissim.pedestrians.items():
            if self.carla.walker_ids.get(vissim_actor_id, None) is not None:
                continue
            
            print(f'Spawning carla actor for vissim pedestrian {vissim_actor_id}')
            carla_actor = CarlaActor(self.carla, "walker", vissim_actor, vissim_actor_id)
            if carla_actor.carla_id != INVALID_ACTOR_ID:
                self.carla.actors[carla_actor.carla_id] = carla_actor

        # Synchronize actors.
        for carla_id, carla_actor in list(self.carla.actors.items()):
            if getattr(self.carla, carla_actor.type + '_ids').get(carla_actor.vissim_id, None) is None:
                continue

            list_name = "vehicles" if carla_actor.type == "vehicle" else "pedestrians"
            vissim_list = getattr(self.vissim, list_name)

            if vissim_list.get(carla_actor.vissim_id, None) is None:
                print(f'Destroying carla actor {carla_id} for missing vissim actor {carla_actor.vissim_id}')
                carla_actor.destroy()
            else:
                vissim_actor = vissim_list[carla_actor.vissim_id]
                carla_actor.synchronize(vissim_actor)

        for name, state in self.vissim.lights_state.items():
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
                transform = actor.get_transform()
                if (transform.location*100).distance(location) < 1:
                        actor.set_state(state)
        
    def close(self):
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.carla.world.apply_settings(settings)

        for carla_actor in list(self.carla.actors.values()):
            carla_actor.destroy()

        self.vissim.close()


def synchronization_loop(args):
    carla_simulation = CarlaSimulation(args)
    vissim_simulation = PTVVissimSimulation(args)

    client = mqttClient.Client(mqttClient.CallbackAPIVersion.VERSION1, "Carla_Sensors", clean_session=True)
    client.connect(brokerIP, brokerPort)

    rate = 1/args.refresh_rate

    for sensor_cfg in config['sensors']:
        try:
            sensor = CARLASensor(sensor_cfg, client, carla_simulation.world)
            print(f"Spawned sensor: {str(sensor.name)}")
        except Exception as e:
            print(f"Error spawning sensor: {e}")

    try:
        synchronization = SimulationSynchronization(vissim_simulation, carla_simulation, args)
        last_tick = time.time()
        while True:
            current_tick = time.time()

            if current_tick - last_tick >= rate:
                vissim_simulation.tick()
                carla_simulation.tick()
                synchronization.tick()
                last_tick = current_tick

    except KeyboardInterrupt:
        logging.info('Cancelled by user.')

    finally:
        logging.info('Cleaning synchronization')
        for sensor in sensors.values():
            sensor.sensor.destroy()
        synchronization.close()



if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--vissim-network',
                           default='examples/NewarkNoGeo',
                           type=str,
                           help='vissim network folder')
    argparser.add_argument('--carla-host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the carla host server (default: 127.0.0.1)')
    argparser.add_argument('--carla-port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--vissim-version',
                           default=2020,
                           type=int,
                           help='ptv-vissim version (default: 2020)')
    argparser.add_argument('--refresh-rate',
                           default=2,
                           type=int,
                           help='rate at which CARLA refreshes actor positions (default: 2Hz)')
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    arguments = argparser.parse_args()

    if arguments.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    synchronization_loop(arguments)
