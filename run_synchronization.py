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
        settings.fixed_delta_seconds = args.step_length
        self.carla.world.apply_settings(settings)
        
    def tick(self):
        self.vissim.tick()
        self.carla.world.tick()

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

    try:
        synchronization = SimulationSynchronization(vissim_simulation, carla_simulation, args)

        while True:
            start = time.time()

            synchronization.tick()

            end = time.time()
            elapsed = end - start
            if elapsed < args.step_length:
                time.sleep(args.step_length - elapsed)

    except KeyboardInterrupt:
        logging.info('Cancelled by user.')

    finally:
        logging.info('Cleaning synchronization')
        synchronization.close()



if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--vissim-network',
                           default='examples/NewRewarkNoGeo',
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
    argparser.add_argument('--step-length',
                           default=0.05,
                           type=float,
                           help='set fixed delta seconds (default: 0.05s)')
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    arguments = argparser.parse_args()

    if arguments.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    synchronization_loop(arguments)
