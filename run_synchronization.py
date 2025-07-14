#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Script to co-simulate CARLA and PTV-Vissim.
"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import argparse
import json
import logging
import time

import carla  # pylint: disable=import-error

# ==================================================================================================
# -- find carla module -----------------------------------------------------------------------------
# ==================================================================================================

import glob
import os
import sys

try:
    sys.path.append(
        glob.glob('../../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==================================================================================================
# -- vissim integration imports --------------------------------------------------------------------
# ==================================================================================================

from vissim_integration.bridge_helper import BridgeHelper
from vissim_integration.carla_simulation import CarlaSimulation
from vissim_integration.vissim_simulation import PTVVissimSimulation
from vissim_integration.constants import INVALID_ACTOR_ID

# ==================================================================================================
# -- synchronization_loop --------------------------------------------------------------------------
# ==================================================================================================

class SimulationSynchronization(object):
    """
    SimulationSynchronization class is responsible for the synchronization of ptv-vissim and carla
    simulations.
    """
    def __init__(self, vissim_simulation, carla_simulation, args):
        self.args = args

        self.vissim = vissim_simulation
        self.carla = carla_simulation

        self.carla2vissim_ids = {}  # Contains only actors controlled by carla.
        self.vissim2carla_ids = {}  # Contains only actors controlled by vissim.

        BridgeHelper.blueprint_library = self.carla.world.get_blueprint_library()
        dir_path = os.path.dirname(os.path.realpath(__file__))
        with open(os.path.join(dir_path, 'data', 'vtypes.json')) as f:
            BridgeHelper.vtypes = json.load(f)

        # Configuring carla simulation in sync mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = args.step_length
        self.carla.world.apply_settings(settings)
        
    def tick(self):
        # -------------------
        # vissim-->carla sync
        # -------------------
        self.vissim.tick()
        self.carla.world.tick()

        # Spawning vissim controlled vehicles in carla.
        for vissim_actor_id, vissim_actor in self.vissim.vehicles.items():
            carla_actor_id = self.vissim2carla_ids.get(vissim_actor_id, None)
            if carla_actor_id is None:
                carla_blueprint = BridgeHelper.get_carla_blueprint(vissim_actor)
                carla_transform = BridgeHelper.get_carla_transform(vissim_actor.get_transform())
                carla_actor_id = self.carla.spawn_actor(carla_blueprint, carla_transform)

                if carla_actor_id != INVALID_ACTOR_ID:
                    self.carla.actors.append(carla_actor_id)
                    self.carla2vissim_ids[carla_actor_id] = vissim_actor_id
                    self.vissim2carla_ids[vissim_actor_id] = carla_actor_id

        # Destroying vissim controlled vehicles in carla.
        for carla_id in self.carla.actors:
            vissim_id = self.carla2vissim_ids.get(carla_id, None)
            if not self.vissim.vehicles.get(vissim_id):
                print(f'Destroying carla actor {carla_id} for missing vissim vehicle {vissim_id}')
                self.carla.destroy_actor(carla_id)
                self.carla2vissim_ids.pop(carla_id)
                self.vissim2carla_ids.pop(vissim_id)

        # Updating vissim controlled vehicles in carla.
        for carla_actor_id in self.carla.actors:
            vissim_actor_id = self.carla2vissim_ids.get(carla_actor_id, None)
            if vissim_actor_id is not None:
                vissim_actor = self.vissim.get_actor(vissim_actor_id)
                carla_actor = self.carla.get_actor(carla_actor_id)

                carla_transform = BridgeHelper.get_carla_transform(vissim_actor.get_transform(),
                                                               carla_actor.bounding_box.extent)
                carla_velocity = BridgeHelper.get_carla_velocity(vissim_actor.get_velocity())
                self.carla.synchronize_vehicle(carla_actor_id, carla_transform, carla_velocity)

        #for key in self.vissim.lights_state:
            #value = self.vissim.lights_state[key]
            #actor = self.carla.world.get_actor(int(key))
            #actor.set_state(value)
        
    def close(self):
        """
        Cleans synchronization.
        """
        # Configuring carla simulation in async mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.carla.world.apply_settings(settings)

        # Destroying synchronized actors.
        for carla_actor_id in self.carla2vissim_ids.keys():
            self.carla.destroy_actor(carla_actor_id)

        # Closing PTV-Vissim connection.
        self.vissim.close()


def synchronization_loop(args):
    """
    Entry point for vissim-carla co-simulation.
    """
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


# ==================================================================================================
# -- main ------------------------------------------------------------------------------------------
# ==================================================================================================

if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--vissim-network',
                           default='examples/Basic4Way',
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
