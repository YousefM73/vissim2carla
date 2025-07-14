#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
""" This module is responsible for the management of the carla simulation. """

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import logging
import io
import os

import carla  # pylint: disable=import-error

from .constants import INVALID_ACTOR_ID, CARLA_SPAWN_OFFSET_Z

# ==================================================================================================
# -- carla simulation ------------------------------------------------------------------------------
# ==================================================================================================


class CarlaSimulation(object):
    """
    CarlaSimulation is responsible for the management of the carla simulation.
    """
    def __init__(self, args):
        self.args = args
        host = args.carla_host
        port = args.carla_port

        self.client = carla.Client(host, port)
        self.client.set_timeout(2.0)

        map = os.path.basename(os.path.normpath(args.vissim_network))
        print(map)
        self.world = self.client.load_world(map)
        self.blueprint_library = self.world.get_blueprint_library()

        self.actors = []

    def get_actor(self, actor_id):
        return self.world.get_actor(actor_id)

    def spawn_actor(self, blueprint, transform):
        transform = carla.Transform(transform.location + carla.Location(0, 0, CARLA_SPAWN_OFFSET_Z),
                                    transform.rotation)

        batch = [
            carla.command.SpawnActor(blueprint, transform).then(
                carla.command.SetSimulatePhysics(carla.command.FutureActor, False))
        ]

        response = self.client.apply_batch_sync(batch, False)[0]
        
        if response.error:
            logging.error('Spawn carla actor failed. %s', response.error)
            return INVALID_ACTOR_ID

        return response.actor_id

    def destroy_actor(self, actor_id):
        actor = self.world.get_actor(actor_id)
        self.actors.remove(actor_id) if actor_id in self.actors else None
        if actor is not None:
            return actor.destroy()
        return False

    def synchronize_vehicle(self, vehicle_id, transform, velocity, lights=None):
        vehicle = self.world.get_actor(vehicle_id)
        if vehicle is None:
            return False

        vehicle.set_transform(transform)
        if velocity is not None:
            vehicle.set_target_velocity(velocity)

        if lights is not None:
            vehicle.set_light_state(carla.VehicleLightState(lights))
        return True
    
    def tick(self):
        """
        Tick to carla simulation.
        """
        self.world.tick()
