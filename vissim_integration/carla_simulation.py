#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import logging
import io
import os

import carla  # pylint: disable=import-error
import random

from .constants import INVALID_ACTOR_ID, CARLA_SPAWN_OFFSET_Z
from .bridge_helper import BridgeHelper

class CarlaActor(object):
    def __init__(self, CarlaSimulation, type, vissim_actor, vissim_actor_id):
        self.type = type
        self.vissim_id = vissim_actor_id
        self.simulation = CarlaSimulation
        
        carla_blueprint = BridgeHelper.get_carla_blueprint(vissim_actor) if type == 'vehicle' else random.choice(BridgeHelper.blueprint_library.filter('walker'))
        carla_transform = BridgeHelper.get_carla_transform(vissim_actor.get_transform())
        self.carla_id = self.simulation.spawn_actor(carla_blueprint, carla_transform, type)

        getattr(self.simulation, type + '_ids')[vissim_actor_id] = self.carla_id

    def synchronize(self, vissim_actor):
        carla_actor = self.simulation.get_actor(self.carla_id)
        carla_transform = BridgeHelper.get_carla_transform(vissim_actor.get_transform(), carla_actor.bounding_box.extent)
        carla_velocity = BridgeHelper.get_carla_velocity(vissim_actor.get_velocity())
        if self.type == 'vehicle':
            self.simulation.synchronize_vehicle(self.carla_id, carla_transform, carla_velocity)
        elif self.type == 'walker':
            self.simulation.synchronize_pedestrian(self.carla_id, carla_transform, carla_velocity)

    def destroy(self):
        self.simulation.destroy_actor(self.carla_id)

class CarlaSimulation(object):
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

        self.vehicle_ids = {}
        self.walker_ids = {}

        self.actors = {}
        self.controllers = {}

    def get_actor(self, actor_id):
        return self.world.get_actor(actor_id)

    def spawn_actor(self, blueprint, transform, type):

        start = transform.location + carla.Location(0, 0, 10)
        direction = carla.Vector3D(0, 0, -1)

        intersection = None #self.world.project_point(start, direction)

        if intersection is not None:
            transform = carla.Transform(intersection.location + carla.Location(0, 0, 5), transform.rotation)

        transform = carla.Transform(transform.location + carla.Location(0, 0, CARLA_SPAWN_OFFSET_Z),
                                    transform.rotation)

        setSimulatePhysics = False
        if type == 'walker':
            setSimulatePhysics = True

        batch = [
            carla.command.SpawnActor(blueprint, transform).then(
                carla.command.SetSimulatePhysics(carla.command.FutureActor, setSimulatePhysics))
        ]

        response = self.client.apply_batch_sync(batch, False)[0]
        actor_id = response.actor_id

        if response.error:
            logging.error('Spawn carla actor failed. %s', response.error)
            return INVALID_ACTOR_ID
        else:
            if type == 'walker':
                walker = self.world.get_actor(actor_id)
                if walker is None:
                    logging.error('Walker actor %s not found after spawn', actor_id)
                    return INVALID_ACTOR_ID

                walker_bp = self.world.get_blueprint_library().find('controller.ai.walker')
                batch = [
                    carla.command.SpawnActor(walker_bp, carla.Transform(), actor_id)
                ]
                response = self.client.apply_batch_sync(batch, True)[0]
                if response.error:
                    logging.error('Spawn carla walker failed. %s', response.error)
                    return INVALID_ACTOR_ID
                else:
                    self.controllers[actor_id] = response.actor_id
                    controller = self.world.get_actor(response.actor_id)
                    if controller:
                        try:
                            controller.start()
                        except RuntimeError as e:
                            logging.error('Failed to start walker controller %s: %s', response.actor_id, e)
                            controller.destroy()
                            walker.destroy()
                            if actor_id in self.controllers:
                                del self.controllers[actor_id]
                            return INVALID_ACTOR_ID
        
        return actor_id

    def destroy_actor(self, actor_id):
        actor = self.world.get_actor(actor_id)
        if self.actors[actor_id] is not None:
            del self.actors[actor_id]
        
        if actor_id in self.controllers:
            controller_id = self.controllers[actor_id]
            controller = self.world.get_actor(controller_id)
            if controller is not None:
                controller.stop()
                controller.destroy()
            del self.controllers[actor_id]

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
    
    def synchronize_pedestrian(self, pedestrian_id, transform, velocity):

        pedestrian = self.world.get_actor(pedestrian_id)

        if pedestrian is None:
            return False

        control = carla.WalkerControl()
        control.speed = velocity.length()
        direction = [velocity.x / control.speed, velocity.y / control.speed, velocity.z / control.speed] if control.speed > 0 else [0, 0, 0]
        control.direction.x = direction[0]
        control.direction.y = direction[1]
        control.direction.z = direction[2]
        
        controller_id = self.controllers.get(pedestrian_id)
        new_transform = carla.Transform(transform.location + carla.Location(0, 0, pedestrian.bounding_box.extent.z), transform.rotation)

        start = transform.location + carla.Location(0, 0, 5)
        direction = carla.Vector3D(0, 0, -1)

        intersection = self.world.project_point(start, direction, 10)

        if intersection is not None:
            new_transform = carla.Transform(intersection.location + carla.Location(0, 0, pedestrian.bounding_box.extent.z), transform.rotation)

        pedestrian.set_transform(new_transform)
        if controller_id is not None:
            controller = self.world.get_actor(controller_id)
            if controller is not None:
                #controller.go_to_location = transform.location
                return
            else:
                logging.warning(f'Walker controller {controller_id} not found for walker {pedestrian_id}')
        #controller = self.get_actor(self.controllers.get(pedestrian_id))
        #controller.go_to_location = transform.location

        # If desync occurs, we can set the transform directly
        #if velocity is not None:
        #    pedestrian.set_target_velocity(velocity)

        return True

    def tick(self):
        self.world.tick()
