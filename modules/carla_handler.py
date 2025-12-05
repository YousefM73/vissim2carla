#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import logging
import io
import os
import math

import carla  # pylint: disable=import-error
import random

from .bridge_helper import BridgeHelper

def get_transform(vissim_actor_data):
    location = vissim_actor_data['location']
    rotation = vissim_actor_data['rotation']
    carla_location = carla.Location(x=location[0], y=location[1], z=location[2])
    carla_rotation = carla.Rotation(pitch=math.degrees(rotation[0]), yaw=math.degrees(rotation[1]), roll=math.degrees(rotation[2]))
    return carla.Transform(carla_location, carla_rotation)

class CarlaActor(object):
    def __init__(self, CarlaSimulation, type, vissim_actor_data):
        self.type = type
        self.vissim_id = str(vissim_actor_data['id'])
        self.simulation = CarlaSimulation
        
        carla_blueprint = BridgeHelper.get_carla_blueprint(vissim_actor_data) if type == 'vehicle' else random.choice(BridgeHelper.blueprint_library.filter('walker'))
        carla_transform = BridgeHelper.get_carla_transform(get_transform(vissim_actor_data))
        self.carla_id = self.simulation.spawn_actor(carla_blueprint, carla_transform, type)

        getattr(self.simulation, type + '_ids')[self.vissim_id] = self.carla_id

    def synchronize(self, vissim_actor):
        carla_actor = self.simulation.get_actor(self.carla_id)
        carla_transform = BridgeHelper.get_carla_transform(get_transform(vissim_actor), carla_actor.bounding_box.extent)

        if self.type == 'vehicle':
            self.simulation.synchronize_vehicle(self.carla_id, carla_transform)
        elif self.type == 'walker':
            self.simulation.synchronize_pedestrian(self.carla_id, carla_transform, vissim_actor['move_direction'], vissim_actor['speed'])

    def destroy(self):
        self.simulation.destroy_actor(self.carla_id)

class CarlaSimulation(object):
    def __init__(self, config, map):
        self.client = carla.Client(config["host"], config["port"])
        self.client.set_timeout(config["timeout"])

        map = os.path.basename(os.path.normpath(map))
        self.world = self.client.load_world(map)
        self.blueprint_library = self.world.get_blueprint_library()

        self.vehicle_ids = {}
        self.walker_ids = {}

        self.actors = {}
        self.controllers = {}
        self.batch = []

        self.walker_spawn_time = {}

    def tick(self):
        self.client.apply_batch(self.batch)
        self.batch.clear()

        self.world.tick()

    def get_actor(self, actor_id):
        return self.world.get_actor(actor_id)

    def spawn_actor(self, blueprint, transform, type):
        transform = carla.Transform(transform.location + carla.Location(0, 0, 5), transform.rotation)
        timestamp = (self.world.get_snapshot()).timestamp

        batch = [
            carla.command.SpawnActor(blueprint, transform).then(
                carla.command.SetSimulatePhysics(carla.command.FutureActor, False))
        ]

        response = self.client.apply_batch_sync(batch, False)[0]
        actor_id = response.actor_id
        self.walker_spawn_time[actor_id] = timestamp

        if response.error:
            logging.error('Spawn carla actor failed. %s', response.error)
            return -1
        else:
            if type == 'walker':
                walker = self.world.get_actor(actor_id)

                if walker is None:
                    logging.error('Walker actor %s not found after spawn', actor_id)
                    return -1

                walker_bp = self.world.get_blueprint_library().find('controller.ai.walker')
                batch = [
                    carla.command.SpawnActor(walker_bp, carla.Transform(), actor_id)
                ]
                response = self.client.apply_batch_sync(batch, True)[0]

                if response.error:
                    logging.error('Spawn carla walker failed. %s', response.error)
                    return -1
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
                            return -1
        
        return actor_id

    def destroy_actor(self, actor_id):
        actor = self.world.get_actor(actor_id)
        if self.actors.get(actor_id) is not None:
            del self.actors[actor_id]
        
        if actor_id in self.controllers:
            controller_id = self.controllers.get(actor_id)
            controller = self.world.get_actor(controller_id)
            if controller is not None:
                controller.stop()
                controller.destroy()
            del self.controllers[actor_id]

        if actor is not None:
            return actor.destroy()
        return False

    def synchronize_vehicle(self, vehicle_id, transform, lights=None):
        vehicle = self.world.get_actor(vehicle_id)
        if vehicle is None:
            return False

        self.batch.append(carla.command.ApplyTransform(vehicle_id, transform))

        if lights is not None:
            vehicle.set_light_state(carla.VehicleLightState(lights))

        return True
    
    def synchronize_pedestrian(self, pedestrian_id, transform, direction, speed):
        pedestrian = self.world.get_actor(pedestrian_id)

        if pedestrian is None:
            return False

        control = carla.WalkerControl()
        control.speed = speed/3.6
        direction = [direction[0] / control.speed, direction[1] / control.speed, direction[2] / control.speed] if control.speed > 0 else [0, 0, 0]
        control.direction.x, control.direction.y, control.direction.z = direction[0], direction[1], direction[2]
        
        controller_id = self.controllers.get(pedestrian_id)
        
        if not hasattr(self, 'ground_check_counter'):
            self.ground_check_counter = {}
            self.last_ground_z = {}
        
        if pedestrian_id not in self.ground_check_counter:
            self.ground_check_counter[pedestrian_id] = 0
            # Do initial ground projection to avoid sinking
            start = transform.location + carla.Location(0, 0, 5)
            direction_vec = carla.Vector3D(0, 0, -1)
            intersection = self.world.project_point(start, direction_vec, 10)
            if intersection is not None:
                self.last_ground_z[pedestrian_id] = intersection.location.z + pedestrian.bounding_box.extent.z
            else:
                self.last_ground_z[pedestrian_id] = transform.location.z + pedestrian.bounding_box.extent.z
        
        self.ground_check_counter[pedestrian_id] += 1
        
        if self.ground_check_counter[pedestrian_id] % 10 == 0:
            start = transform.location + carla.Location(0, 0, 5)
            direction_vec = carla.Vector3D(0, 0, -1)
            intersection = self.world.project_point(start, direction_vec, 10)
            
            if intersection is not None:
                ground_z = intersection.location.z + pedestrian.bounding_box.extent.z
                self.last_ground_z[pedestrian_id] = ground_z
                new_transform = carla.Transform(carla.Location(transform.location.x, transform.location.y, ground_z), transform.rotation)
            else:
                new_transform = carla.Transform(carla.Location(transform.location.x, transform.location.y, self.last_ground_z[pedestrian_id]), transform.rotation)
        else:
            new_transform = carla.Transform(carla.Location(transform.location.x, transform.location.y, self.last_ground_z[pedestrian_id]), transform.rotation)

        self.batch.append(carla.command.ApplyTransform(pedestrian_id, new_transform))

        if controller_id is not None:
            controller = self.world.get_actor(controller_id)
            if controller is not None:
                pedestrian.apply_control(control)
                return
            else:
                logging.warning(f'Walker controller {controller_id} not found for walker {pedestrian_id}')

        return True