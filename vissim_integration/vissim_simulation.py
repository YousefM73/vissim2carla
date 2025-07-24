#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
""" This module is responsible for the management of the ptv-vissim simulation. """

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import enum
import logging
import math
import os

import carla  # pylint: disable=import-error
import win32com.client as com
from ctypes import *

from . import constants

# ==================================================================================================
# -- vissim definitions ----------------------------------------------------------------------------
# ==================================================================================================

class VissimLightState(enum.Enum):
    """
    VissimLightState contains the different vissim indicator states.
    """
    LEFT = 1
    NO = 0
    RIGHT = -1

def SignalState(signal_state):
    if signal_state == "RED":
        return carla.TrafficLightState.Red
    elif signal_state == "AMBER":
        return carla.TrafficLightState.Yellow
    elif signal_state == "GREEN":
        return carla.TrafficLightState.Green
    elif signal_state == "OFF":
        return carla.TrafficLightState.Off
    else:
        return carla.TrafficLightState.Unknown

class VissimVehicle(object):
    """
    VissimVehicle holds the data relative to traffic vehicles in vissim.
    """
    def __init__(self,
                 vehicle_id,
                 type_id,
                 model_filename,
                 color,
                 location,
                 rotation,
                 velocity,
                 lights_state=VissimLightState.NO):
        # Static parameters.
        self.id = vehicle_id
        self.type = type_id
        self.model_filename = model_filename
        self.color = color

        # Dynamic attributes.
        loc = carla.Location(location[0], location[1], location[2])
        rot = carla.Rotation(math.degrees(rotation[0]), math.degrees(rotation[1]),
                             math.degrees(rotation[2]))
        self._transform = carla.Transform(loc, rot)
        self._velocity = carla.Vector3D(
            velocity * math.cos(math.radians(rot.yaw)) * math.cos(math.radians(rot.pitch)),
            velocity * math.sin(math.radians(rot.yaw)) * math.cos(math.radians(rot.pitch)),
            velocity * math.sin(math.radians(rot.pitch)))
        self._lights_state = lights_state

    def get_velocity(self):
        return self._velocity

    def get_transform(self):
        return self._transform

class VissimPedestrian(object):
    """
    VissimPedestrian holds the data relative to traffic pedestrians in vissim.
    """
    def __init__(self, pedestrian_id, type_id, position, rotation, speed, move_direction):
        self.id = pedestrian_id
        self.type = type_id
        self.move_direction = move_direction
        
        self.position = carla.Location(position[0], position[1], position[2])
        self.rotation = carla.Rotation(math.degrees(rotation[0]), math.degrees(rotation[1]),
                             math.degrees(rotation[2]))
        self._speed = speed
    def get_velocity(self):
        return carla.Vector3D(self._speed * self.move_direction[0],
                               self._speed * self.move_direction[1],
                               self._speed * self.move_direction[2])

    def get_transform(self):
        return carla.Transform(self.position, self.rotation)

# ==================================================================================================
# -- vissim simulation -----------------------------------------------------------------------------
# ==================================================================================================

def calculate_pitch(object):
    front_x = object.AttValue('CoordFrontX')
    front_y = object.AttValue('CoordFrontY')
    front_z = object.AttValue('CoordFrontZ')
    rear_x = object.AttValue('CoordRearX')
    rear_y = object.AttValue('CoordRearY')
    rear_z = object.AttValue('CoordRearZ')
            
    horizontal_distance = math.sqrt((front_x - rear_x)**2 + (front_y - rear_y)**2)
    height_difference = front_z - rear_z
    if horizontal_distance > 0:
        pitch = math.atan2(height_difference, horizontal_distance)
    else:
        pitch = 0.0
    
    return pitch

class PTVVissimSimulation(object):
    def __init__(self, args):
        
        logging.info('Establishing a connection with a GUI version of PTV-Vissim')
        self.proxy = com.gencache.EnsureDispatch("Vissim.Vissim")
        folder = os.path.dirname(os.path.dirname(__file__))
        network_path = os.path.join(folder, args.vissim_network)
        network_file = None
        for root, dirs, files in os.walk(network_path):
            for file in files:
                if file.lower().endswith('.inpx'):
                    network_file = os.path.join(root, file)

        self.proxy.LoadNet(network_file, False)

        self.vehicles = {}
        self.pedestrians = {}

    def get_actor(self, actor_id):
        return self.vehicles[actor_id] if actor_id in self.vehicles else self.pedestrians[actor_id] if actor_id in self.pedestrians else None

    def tick(self):
        self.proxy.Simulation.RunSingleStep()

        vehicles = {}
        for veh in self.proxy.Net.Vehicles:
            veh_id = veh.AttValue('No')
            yaw = math.radians(veh.AttValue('OrientationAngle'))
            pitch = calculate_pitch(veh)

            front_x = veh.AttValue('CoordFrontX')
            front_y = veh.AttValue('CoordFrontY')
            front_z = veh.AttValue('CoordFrontZ')

            vehicles[veh_id] = VissimVehicle(
                veh_id, veh.AttValue('VehType'), veh.AttValue('VehType'),
                veh.AttValue('Color1'),
                [front_x, front_y, front_z],
                [pitch, yaw, 0], veh.AttValue('Speed'),
                veh.AttValue("Indicating"))

        self.vehicles = vehicles
        
        pedestrians = {}
        for ped in self.proxy.Net.Pedestrians:
            ped_id = ped.AttValue('No')
            yaw = math.radians(ped.AttValue('OrientationAngle'))

            front_x = ped.AttValue('CoordFrontX')
            front_y = ped.AttValue('CoordFrontY')
            front_z = ped.AttValue('CoordFrontZ')
            center_x = ped.AttValue('CoordCentX')
            center_y = ped.AttValue('CoordCentY')
            center_z = ped.AttValue('CoordCentZ')

            move_direction_x = front_x - center_x
            move_direction_y = front_y - center_y
            move_direction_z = front_z - center_z

            pedestrians[ped_id] = VissimPedestrian(
                ped_id, ped.AttValue('PedType'),
                [center_x, center_y, center_z],
                [0, yaw, 0], ped.AttValue('Speed'),
                [move_direction_x, move_direction_y, move_direction_z])

        self.pedestrians = pedestrians

        for signal_controller in self.proxy.Net.SignalControllers:
            for signal_group in signal_controller.SGs:
                state = signal_group.AttValue('SigState')
                name = signal_group.AttValue('Name')
                for carla_light in name.split(','):
                    return
                    #self.lights_state[carla_light] = SignalState(state)

    def close(self):
        return
