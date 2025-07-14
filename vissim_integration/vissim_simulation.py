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


class Simulator_Veh_Data(Structure):
    """
    Structure to hold the data sent to vissim about the status of the simulator vehicles (i.e.,
    carla vehicles).
    """
    _fields_ = [
        ('Position_X', c_double),  # front center of the vehicle in m
        ('Position_Y', c_double),  # front center of the vehicle in m
        ('Position_Z', c_double),  # front center of the vehicle in m
        ('Orient_Heading', c_double),  # in radians, eastbound = zero, northbound = +Pi/2 */
        ('Orient_Pitch', c_double),  # in radians, uphill = positive
        ('Speed', c_double),  # in m/s
        ('ControlledByVissim', c_bool),  # affects next time step
        ('RoutingDecisionNo', c_long),  # used once if ControlledByVissim changed from false to true
        ('RouteNo', c_long)  # used once if ControlledByVissim changed from false to true
    ]


class VISSIM_Veh_Data(Structure):
    """
    Structure to hold the data received from vissim about the status of the traffic vehicles (i.e.,
    vissim vehicles).
    """
    _fields_ = [
        ('VehicleID', c_long),
        ('VehicleType', c_long),  # vehicle type number from Vissim
        ('ModelFileName', c_char * constants.NAME_MAX_LENGTH),  # .v3d
        ('color', c_long),  # RGB
        ('Position_X', c_double),  # front center of the vehicle in m
        ('Position_Y', c_double),  # front center of the vehicle in m
        ('Position_Z', c_double),  # front center of the vehicle in m
        ('Orient_Heading', c_double),  # in radians, eastbound = zero, northbound = +Pi/2 */
        ('Orient_Pitch', c_double),  # in radians, uphill = positive
        ('Speed', c_double),  # in m/s
        ('LeadingVehicleID', c_long),  # relevant vehicle in front
        ('TrailingVehicleID', c_long),  # next vehicle back on the same lane
        ('LinkID', c_long),  # Vissim link attribute “Number”
        ('LinkName', c_char * constants.NAME_MAX_LENGTH),  # empty if “Name” not set in Vissim
        ('LinkCoordinate', c_double),  # in m
        ('LaneIndex', c_int),  # 0 = rightmost
        ('TurningIndicator', c_int),  # 1 = left, 0 = none, -1 = right
        ('PreviousIndex', c_long),  # for interpolation: index in the array in the previous Vissim time step, < 0 = new in the visibility area
        ('NumUDAs', c_long),  # the number of UDA values in the following array
        ('UDA', c_double * constants.MAX_UDA)  # the first MAX_UDA user-defined numeric vehicle attributes
    ]


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
        """
        Returns the vehicle's velocity.
        """
        return self._velocity

    def get_transform(self):
        """
        Returns carla transform.
        """
        return self._transform


# ==================================================================================================
# -- vissim simulation -----------------------------------------------------------------------------
# ==================================================================================================


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

    def get_actor(self, actor_id):
        return self.vehicles[actor_id]

    def tick(self):
        self.proxy.Simulation.RunSingleStep()

        vehicles = {}
        for veh in self.proxy.Net.Vehicles:
            veh_id = veh.AttValue('No')
            yaw = math.radians(veh.AttValue('OrientationAngle'))

            front_x = veh.AttValue('CoordFrontX')
            front_y = veh.AttValue('CoordFrontY')
            front_z = veh.AttValue('CoordFrontZ')
            rear_x = veh.AttValue('CoordRearX')
            rear_y = veh.AttValue('CoordRearY')
            rear_z = veh.AttValue('CoordRearZ')
            
            horizontal_distance = math.sqrt((front_x - rear_x)**2 + (front_y - rear_y)**2)
            height_difference = front_z - rear_z
            if horizontal_distance > 0:
                pitch = math.atan2(height_difference, horizontal_distance)
            else:
                pitch = 0.0

            vehicles[veh_id] = VissimVehicle(
                veh_id, veh.AttValue('VehType'), veh.AttValue('VehType'),
                veh.AttValue('Color1'),
                [front_x, front_y, front_z],
                [pitch, yaw, 0], veh.AttValue('Speed'),
                veh.AttValue("Indicating"))

        self.vehicles = vehicles
        
        for signal_controller in self.proxy.Net.SignalControllers:
            for signal_group in signal_controller.SGs:
                state = signal_group.AttValue('SigState')
                name = signal_group.AttValue('Name')
                for carla_light in name.split(','):
                    return
                    #self.lights_state[carla_light] = SignalState(state)

    def close(self):
        return
