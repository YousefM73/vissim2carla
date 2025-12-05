import json
import os
import math
import tempfile

import win32com.client as com
import carla

import argparse
import logging


# // Helpers
def signal_state(signal_state):
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

def calculate_pitch(object):
    front_x, front_y, front_z = object.AttValue('CoordFrontX'), object.AttValue('CoordFrontY'), object.AttValue('CoordFrontZ')
    rear_x, rear_y, rear_z = object.AttValue('CoordRearX'), object.AttValue('CoordRearY'), object.AttValue('CoordRearZ')
            
    horizontal_distance = math.sqrt((front_x - rear_x)**2 + (front_y - rear_y)**2)
    height_difference = front_z - rear_z
    if horizontal_distance > 0:
        pitch = math.atan2(height_difference, horizontal_distance)
    else:
        pitch = 0.0
    
    return pitch


# // Simulation
class Simulation(object):
    def __init__(self):

        logging.info('Establishing a connection with a GUI version of PTV-Vissim')
        self.proxy = com.gencache.EnsureDispatch("Vissim.Vissim")

        primary_folder = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        folder = os.path.join(primary_folder, 'network')

        network_file = next((os.path.join(folder, f) for f in os.listdir(folder) if f.lower().endswith('.inpx')), None)
        self.proxy.LoadNet(network_file, False)

        self.vehicles = {}
        self.pedestrians = {}
        self.signal_states = {}

    def tick(self):
        self.proxy.Simulation.RunSingleStep()

        vehicles = {}
        for veh in self.proxy.Net.Vehicles:
            veh_id = veh.AttValue('No')
            yaw = math.radians(veh.AttValue('OrientationAngle'))
            pitch = calculate_pitch(veh)

            front_x, front_y, front_z = veh.AttValue('CoordFrontX'), veh.AttValue('CoordFrontY'), veh.AttValue('CoordFrontZ')

            vehicles[veh_id] = {
                'id': veh_id,
                'type': veh.AttValue('VehType'),
                'model_filename': veh.AttValue('VehType'),
                'color': veh.AttValue('Color1'),
                'location': [front_x, front_y, front_z],
                'rotation': [pitch, yaw, 0],
                'lights_state': veh.AttValue("Indicating")
            }

        self.vehicles = vehicles

        pedestrians = {}
        for ped in self.proxy.Net.Pedestrians:
            ped_id = ped.AttValue('No')
            yaw = math.radians(ped.AttValue('OrientationAngle'))

            front_x, front_y, front_z = ped.AttValue('CoordFrontX'), ped.AttValue('CoordFrontY'), ped.AttValue('CoordFrontZ')
            center_x, center_y, center_z = ped.AttValue('CoordCentX'), ped.AttValue('CoordCentY'), ped.AttValue('CoordCentZ')

            move_direction_x, move_direction_y, move_direction_z = front_x - center_x, front_y - center_y, front_z - center_z

            pedestrians[ped_id] = {
                'id': ped_id,
                'type': ped.AttValue('PedType'),
                'location': [center_x, center_y, center_z],
                'rotation': [0, yaw, 0],
                'move_direction': [move_direction_x, move_direction_y, move_direction_z],
                'speed': ped.AttValue('Speed'),
            }

        self.pedestrians = pedestrians

        for signal_head in self.proxy.Net.SignalHeads:
            name = signal_head.AttValue('Name')
            state = signal_head.AttValue('SigState')
            self.signal_states[name] = signal_state(state)


# // Main Loop
def main():
    data_file = os.path.join(tempfile.gettempdir(), 'vissim_data.json')
    try:
        sim = Simulation()
        while True:
            if os.path.exists(os.path.join(tempfile.gettempdir(), 'vissim_terminate.flag')):
                os.remove(os.path.join(tempfile.gettempdir(), 'vissim_terminate.flag'))
                break
            
            sim.tick()
            
            sim_data = {
                'vehicles': sim.vehicles,
                'pedestrians': sim.pedestrians,
                'signals': sim.signal_states
            }

            temp_file = data_file + '.tmp'
            with open(temp_file, 'w') as f:
                json.dump(sim_data, f)
            try:
                os.replace(temp_file, data_file)
            except OSError:
                pass

    except KeyboardInterrupt:
        logging.info('Cancelled by user.')

    except:
        logging.exception('An error occurred during the simulation.')


# // Initialization
if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    arguments = argparser.parse_args()

    if arguments.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    main()