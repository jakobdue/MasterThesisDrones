# -*- coding: utf-8 -*-
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2016 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Simple example that connects to one crazyflie (check the address at the top
and update it to your crazyflie address) and send a sequence of setpoints,
one every 5 seconds.

This example is intended to work with the Loco Positioning System in TWR TOA
mode and with the Lighthouse Positioning System. It aims at documenting how
to set the Crazyflie in position control mode and how to send setpoints.
"""
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator
from cflib.crazyflie.swarm import CachedCfFactory
from functools import partial


from cflib.crazyflie.swarm import Swarm
# URI to the Crazyflie to connect to
uri1 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E703')
uri2 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E708')

position_params = {
    uri1: [0],
    uri2: [1]
    }

uris = [uri1, uri2]

# Variables:
current_vec_lenght = 0.35
eta_safety_distance = 0.7  # meters
activation_distance = 0.5  # meters
k_stiffness = 6.0
force_scaler = 1.0


# Change the sequence according to your setup
#             x    y    z  YAW

sequences = {
    0: [
    (1.0, -1.0, 0.7, 0),
    (1.0, -1.0, 0.7, 0),
    (-1.0, 1.0, 0.7, 0),
    (-1.0, 1.0, 0.4, 0),
    (-1.0, 1.0, 0.0, 0)
    ],
    1: [
    ( 1.0,  1.0, 0.7, 0),
    ( 1.0,  1.0, 0.7, 0),
    (-1.0, -1.0, 0.7, 0),
    (-1.0, -1.0, 0.4, 0),
    (-1.0, -1.0, 0.0, 0),
    ]
    }
    
curr_pos = {uri1: (0, 0, 0),
            uri2: (0, 0, 0)
            }

def quadratic_penalty_g(pos1, pos2, eta_safety_distance):
    dx = pos2[0] - pos1[0]
    dy = pos2[1] - pos1[1]
    dz = pos2[2] - pos1[2]

    distance = (dx**2 + dy**2 + dz**2)**0.5

    return ((dx/distance, dy/distance, dz/distance), distance - eta_safety_distance)


"""
def barrier_force(pos1, pos2, eta_safety_distance, k_stiffness, activation_distance):
    vec, g = quadratic_penalty_g(pos1, pos2, eta_safety_distance)

    if g <= 0:
        print(f'COLLISION IMMINENT! g={g:.2f}')
        val = -(1/2) * k_stiffness * g
        return (vec[0] * val, vec[1] * val, vec[2] * val)

    elif g >= activation_distance:
        print(f"No barrier force. g={g:.2f}")
        return (0, 0, 0)

    else:
        print(f'Barrier active! g={g:.2f}')
        val = -eta_safety_distance * (1/g) 
        return (vec[0] * val, vec[1] * val, vec[2] * val)
         
"""

def barrier_force(pos1, pos2, eta_safety_distance, k_stiffness, activation_distance):
    vec, g = quadratic_penalty_g(pos1, pos2, eta_safety_distance)

    if g <= 0:
        print(f'COLLISION IMMINENT! g={g:.2f}')
        val = -(1/2) * k_stiffness * g**2
        return (vec[0] * val, vec[1] * val, vec[2] * val)

    else: #g > 0:
        print(f"No barrier force. g={g:.2f}")
        return (0, 0, 0)
         




def limit_velocity(cf, xy=0.25, z=0.20):
    # m/s
    cf.param.set_value('posCtlPid.xVelMax', xy)
    cf.param.set_value('posCtlPid.yVelMax', xy)
    cf.param.set_value('posCtlPid.zVelMax', z)

def take_off(cf, position):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position[2] / take_off_time

    print(f'take off at {position[2]}')

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)


def position_callback(uri, timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    curr_pos[uri] = (x, y, z)
    print('pos: ({}, {}, {})'.format(x, y, z))


def start_position_printing(scf):
    uri = scf.cf.link_uri

    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)

    # Bind uri into the callback
    log_conf.data_received_cb.add_callback(partial(position_callback, uri))

    log_conf.start()


def run_sequence(scf, num_seq):
    cf = scf.cf

    limit_velocity(cf, xy=0.35, z=0.40)
    #limit_tilt(cf, deg=10.0)

    # Arm the Crazyflie
    cf.platform.send_arming_request(True)
    time.sleep(1.0)

    take_off(cf, sequences[num_seq][0])
    if num_seq == 0:
        me = uri1
        other = uri2
    else:
        me = uri2
        other = uri1

    time.sleep(1.0)

    for position in sequences[num_seq]:
        print('Setting position {}'.format(position))
        for i in range(40):
            force = barrier_force(curr_pos[me], curr_pos[other], eta_safety_distance, k_stiffness=k_stiffness, activation_distance=activation_distance)
            print(f'Calculated barrier force: {force}')
            if force != (0, 0, 0):
                
                current_vec = (position[0] - curr_pos[me][0], position[1] - curr_pos[me][1], position[2] - curr_pos[me][2])
                #print(f'Current vector to target: {current_vec}')
                length = ((current_vec[0]**2 + current_vec[1]**2 + current_vec[2]**2)**0.5)
                if length > 0:
                    current_vec = (current_vec[0] / length * current_vec_lenght, current_vec[1] / length * current_vec_lenght, current_vec[2] / length * current_vec_lenght)
                print(f'Scaled current vector to target: {current_vec}')

                # scLE THE FORCE 
                force = (force[0] * force_scaler, force[1] * force_scaler, force[2] * force_scaler)
                
                cf.commander.send_velocity_world_setpoint(force[0] + current_vec[0], force[1] + current_vec[1], force[2] + current_vec[2], 0)
                
                #cf.commander.send_velocity_world_setpoint(force[0], force[1], force[2], 0)
            else:
                cf.commander.send_position_setpoint(position[0],
                                                    position[1],
                                                    position[2],
                                                    position[3])
            # if space bar pressed then send stop
            #if (input() == ' '):
            #    print('Stopping')
            #    cf.commander.send_stop_setpoint()
            #    return



            time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(start_position_printing)
        swarm.parallel_safe(run_sequence, args_dict=position_params)






























