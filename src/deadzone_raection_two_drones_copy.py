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
import math
import time


from cflib.crazyflie.swarm import Swarm
# URI to the Crazyflie to connect to
uri1 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E708')
uri2 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E706')

# Change the sequence according to your setup
#             x    y    z  YAW


def move_to_position(cf, target, v_max=0.3, dt=0.05, tol=0.02):
    """
    cf: Crazyflie object
    target: (x, y, z, yaw)
    v_max: max linear speed [m/s]
    dt: control period [s]
    tol: position tolerance [m]
    """

    while True:
        state = cf.state_estimate
        dx = target[0] - state.x
        dy = target[1] - state.y
        dz = target[2] - state.z

        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist < tol:
            break

        scale = min(v_max / dist, 1.0)
        vx = dx * scale
        vy = dy * scale
        vz = dz * scale

        cf.commander.send_velocity_world_setpoint(vx, vy, vz, 0)
        time.sleep(dt)

    cf.commander.send_velocity_world_setpoint(0, 0, 0, 0)


sequences = {
    0: [
    (-1.0, -1.0, 0.4, 0),
    (-1.0, -1.0, 0.6, 0),
    (1.0, 1.0, 0.6, 0),
    (1.0, 1.0, 0.4, 0),
    ],
    1: [
    ( 1.0,  1.0, 0.4, 0),
    ( 1.0,  1.0, 0.6, 0),
    (-1.0, -1.0, 0.6, 0),
    (-1.0, -1.0, 0.4, 0),
    ]
    }
    
curr_pos = {uri1: (0, 0, 0),
            uri2: (0, 0, 0)
            }


def about2collide(uri, pos1, pos2, safety_distance):
    temp_curr_pos = curr_pos.copy()
    curr_pos.update({uri: pos1})
    
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    dz = pos1[2] - pos2[2]
    distance = (dx**2 + dy**2 + dz**2)**0.5
    return distance < safety_distance


def setup_state_logging(cf):
    logconf = LogConfig(name='State', period_in_ms=50)
    logconf.add_variable('kalman.stateX', 'float')
    logconf.add_variable('kalman.stateY', 'float')
    logconf.add_variable('kalman.stateZ', 'float')

    cf.state_estimate = type('', (), {})()

    def cb(ts, data, _):
        cf.state_estimate.x = data['kalman.stateX']
        cf.state_estimate.y = data['kalman.stateY']
        cf.state_estimate.z = data['kalman.stateZ']

    logconf.data_received_cb.add_callback(cb)
    cf.log.add_config(logconf)
    logconf.start()

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
    global curr_pos
    curr_pos[uri] = (x, y, z)
    print('pos: ({}, {}, {})'.format(x, y, z))


def start_position_printing(scf):
    uri = scf.cf.link_uri
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback, uri)
    log_conf.start()


def run_sequence(scf, num_seq):
    cf = scf.cf

    # Arm the Crazyflie
    cf.platform.send_arming_request(True)
    time.sleep(1.0)

    take_off(cf, sequences[num_seq][0])
    time.sleep(1.0)

    for position in sequences[num_seq]:
        move_to_position(cf, position, v_max=0.25)    
        
        
        """ print('Setting position {}'.format(position))
        for i in range(50):
            cf.commander.send_position_setpoint(position[0],
                                                position[1],
                                                position[2],
                                                position[3])

            if about2collide(num_seq, curr_pos[0], curr_pos[1], safety_distance):
                print('Warning: Drones are too close! Changing movement.')

                if num_seq == 0:
                    cf.commander.send_position_setpoint(curr_pos[uri1][0], curr_pos[uri1][1], curr_pos[uri1][2], 0.0)
                else:
                    cf.commander.send_position_setpoint(curr_pos[uri2][0], curr_pos[uri2][1], curr_pos[uri2][2], 0.0)
            time.sleep(0.1) """

    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    uris = [uri1, uri2]
    #sequences = [sequence1, sequence2]
    position_params = {
        uri1: [0],
        uri2: [1]
        }


    safety_distance = 0.6  # meters

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(start_position_printing)
        swarm.parallel_safe(setup_state_logging)
        swarm.parallel_safe(run_sequence, args_dict=position_params)






























