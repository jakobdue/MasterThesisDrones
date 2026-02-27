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
import random
import math


from cflib.crazyflie.swarm import Swarm
# URI to the Crazyflie to connect to
uri1 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E706')
uri2 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E705')

# Change the sequence according to your setup
#             x    y    z  YAW

sequences = {
    0: [
    (-1.0, -1.0, 0.4, 0),
    (-1.0, -1.0, 0.6, 0),
    (1.0, 1.0, 0.6, 0),
    (1.0, 1.0, 0.4, 0),
    (1.0, 1.0, 0.0, 0)
    ],
    1: [
    ( 1.0,  1.0, 0.4, 0),
    ( 1.0,  1.0, 0.6, 0),
    (-1.0, -1.0, 0.6, 0),
    (-1.0, -1.0, 0.4, 0),
    (-1.0, -1.0, 0.0, 0),
    ]
    }
    
curr_pos = {uri1: (0, 0, 0),
            uri2: (0, 0, 0)
            }











class Graph:
    def __init__(self, edges):
        self.edges = edges  # dict: node -> list of neighbors

    def adjacent(self, v):
        return self.edges.get(v, [])


def isvalid(node):
    return True  # no obstacles in this simple example


def backtrace(v):
    # here we assume each node stores its parent
    path = []
    while v is not None:
        path.append(v)
        v = v.parent
    return path[::-1]

class Node:
    def __init__(self, name, parent=None):
        self.name = name
        self.parent = parent

    def __eq__(self, other):
        return self.name == other.name

class RRT:
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        self.expand_dis = 0.5
        self.max_iter = 1000
        self.get_random_node = lambda: Node(random.uniform(-5, 5), random.uniform(-5, 5), random.uniform(0, 10))
        def get_nearest_node_index(self, node_list, rnd_node):
            min(range(len(node_list)), key=lambda i: self.distance(node_list[i], rnd_node))
        def steer(self, from_node, to_node):
            """Steer from one node to another, step-by-step."""
            theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
            new_node = Node(from_node.x + self.step_size * math.cos(theta),
                            from_node.y + self.step_size * math.sin(theta))
            new_node.cost = from_node.cost + self.step_size
            new_node.parent = from_node
            return new_node


class Node:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None

def dfs(G, v0, v_goal):
    S = [v0]

    while len(S) > 0:
        # get a node to expand
        v = S.pop()

        # find the path?
        if v == v_goal:
            return backtrace(v)

        # expand the node
        v_adj = G.adjacent(v)

        # add valid new nodes to the front
        for adj in v_adj:
            if isvalid(adj):
                S.append(adj)

    return None


def planning(self):
    """
    rrt path planning
    """

    self.node_list = [self.start]

    for _ in range(self.max_iter):
        # get a node to expand
        rnd_node = self.get_random_node()
        nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
        nearest_node = self.node_list[nearest_ind]

        # expand the node
        new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

        # add valid new nodes to the front
        if self.check_collision_free(new_node):
            self.node_list.append(new_node)

        if close_enough(new_node, self.goal):
            return self.generate_final_course()

    return None  # cannot find path


planner = RRT(Node(0, 0), Node(5, 5))
path = planner.planning()














""" 


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

    limit_velocity(cf, xy=0.25, z=0.20)
    #limit_tilt(cf, deg=10.0)

    # Arm the Crazyflie
    cf.platform.send_arming_request(True)
    time.sleep(1.0)

    take_off(cf, sequences[num_seq][0])
    time.sleep(1.0)

    for position in sequences[num_seq]:
        print('Setting position {}'.format(position))
        for i in range(80):
            if about2collide(curr_pos[uri1], curr_pos[uri2], safety_distance):
                print('Warning: Drones are too close! Changing movement.')
param.set_value('posCtlPid.xVelMax', xy)
                if num_seq == 0:
                    cf.commander.send_position_setpoint(-1, 1, 0.5, 0.0)
                else:
                    cf.commander.send_position_setpoint(1, -1, 0.5, 0.0)
            else:
                cf.commander.send_position_setpoint(position[0],
                                                    position[1],
                                                    position[2],
                                                    position[3])


            time.sleep(0.1)

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


    safety_distance = 1.0  # meters

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(start_position_printing)
        swarm.parallel_safe(run_sequence, args_dict=position_params)








 """





















