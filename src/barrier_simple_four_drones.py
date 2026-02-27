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
uri1 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E702')
uri2 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E704')
uri3 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E707')
uri4 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E701')

position_params = {
    uri1: [0],
    uri2: [1],
    uri3: [2],
    uri4: [3]
}

uris = [uri1, uri2, uri3, uri4]

# Variables:
num_drones = 4
eta_safety_distance = 0.75  # meters
k_stiffness = 10.0
max_speed = 0.5

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
    ],
    2: [
    ( -1.0,  -1.0, 0.7, 0),
    ( -1.0,  -1.0, 0.7, 0),
    (-1.0, -1.0, 0.7, 0),
    (1.0, 1.0, 0.4, 0),
    (1.0, 1.0, 0.0, 0),
    ],
    3: [
    ( -1.0,  1.0, 0.7, 0),
    ( -1.0,  1.0, 0.7, 0),
    (-1.0, -1.0, 0.7, 0),
    (1.0, -1.0, 0.4, 0),
    (1.0, -1.0, 0.0, 0),
    ] 
}
    
curr_pos = {uri1: (0, 0, 0),
            uri2: (0, 0, 0),
            uri3: (0, 0, 0),
            uri4: (0, 0, 0)
            } 

def add_vecs(vec1, vec2):
    return (vec1[0] + vec2[0], vec1[1] + vec2[1], vec1[2] + vec2[2])

def scale_vec(vec, scaler):
    return (vec[0] * scaler, vec[1] * scaler, vec[2] * scaler)

def sub_vecs(vec1, vec2):
    return (vec1[0] - vec2[0], vec1[1] - vec2[1], vec1[2] - vec2[2])

def vec_length(vec):
    return (vec[0]**2 + vec[1]**2 + vec[2]**2)**0.5

def quadratic_penalty_g(pos1, pos2, eta_safety_distance):
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    dz = pos1[2] - pos2[2]

    distance = (dx**2 + dy**2 + dz**2)**0.5
    if distance == 0:
        return ((0, 0, 0), -eta_safety_distance)  # Avoid division by zero

    return ((dx/distance, dy/distance, dz/distance), distance - eta_safety_distance)

def square_dist(pos1, pos2):
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    dz = pos1[2] - pos2[2]

    return (dx**2 + dy**2 + dz**2)# avoiding sqrt for efficiency, since we only care about relative distances

def barrier_force(pos1, pos2, eta_safety_distance, k_stiffness):
    vec, g = quadratic_penalty_g(pos1, pos2, eta_safety_distance)

    if g <= 0:
        #print(f'COLLISION IMMINENT! g={g:.2f}')
        val = -k_stiffness * g
        return (vec[0] * val, vec[1] * val, vec[2] * val)

    else: #g > 0:
        #print(f"No barrier force. g={g:.2f}")
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
    #print('pos: ({}, {}, {})'.format(x, y, z))


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

    # Arm the Crazyflie
    cf.platform.send_arming_request(True)
    time.sleep(1.0)

    take_off(cf, sequences[num_seq][0])
    me = uris[num_seq]
    
    others = list(range(0,num_drones))
    others = list(map(lambda x: uris[x], others))
    others = list(filter(lambda x: x != me, others))
   
    time.sleep(1.0)

    for position in sequences[num_seq]:
        print('Setting position {}'.format(position))
        while (square_dist(curr_pos[me], position) > 0.025):  # while we are not close enough to the target, 0,025 is five cm since it is the squared distance.
            current_vec = sub_vecs(position, curr_pos[me])
            length = vec_length(current_vec)
            current_vec = scale_vec(current_vec, (1/length)*max_speed) if length > 0 else (0, 0, 0)
            
            force_list = []
            for other in others:
                force_list.append(barrier_force(curr_pos[me], curr_pos[other], eta_safety_distance, k_stiffness=k_stiffness))
            # Calculate the total barrier force by summing the contributions from all other drones in the swarm
            force = (0, 0, 0)
            for f in force_list:
                force = add_vecs(force, f)
            
            if force != (0, 0, 0):
                vec_to_send = add_vecs(force, current_vec)
                #Scale the total vector to the max_reactiion if it is too long
                vec_to_send_length = vec_length(vec_to_send)
                if vec_to_send_length > max_speed:
                    current_vec = scale_vec(vec_to_send, max_speed / vec_to_send_length) 
                else:
                    current_vec = vec_to_send
                
            cf.commander.send_velocity_world_setpoint(current_vec[0], current_vec[1], current_vec[2], 0)

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






























