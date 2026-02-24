import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.crazyflie.swarm import CachedCfFactory
from functools import partial


from cflib.crazyflie.swarm import Swarm
# URI to the Crazyflie to connect to
uri1 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E701')
uri2 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E708')

position_params = {
    uri1: [0],
    uri2: [1]
    }

uris = [uri1, uri2]

# Variables:
current_vec_lenght = 1.0
eta_safety_distance = 1.5  # meters
k_stiffness = 1.0
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



def cubic_spline_B(v): 
    if abs(v) < 1:
        return (2/3) - v**2 + 0.5 * abs(v)**3
    elif 1 <= abs(v) < 2:
        return (1/6) * (2 - abs(v))**3
    else:
        return 0

def h_eps(z, eps):
    return 1.5 * cubic_spline_B(2 * z / eps)

def p_eps(z, eps, n):
    if n not in (2, 3):
        raise ValueError("n must be 2 or 3")
    return h_eps(z, eps) / (z ** (n - 1))




# find the gradien of the penalty function p_eps with respect to the position of the drone, which is the barrier force
def barrier_force_cubic_spline(pos1, pos2, eta_safety_distance, k_stiffness):
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    dz = pos1[2] - pos2[2]

    distance = (dx**2 + dy**2 + dz**2)**0.5
    if distance == 0:
        return ((0, 0, 0), -eta_safety_distance)  # Avoid division by zero

    p = p_eps(distance, eta_safety_distance, n=3)
    grad_p = (p * dx / distance, p * dy / distance, p * dz / distance)

    if distance < eta_safety_distance:
        print(f'COLLISION IMMINENT! distance={distance:.2f}')
        vec = (grad_p[0] * k_stiffness, grad_p[1] * k_stiffness, grad_p[2] * k_stiffness)
        length = (vec[0]**2 + vec[1]**2 + vec[2]**2)**0.5
        if square_dist((0, 0, 0), vec) > current_vec_lenght**2:
            vec = (vec[0] / length * current_vec_lenght, vec[1] / length * current_vec_lenght, vec[2] / length * current_vec_lenght)
        return vec
    else:
        print(f"No barrier force. distance={distance:.2f}")
        return (0, 0, 0)













""" def quadratic_penalty_g(pos1, pos2, eta_safety_distance):
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    dz = pos1[2] - pos2[2]

    distance = (dx**2 + dy**2 + dz**2)**0.5
    if distance == 0:
        return ((0, 0, 0), -eta_safety_distance)  # Avoid division by zero

    return ((dx/distance, dy/distance, dz/distance), distance - eta_safety_distance)
 """

def square_dist(pos1, pos2):
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    dz = pos1[2] - pos2[2]

    return (dx**2 + dy**2 + dz**2)# avoiding sqrt for efficiency, since we only care about relative distances


""" def barrier_force(pos1, pos2, eta_safety_distance, k_stiffness):
    vec, g = quadratic_penalty_g(pos1, pos2, eta_safety_distance)

    if g <= 0:
        print(f'COLLISION IMMINENT! g={g:.2f}')
        val = -k_stiffness * g
        return (vec[0] * val, vec[1] * val, vec[2] * val)

    else: #g > 0:
        print(f"No barrier force. g={g:.2f}")
        return (0, 0, 0)
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
        while (square_dist(curr_pos[me], position) > 0.025):  # while we are not close enough to the target, 0,025 is five cm since it is the squared distance.
            force = barrier_force_cubic_spline(curr_pos[me], curr_pos[other], eta_safety_distance, k_stiffness=k_stiffness)
            print(f'Calculated barrier force: {force}')
            if force != (0, 0, 0):
                
                current_vec = (position[0] - curr_pos[me][0], position[1] - curr_pos[me][1], position[2] - curr_pos[me][2])
                length = ((current_vec[0]**2 + current_vec[1]**2 + current_vec[2]**2)**0.5)

                if length > 0:
                    current_vec = (current_vec[0] / length * current_vec_lenght, current_vec[1] / length * current_vec_lenght, current_vec[2] / length * current_vec_lenght)
                print(f'Scaled current vector to target: {current_vec}')

                # Scale the force
                force = (force[0] * force_scaler, force[1] * force_scaler, force[2] * force_scaler)
                
                cf.commander.send_velocity_world_setpoint(force[0] + current_vec[0], force[1] + current_vec[1], force[2] + current_vec[2], 0)
                
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
    
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(start_position_printing)
        swarm.parallel_safe(run_sequence, args_dict=position_params)






























