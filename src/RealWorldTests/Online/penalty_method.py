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

# URI of the Crazyflies to connect to
uri1 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E705')
uri2 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E704')
uri3 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E710')
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
runtimes = []

# Change the sequence according to your setup
#             x    y    z  YAW

sequences = { # sequence patroll
    0:[
    (-1.5,-1.5, 0.7, 0),
    (1.5, -1.5, 0.7, 0),
    (1.5, 1.5, 0.7, 0),
    (-1.5, 1.5, 0.7, 0),
    (-1.5, -1.5, 0.7,0),
    (-1.5,-1.5, 0.0, 0)
    ],
    1: [
    (1.5, -1.5, 0.7, 0),
    (1.5, 1.5, 0.7, 0),
    (-1.5, 1.5, 0.7, 0),
    (-1.5, -1.5, 0.7,0),
    (1.5, -1.5, 0.7, 0),
    (1.5, -1.5, 0.0, 0)
    ],
    2: [
    (1.5, 1.5, 0.7, 0),
    (-1.5, 1.5, 0.7, 0),
    (-1.5, -1.5, 0.7,0),
    (1.5, -1.5, 0.7, 0),
    (1.5, 1.5, 0.7, 0),
    (1.5,  1.5, 0.0, 0),
    ],
    3: [
    (-1.5, 1.5, 0.7, 0),
    (-1.5, -1.5, 0.7,0),
    (1.5, -1.5, 0.7, 0),
    (1.5, 1.5, 0.7, 0),
    (-1.5, 1.5, 0.7, 0),
    (-1.5,  1.5, 0.0, 0),
    ],
}

sequences_ = { # cross mission
    0: [
    (-0.2, -0.2, 0.7, 0),
    (1.0, 1.0, 0.4, 0),
    (1.0, 1.0, 0.0, 0)
    ],
    1: [
    (-0.2,  0.2, 0.7, 0),
    (1.0, -1.0, 0.4, 0),
    (1.0, -1.0, 0.0, 0),
    ],
    2: [
    (0.2, 0.2, 0.7, 0),
    (-1.0, -1.0, 0.4, 0),
    (-1.0, -1.0, 0.0, 0),
    ],
    3: [
    ( 0.2, -0.2, 0.7, 0),
    (-1.0, 1.0, 0.4, 0),
    (-1.0, 1.0, 0.0, 0),
    ] 
}

sequence_1 = { # Mission 1
    0: [
    (-1, -1, 0.7, 0),
    (1.2, 1.0, 0.4, 0),
    (1.2, 1.0, 0.0, 0)
    ],
    1: [
    (-1.0,  1.0, 0.7, 0),
    (1.0, -1.0, 0.4, 0),
    (1.0, -1.0, 0.0, 0),
    ],
    2: [
    (0.0, -1.0, 0.7, 0),
    (-0.2, 1.0, 0.4, 0),
    (-0.2, 1.0, 0.0, 0),
    ],
    3: [
    ( 0.6, -1.0, 0.7, 0),
    (0.6, 1.0, 0.4, 0),
    (0.6, 1.0, 0.0, 0),
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
    if distance == 0: # Avoid division by zero
        return ((0, 0, 0), -eta_safety_distance)  

    return ((dx/distance, dy/distance, dz/distance), distance - eta_safety_distance)

def square_dist(pos1, pos2):
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    dz = pos1[2] - pos2[2]

    # avoiding sqrt for efficiency, since we only care about relative distances
    return (dx**2 + dy**2 + dz**2)

def barrier_force(pos1, pos2, eta_safety_distance, k_stiffness):
    vec, g = quadratic_penalty_g(pos1, pos2, eta_safety_distance)

    if g <= 0:
        val = -k_stiffness * g
        return (vec[0] * val, vec[1] * val, vec[2] * val)

    else: #g > 0:
        return (0, 0, 0)

def limit_velocity(cf, xy=0.25, z=0.20):
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

def start_position_printing(scf):
    uri = scf.cf.link_uri
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')
    scf.cf.log.add_config(log_conf)

    # Binding uri into the callback
    log_conf.data_received_cb.add_callback(partial(position_callback, uri))
    log_conf.start()

def run_sequence(scf, num_seq):
    cf = scf.cf

    # Arming the Crazyflie
    cf.platform.send_arming_request(True)
    time.sleep(1.0)

    take_off(cf, sequences[num_seq][0])
    start = time.perf_counter()
    me = uris[num_seq]
    
    others = list(range(0,num_drones))
    others = list(map(lambda x: uris[x], others))
    others = list(filter(lambda x: x != me, others))
   
    time.sleep(1.0)

    for position in sequences[num_seq]:
        while (square_dist(curr_pos[me], position) > 0.025): 
            current_vec = sub_vecs(position, curr_pos[me])
            length = vec_length(current_vec)
            current_vec = scale_vec(current_vec, (1/length)*max_speed) if length > 0 else (0, 0, 0)
            
            force_list = []
            for other in others:
                force_list.append(barrier_force(curr_pos[me], curr_pos[other], eta_safety_distance, k_stiffness=k_stiffness))

            # Calculating the total barrier force by summing the contributions from all other drones
            force = (0, 0, 0)
            for f in force_list:
                force = add_vecs(force, f)
            
            if force != (0, 0, 0):
                vec_to_send = add_vecs(force, current_vec)

                # Scaling the total vector to the max_reactiion if it is too long
                vec_to_send_length = vec_length(vec_to_send)
                if vec_to_send_length > max_speed:
                    current_vec = scale_vec(vec_to_send, max_speed / vec_to_send_length) 
                else:
                    current_vec = vec_to_send
                
            cf.commander.send_velocity_world_setpoint(current_vec[0], current_vec[1], current_vec[2], 0)
            time.sleep(0.1)

    cf.commander.send_stop_setpoint()

    # Giving control to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    end = time.perf_counter()
    runtime = end - start
    runtimes.append(runtime)

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(start_position_printing)
        swarm.parallel_safe(run_sequence, args_dict=position_params)

    # calculate average runtimes and write to file
    avg_runtime = sum(runtimes) / len(runtimes)

    with open("timings_barrier_Patroll_mission.txt", "a") as f:
        f.write(f"Barrier Simple algorithm runs in: {avg_runtime}\n")
