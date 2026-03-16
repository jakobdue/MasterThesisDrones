import time
from turtle import position

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.crazyflie.swarm import CachedCfFactory
from functools import partial
import random
from pynput import keyboard


from cflib.crazyflie.swarm import Swarm
# URI to the Crazyflie to connect to

uris = [
    uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E706'),
    uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702'),
    uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E708'),
    uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E707'),
    uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701'),
    ]

position_params = {uri: [i] for i, uri in enumerate(uris)}

neighboring_drones = {uri: [] for uri in uris}

# Variables:
interdrone_dist_goal = 1.0
max_neighbors = 3
num_drones = len(uris)
eta_safety_distance = 0.65  # meters
runtimes = []
land = False
landing_req_sent = [False for _ in range(num_drones)]

# Change the sequence according to your setup
#             x    y    z  YAW

def on_press(key):
    global land
    if key == keyboard.Key.space:
        land = True
        print("Landing triggered")

listener = keyboard.Listener(on_press=on_press)
listener.start()

last_distances = {
    (u1, u2): float('inf')
    for u1 in uris
    for u2 in uris
    if u1 != u2
}

drone_inbound = {uri: False for uri in uris}

curr_pos = {uri: (0, 0, 0) for uri in uris}
                
def speed_scaler(distance,inbound):
    if inbound:
        if distance < 1:
            return 0.2
        else: 
            return distance * 0.7 - 0.5
    else: 
        if distance < 0.5:
            return 0.5
        else:  
            return distance * 0.7 + 0.2

def add_vecs(vec1, vec2):
    return (vec1[0] + vec2[0], vec1[1] + vec2[1], vec1[2] + vec2[2])

def scale_vec(vec, scaler):
    return (vec[0] * scaler, vec[1] * scaler, vec[2] * scaler)

def sub_vecs(vec1, vec2):
    return (vec1[0] - vec2[0], vec1[1] - vec2[1], (vec1[2] - vec2[2]))

def vec_length(vec):
    return (vec[0]**2 + vec[1]**2 + vec[2]**2)**0.5

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
def barrier_force_cubic_spline(pos1, pos2, eta_safety_distance):
    (dx, dy, dz) = sub_vecs(pos1, pos2)

    distance = vec_length((dx, dy, dz))
    if distance == 0:
        return ((0, 0, 0), 0)  # Avoid division by zero

    if distance < eta_safety_distance+0.3:
        print(f'COLLISION IMMINENT! distance={distance:.2f}')
        p = p_eps(distance, eta_safety_distance, n=3)
        grad_p = (p * dx / distance, p * dy / distance, p * dz / distance)
        return (grad_p, distance)    
    
    else:
        print(f"No barrier force. distance={distance:.2f}")
        return ((0, 0, 0), distance)

def square_dist(pos1, pos2):
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    dz = pos1[2] - pos2[2]

    return (dx**2 + dy**2 + dz**2)# avoiding sqrt for efficiency, since we only care about relative distances

def random_goal():
    x = random.uniform(-1.5, 1.5)
    y = random.uniform(-1.5, 1.5)
    z = random.uniform(0.5, 2.0)
    return (x, y, z, 0)



def take_off(cf, position):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position[2] / take_off_time

    print(f'take off at {position[2]}')

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)


def landing(cf, position):
    landing_time = 3.0
    sleep_time = 0.1
    steps = int(landing_time / sleep_time)
    vz = -position[2] / landing_time

    print(f'landing at {position[2]}')

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

def position_callback(uri, timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    curr_pos[uri] = (x, y, z)
    #update last_distances
    inbound = False
    for other_uri in curr_pos:
        if other_uri != uri:
            dist = square_dist(curr_pos[uri], curr_pos[other_uri])
            if dist < last_distances[(uri, other_uri)]:
                inbound = True
            last_distances[(uri, other_uri)] = dist
            last_distances[(other_uri, uri)] = dist
    if inbound: 
        drone_inbound[uri] = True
    else:
        drone_inbound[uri] = False
    


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

    goal = random_goal()

    # Arm the Crazyflie
    cf.platform.send_arming_request(True)
    time.sleep(1.0)

    take_off(cf, goal)
    start = time.perf_counter()
    me = uris[num_seq]
    
    others = list(range(0,num_drones))
    others = list(map(lambda x: uris[x], others))
    others = list(filter(lambda x: x != me, others))
   
    time.sleep(1.0)
    

    while True:  # while we are not close enough to the target, 0,025 is five cm since it is the squared distance.
        dist_list = [(other, last_distances[(me, other)]) for other in others]
        dist_list.sort(key=lambda x: x[1])
        neighbors = [d[0] for d in dist_list[:max_neighbors]]
        neighboring_drones[me] = neighbors

        current_vec = (0,0,0)
        for i, neighbor in enumerate(neighboring_drones[me]):
            if i == 2:
                current_vec_temp = scale_vec(sub_vecs(curr_pos[me], curr_pos[neighbor]), 0.03)
            else:
                current_vec_temp = sub_vecs(curr_pos[neighbor], curr_pos[me])
                current_vec_length = vec_length(current_vec_temp)
                if abs(current_vec_length - interdrone_dist_goal) > 0.15: 
                    if current_vec_length < interdrone_dist_goal:
                        current_vec_temp = scale_vec(current_vec_temp, -(interdrone_dist_goal-current_vec_length) / current_vec_length ) 

            current_vec = add_vecs(current_vec, current_vec_temp)

        force_list = []
        closest_drone_dist = float('inf')
        for other in others:
            force, dist = barrier_force_cubic_spline(curr_pos[me], curr_pos[other], eta_safety_distance)
            force_list.append(force)
            if dist < closest_drone_dist: 
                closest_drone_dist = dist
        
        max_speed = 0.2 #speed_scaler(closest_drone_dist, drone_inbound[me])
        #print(f'Closest drone distance: {closest_drone_dist:.2f}, max speed set to: {max_speed:.2f}')
        # Scale the current vector to unit vector
        current_vec_max_speed = max_speed
        current_vec_length = vec_length(current_vec)
        current_vec = scale_vec(current_vec, current_vec_max_speed / current_vec_length if current_vec_length > current_vec_max_speed else 1)
        # Calculate the total barrier force by summing the contributions from all other drones in the swarm
        force = (0, 0, 0)
        for f in force_list:
            force = add_vecs(force, f)
        
        #print(f'Calculated individual barrier forces: {force_list}')

        if force != (0, 0, 0):
            vec_to_send = add_vecs(force, current_vec)
            #Scale the total vector to the max_reactiion if it is too long
            vec_to_send_length = vec_length(vec_to_send)
            if vec_to_send_length > max_speed:
                current_vec = scale_vec(vec_to_send, max_speed / vec_to_send_length) 
            else:
                current_vec = vec_to_send
            
        if land and not landing_req_sent[num_seq]:
            landing(cf, curr_pos[me])
            break
            
        cf.commander.send_velocity_world_setpoint(current_vec[0], current_vec[1], current_vec[2], 0)
    
        
            
        
    
    
    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
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
        f.write(f"Barrier Improved, dynamic speed + orientation aware algorithm runs in: {avg_runtime}\n")




