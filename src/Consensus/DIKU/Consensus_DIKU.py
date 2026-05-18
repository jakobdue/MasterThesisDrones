import time
from turtle import position

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.crazyflie.swarm import CachedCfFactory
from functools import partial
import random
from pynput import keyboard
import math


from cflib.crazyflie.swarm import Swarm
# URI to the Crazyflie to connect to

uris = [
    #uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701'),
    #uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702'),
    uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E703'),
    #uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E704'),
    uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E705'),
    #uri_helper.uri_from_env(default='radio://1/100/2M/E7E7E7E706'),
    uri_helper.uri_from_env(default='radio://1/100/2M/E7E7E7E707'),
    uri_helper.uri_from_env(default='radio://1/100/2M/E7E7E7E708'),
    #uri_helper.uri_from_env(default='radio://1/100/2M/E7E7E7E709'),
    #uri_helper.uri_from_env(default='radio://1/100/2M/E7E7E7E710'),
    ]


""" goal_positions_D =[
    (0,    0,    1.5, 0),
    (0,    0.5,  1.5, 0),
    (0,    1,    1.5, 0),
    (0,    1.5,  1.5, 0),
    (0.5,  0.15, 1.5, 0),
    (0.65, 0.5,  1.5, 0),
    (0.65, 1,    1.5, 0),
    (0.5,  1.35, 1.5, 0),
] """

goal_positions_D = [
    (0.65, 0.5,  1.5, 0),
    (0.5,  0.15, 1.5, 0),
    (0,    1.5,  1.5, 0),
    (0,    1,    1.5, 0),
    (0,    0.5,  1.5, 0),
    (0,    0,    1.5, 0),
    (0.5,  1.35, 1.5, 0),
    (0.65, 1,    1.5, 0)
]

position_params = {uri: [i] for i, uri in enumerate(uris)}

neighboring_drones = {uri: [] for uri in uris}

# Variables:
interdrone_dist_goal = 0.5
hight_from_ground = 1.5
slack = 0.10
max_neighbors = 2
num_drones = len(uris)
eta_safety_distance = 0.2  # meters
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

# find the gradien of the penalty function p_eps with respect to the position of the drone, which is the barrier force
def cubic_spline_B_derivative(v):
    if v < 1:
        return -2*v + 1.5 * v ** 2
    elif 1 <= v < 2:
        return -0.5 * (2 - v)**2
    else:
        return 0.0

def barrier_force_cubic_spline(pos1, pos2, eps):
    dx, dy, dz = sub_vecs(pos2, pos1)
    z = vec_length((dx, dy, dz))

    if z < 1e-6:
        return (0, 0, 0), z

    # Only apply force inside interaction radius
    if z >= eps:
        return (0, 0, 0), z

    # Compute h(z)
    v = 2 * z / eps
    B = cubic_spline_B(v)
    h = 1.5 * B

    # Compute h'(z)
    B_prime = cubic_spline_B_derivative(v)
    h_prime = (3 / eps) * B_prime

    n = 3  # since we're using 3D

    # p_prime (Negative gradient of the penalty function with respect to z)
    p_prime = (z * h_prime - (n-1) * h) / z**n 

    # gradient = dp/dz * (x - y)/z
    grad = (p_prime * dx, p_prime * dy, p_prime * dz)

    return grad, z

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

    # Arm the Crazyflie
    cf.platform.send_arming_request(True)
    time.sleep(1.0)

    take_off(cf, (0,0,1.0))
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
        neighbour_direction = (0,0,0)
        for i, neighbor in enumerate(neighboring_drones[me]):
            neighbour_direction_temp = sub_vecs(curr_pos[neighbor], curr_pos[me])
            neighbour_direction_length = vec_length(neighbour_direction_temp)
            if abs(neighbour_direction_length - interdrone_dist_goal) > slack: 
                if neighbour_direction_length < interdrone_dist_goal:
                    neighbour_direction_temp = scale_vec(neighbour_direction_temp, -(interdrone_dist_goal-neighbour_direction_length) / neighbour_direction_length ) 

                neighbour_direction = add_vecs(neighbour_direction, neighbour_direction_temp)

        polar_angle = math.atan2(neighbour_direction[1], neighbour_direction[0])
        #print(f'Polar angle to neighbors: {math.degrees(polar_angle):.2f} degrees')

        # divide the circle in 8 and decide which one we are in
        sector = int((polar_angle + math.pi) / (math.pi / 4)) % 8
        goal_pos = goal_positions_D[sector]

        # sort you neighbors after polar angles
        neighbors_sorted = sorted(neighboring_drones[me], key=lambda x: math.atan2(curr_pos[x][1] - curr_pos[me][1], curr_pos[x][0] - curr_pos[me][0]))

        # calculate the consensus vector using the point before and after the goal pos, the goal pos is local and not gobal so we only look at relational coordinates.
        neighbour1_vec = sub_vecs(curr_pos[neighbors_sorted[0]], curr_pos[me])
        neighbour2_vec = sub_vecs(curr_pos[neighbors_sorted[1]], curr_pos[me])
        goal_vec1 = sub_vecs(goal_pos, goal_positions_D[(sector - 1) % 8])
        goal_vec2 = sub_vecs(goal_pos, goal_positions_D[(sector + 1) % 8])

        # calculate direction to move in, looking at the difference between neighbour1_vec and goal_vec1 and so on
        consensus_vec = (0,0,0)
        if vec_length(neighbour1_vec) > 0 and vec_length(goal_vec1) > 0:
            consensus_vec = add_vecs(consensus_vec, scale_vec(neighbour1


        force_list = []
        closest_drone_dist = float('inf')
        for other in others:
            force, dist = barrier_force_cubic_spline(curr_pos[me], curr_pos[other], eta_safety_distance)
            force_list.append(force)
            if dist < closest_drone_dist: 
                closest_drone_dist = dist
        
        
        #print(f'Closest drone distance: {closest_drone_dist:.2f}, max speed set to: {max_speed:.2f}')
        # Scale the current vector to unit vector
        max_speed = 0.12 #speed_scaler(closest_drone_dist, drone_inbound[me])
        current_vec_max_speed = max_speed
        current_vec_length = vec_length(current_vec)
        
        
        
        
        
        #current_vec = scale_vec(current_vec, current_vec_max_speed / current_vec_length if current_vec_length > current_vec_max_speed else 1)
        if current_vec_length > max_speed:
            current_vec = scale_vec(current_vec, max_speed / current_vec_length) 
        
        
        
        # Calculate the total barrier force by summing the contributions from all other drones in the swarm
        force = (0, 0, 0)
        for f in force_list:
            force = add_vecs(force, f)
        
        

        if force != (0, 0, 0):
            print(f'Calculated barrier force: {force}')
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
            

        if vec_length(current_vec) < 0.05:
            current_vec = (0, 0, 0)
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




