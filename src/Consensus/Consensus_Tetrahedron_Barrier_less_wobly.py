import time
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

agent_1 = [
    uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701'),
    uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702'),
    uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E703'),
    #uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E704'),
    uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E705'),
    ]

agent_2 = [
    uri_helper.uri_from_env(default='radio://1/100/2M/E7E7E7E706'),
    #uri_helper.uri_from_env(default='radio://1/100/2M/E7E7E7E707'),
    uri_helper.uri_from_env(default='radio://1/100/2M/E7E7E7E708'),
    uri_helper.uri_from_env(default='radio://1/100/2M/E7E7E7E709'),
    uri_helper.uri_from_env(default='radio://1/100/2M/E7E7E7E710'),
    ]

uris = agent_1 + agent_2

drone_mapping = {uri: [i] for i, uri in enumerate(uris)}

neighboring_drones_agent_1 = {uri: [] for uri in agent_1}
neighboring_drones_agent_2 = {uri: [] for uri in agent_2}

# Consensus variables:
interdrone_dist_goal = 0.8
slack = 0.20
max_neighbors = 3
intra_agent_eta_safety_distance = 0.2

# Barrier function variables:
num_drones = len(uris)
contact_candidate_set = {1: [False for _ in range(len(agent_1))],
                         2: [False for _ in range(len(agent_2))]
                        }
eta_safety_distance = 1.65  # meters

# General variables:
max_speed = 0.3
runtimes = []
land = False
landing_req_sent = [False for _ in range(len(uris))]



sequences = { # sequence patroll
    1:[
    (-1.5,-1.5, 0.7, 0),
    (1.5, 1.5, 0.7, 0),
    (1.5, 1.5, 0.0, 0),
    ],
    2: [
    (1.5, 1.5, 0.7, 0),
    (-1.5,-1.5, 0.7, 0),
    (-1.5,-1.5, 0.0, 0),
    ]
}

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
        #print(f'COLLISION IMMINENT! distance={distance:.2f}')
        p = p_eps(distance, eta_safety_distance, n=3)
        grad_p = (p * dx / distance, p * dy / distance, p * dz / distance)
        return (grad_p, distance)    
    
    else:
        #print(f"No barrier force. distance={distance:.2f}")
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

def filter_func(agent, x):
    if agent == 1:
        return x in agent_1
    else:        
        return x in agent_2
        

def run_sequence(scf, num_seq):
    cf = scf.cf

    # Arm the Crazyflie
    cf.platform.send_arming_request(True)
    time.sleep(1.0)

    take_off(cf, (0,0,1.0))
    start = time.perf_counter()
    
    me = uris[num_seq]

    if me in agent_1:
        neighboring_drones = neighboring_drones_agent_1
        agent = 1
    else:        
        neighboring_drones = neighboring_drones_agent_2
        agent = 2
    
    same_agent = list(range(0,num_drones))
    same_agent = list(map(lambda x: uris[x], same_agent))

    same_agent = list(filter(lambda x: x != me and filter_func(agent, x), same_agent))
   
    time.sleep(1.0)
    
    
    for position in sequences[agent]:
        print('Setting position {}'.format(position))
        while (square_dist(curr_pos[me], position) > 0.25):  # while we are not close enough to the target, 0,025 is five cm since it is the squared distance.
            dist_list = [(other, last_distances[(me, other)]) for other in same_agent]
            dist_list.sort(key=lambda x: x[1])
            neighbors = [d[0] for d in dist_list[:max_neighbors]]
            neighboring_drones[me] = neighbors

            current_vec = (0,0,0)
            
            # Consensus algorithm:
            for i, neighbor in enumerate(neighboring_drones[me]):
                current_vec_temp = sub_vecs(curr_pos[neighbor], curr_pos[me])
                current_vec_length = vec_length(current_vec_temp)
                if abs(current_vec_length - interdrone_dist_goal) > slack: 
                    if current_vec_length < interdrone_dist_goal:
                        current_vec_temp = scale_vec(current_vec_temp, -(interdrone_dist_goal-current_vec_length) / current_vec_length ) 

                    current_vec = add_vecs(current_vec, current_vec_temp)        
            
            if current_vec == (0, 0, 0):
                direction = sub_vecs(position, curr_pos[me])
                current_direction_vec_length = vec_length(direction)
                
                if current_direction_vec_length > max_speed:
                    current_vec = scale_vec(current_vec, max_speed / current_vec_length)
                
                current_vec = add_vecs(current_vec, direction)
            

            
            
            # Intra agent barrier function:
            force_list = []
            closest_drone_dist = float('inf')
            for other in same_agent:
                force, dist = barrier_force_cubic_spline(curr_pos[me], curr_pos[other], intra_agent_eta_safety_distance)
                force_list.append(force)
                if dist < closest_drone_dist: 
                    closest_drone_dist = dist
            
            # Calculate the total barrier force by summing the contributions from all other drones in the swarm
            force = (0, 0, 0)
            for f in force_list:
                force = add_vecs(force, f)
            
            if force != (0, 0, 0):
                print(f'Calculated barrier force: {force}')
                current_vec = add_vecs(force, current_vec)


            # Inter agent barrier function:
            agent_center = (0, 0, 0)
            for neighbour in neighboring_drones[me]:
                agent_center = add_vecs(agent_center, curr_pos[neighbour])
            agent_center = scale_vec(agent_center, 1/len(neighboring_drones[me]))
            surface_norm = sub_vecs(curr_pos[me], agent_center)

            skalar_product = surface_norm[0] * current_vec[0] + surface_norm[1] * current_vec[1] + surface_norm[2] * current_vec[2]
            
            agent_index = agent_1.index(me) if agent == 1 else agent_2.index(me)
            if skalar_product > 0:
                contact_candidate_set[agent][agent_index] = True
            else:
                contact_candidate_set[agent][agent_index] = False
            
            if contact_candidate_set[agent][agent_index]:
                force_list = []
                closest_drone_dist = float('inf')
                if agent == 1:
                    other_agent = agent_2
                else:
                    other_agent = agent_1
                
                other_agent_candidates = [] # the other agent is 3 - current agent, since agents are 1 and 2
                # filter the other agent candidates, from agent_1 we only want those that are in agent_2 and vice versa, using the other agent candidates
                for i, b in enumerate(contact_candidate_set[3-agent]):
                    if b:
                        other_agent_candidates.append(other_agent[i])
                        print(f'me: {me} is a contact candidate, other agent candidate: {other_agent[i]}')
                
                for other in other_agent_candidates:
                    force, dist = barrier_force_cubic_spline(curr_pos[me], curr_pos[other], eta_safety_distance)
                    force_list.append(force)
                    if dist < closest_drone_dist: 
                        closest_drone_dist = dist
                
                # Calculate the total barrier force by summing the contributions from all other drones in the swarm
                force = (0, 0, 0)
                for f in force_list:
                    force = add_vecs(force, f)
                
                if force != (0, 0, 0):
                    # print current vec before and after applying the barrier force, to see the effect of the barrier function
                    print(f'Current vec before applying barrier force: {current_vec}')
                    print(f'Calculated barrier force: {force}')
                    current_vec = add_vecs(force, current_vec)
                    print(f'Current vec after applying barrier force: {current_vec}')




                
            if land and not landing_req_sent[num_seq]:
                break
                
            #current_vec = scale_vec(current_vec, current_vec_max_speed / current_vec_length if current_vec_length > current_vec_max_speed else 1)
            current_vec_length = vec_length(current_vec)
            if current_vec_length > max_speed:
                current_vec = scale_vec(current_vec, max_speed / current_vec_length) 

            if vec_length(current_vec) < 0.05:
                current_vec = (0, 0, 0)
            cf.commander.send_velocity_world_setpoint(current_vec[0], current_vec[1], current_vec[2], 0)
        
            
            
        
    
    landing(cf, curr_pos[me])
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
        swarm.parallel_safe(run_sequence, args_dict=drone_mapping)

    """ # calculate average runtimes and write to file
    avg_runtime = sum(runtimes) / len(runtimes)

    with open("timings_barrier_Patroll_mission.txt", "a") as f:
        f.write(f"Barrier Improved, dynamic speed + orientation aware algorithm runs in: {avg_runtime}\n")
 """



