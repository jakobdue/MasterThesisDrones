import matplotlib.pyplot as plt
from scipy.optimize import minimize
import numpy as np
import time
from turtle import position

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.crazyflie.swarm import CachedCfFactory
from functools import partial


from cflib.crazyflie.swarm import Swarm



type Point = tuple[float, float, float]
type Path  = list[Point] # points is a list of tuples with (x,y,z) coordinates.

# constants: 
eta_safety_distance = 1.0
T = 10
dt = 1 / (T - 1) # normalize to the number of timesteps.


cross_sequences = {
    0: [(-0.2, -0.2, 0.7), (1.0, 1.0, 0.7)],
    1: [(-0.2,  0.2, 0.7), (1.0, -1.0, 0.7)],
    2: [(0.2, 0.2, 0.7), (-1.0, -1.0, 0.7)],
    3: [(0.2, -0.2, 0.7), (-1.0, 1.0, 0.7)],
}



original_sequences = {
    0: [(-1.0, -1.0, 0.7), (1.2, 1.0, 0.7)],
    1: [(-1.0,  1.0, 0.7), (1.0, -1.0, 0.7)],
    2: [(0.0, -1.0, 0.7), (-0.2, 1.0, 0.7)],
    3: [(0.6, -1.0, 0.7), (0.6, 1.0, 0.7)],
}



def generate_circle_crossing(num_drones=20, radius=1.5, z=0.7):
    sequences = {}

    for i in range(num_drones):
        theta = 2 * np.pi * i / num_drones

        start = (
            radius * np.cos(theta),
            radius * np.sin(theta),
            z
        )

        end = (
            -start[0],
            -start[1],
            z
        )

        sequences[i] = [start, end]

    return sequences


#original_sequences = generate_circle_crossing(50)


# interpolate points in the paths to have T points in total. there is always two points in the path, so we can interpolate T-2 points between them.
def interpolate_path(path: Path, noise_std: float = 0.01) -> Path:
    if len(path) < 2:
        raise ValueError("Path must have at least 2 points to interpolate.")
    
    start, end = path[0], path[1]
    interpolated_path = []
    
    for t in range(T):
        alpha = t / (T - 1)  # alpha goes from 0 to 1
        point = (
            start[0] + alpha * (end[0] - start[0]),
            start[1] + alpha * (end[1] - start[1]),
            start[2] + alpha * (end[2] - start[2])
        )
        # add noise only to interior points
        if 0 < t < T-1:
            noise = np.random.normal(0.0, noise_std, size=3)
            point = (
                point[0] + noise[0],
                point[1] + noise[1],
                max(point[2] + noise[2], 0.0)  # optional: keep above ground
            )
        interpolated_path.append(point)
    
    return interpolated_path




drones = [
    interpolate_path(cross_sequences[i])
    for i in range(len(cross_sequences))
]



def make_bounds(paths):
    bounds = []
    for path in paths:
        for k, point in enumerate(path):
            for d in range(3):
                if k == 0 or k == T-1:
                    bounds.append((point[d], point[d]))  # fixed
                elif d == 2:
                    bounds.append((0.0, None))  # z >= 0
                else:
                    bounds.append((None, None))
    return bounds

def flatten_paths(paths: list[Path]) -> np.ndarray:
    return np.array([coord for path in paths for point in path for coord in point])

def unflatten_paths(x: np.ndarray, num_paths: int) -> list[Path]:
    paths = []
    idx = 0
    for _ in range(num_paths):
        path = []
        for _ in range(T):
            path.append((x[idx], x[idx+1], x[idx+2]))  # clamp z here
            idx += 3
        paths.append(path)
    return paths

x0 = flatten_paths(drones)
bounds = make_bounds(drones)


def add_vecs(vec1, vec2):
    return (vec1[0] + vec2[0], vec1[1] + vec2[1], vec1[2] + vec2[2])

def scale_vec(scaler, vec):
    return (vec[0] * scaler, vec[1] * scaler, vec[2] * scaler)

def sub_vecs(vec1, vec2):
    return (vec1[0] - vec2[0], vec1[1] - vec2[1], (vec1[2] - vec2[2]))

def vec_length(vec):
    return (vec[0]**2 + vec[1]**2 + vec[2]**2)**0.5

def squared_vec_length(vec):
    return (vec[0]**2 + vec[1]**2 + vec[2]**2)

def clip(vec, max_norm=40.0):
    norm = vec_length(vec)
    if norm > max_norm:
        return scale_vec(max_norm / norm, vec)
    return vec

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



def cubic_spline_B_derivative(v):
    if v < 1:
        return -2*v + 1.5 * v ** 2
    elif 1 <= v < 2:
        return -0.5 * (2 - v)**2
    else:
        return 0.0

def barrier_force_cubic_spline(pos1, pos2, eps):
    dx, dy, dz = sub_vecs(pos1, pos2)
    z = vec_length((dx, dy, dz))

    if z < 1e-6:
        return (0, 0, 0)

    # Only apply force inside interaction radius
    if z >= eps:
        return (0, 0, 0)

    # Compute h(z)
    v = 2 * z / eps
    B = cubic_spline_B(v)
    h = 1.5 * B

    # Compute h'(z)
    B_prime = cubic_spline_B_derivative(v)
    h_prime = (3 / eps) * B_prime

    n = 3  # since we're using 3D

    # p_prime (Negative gradient of the penalty function with respect to z)
    p_prime = (z * h_prime - (n-1) * h) / z**(n+1) 

    # gradient = dp/dz * (x - y)
    grad = (p_prime * dx, p_prime * dy, p_prime * dz)

    return grad



def total_path_energy(paths:list[Path]): 
    total_path_length_energy = 0.0
    # calculate the total lenght of a path by summing the distances between consecutive points only taking the square root in the end to save computation time.
    for path in paths: 
        for k in range(len(path)-1): 
            diff = sub_vecs(path[k+1], path[k])
            total_path_length_energy += squared_vec_length(diff) / dt
    
    total_barrier_energy = 0.0
    for k in range(T):
        for i in range(len(paths)): 
            for j in range(i+1, len(paths)): 
                (dx, dy, dz) = sub_vecs(paths[i][k], paths[j][k])

                distance = vec_length((dx, dy, dz))
                total_barrier_energy += p_eps(distance, eta_safety_distance, 3)

    total_diversion_energy = 0.0

         
    return (total_path_length_energy, total_barrier_energy)


def gradient_barrier_energy(paths:list[Path]):
    grad_smooth:list[Path] = [[(0,0,0) for _ in range(T)] for _ in range(len(paths))]
    grad_barrier:list[Path] = [[(0,0,0) for _ in range(T)] for _ in range(len(paths))]
    for i in range(len(paths)):
        for k in range(T):
            if k == 0 or k == T-1:
              continue
            else:
                grad_smooth[i][k] = scale_vec(
                    2 / dt,
                    sub_vecs(
                        sub_vecs(scale_vec(2, paths[i][k]), paths[i][k-1]),
                        paths[i][k+1]
                    )
                )
            
        # compute the gradient of the barrier energy with respect to the position of each point in the path.
        for k in range(T):
            per_point_barrier_force = (0, 0, 0)
            for j in range(len(paths)):
                if j == i:
                    continue
                else: 
                    per_point_barrier_force = add_vecs(per_point_barrier_force, barrier_force_cubic_spline(paths[i][k], paths[j][k], eta_safety_distance))
            grad_barrier[i][k] = per_point_barrier_force
                
    return (grad_smooth, grad_barrier)


    
def objective(x: np.ndarray, num_paths: int):
    paths = unflatten_paths(x, num_paths)
    path_energy, barrier_energy = total_path_energy(paths)
    return barrier_energy + path_energy

def objective_grad(x: np.ndarray, num_paths: int):
    paths = unflatten_paths(x, num_paths)
    grad_smooth, grad_barrier = gradient_barrier_energy(paths)

    grad = []
    for i in range(len(paths)):
        for k in range(T):
            g = add_vecs(grad_smooth[i][k], grad_barrier[i][k])
            grad.extend(g)

    return np.array(grad)






res = minimize(
    objective,
    x0,
    args=(len(drones),),
    method='L-BFGS-B',   # good default
    jac=objective_grad,
    bounds=bounds,
    options={'maxiter': 200, 'disp': True}
)

new_paths = unflatten_paths(res.x, len(drones))

# convert new_paths back to sequence format for running with the drones: 
drones_new_paths = {}
for i in range(len(new_paths)):
    drones_new_paths[i] = new_paths[i]





def print_closest_pairs(paths: list[Path]):
    global_min_dist = float('inf')
    global_info = None

    for k in range(T):
        for i in range(len(paths)):
            for j in range(i+1, len(paths)):
                diff = sub_vecs(paths[i][k], paths[j][k])
                dist = vec_length(diff)

                if dist < global_min_dist:
                    global_min_dist = dist
                    global_info = (k, i, j, paths[i][k], paths[j][k])

    k, i, j, p1, p2 = global_info

    print("\nClosest pair overall:")
    print(f" timestep k = {k}")
    print(f" drones ({i}, {j})")
    print(f" distance = {global_min_dist:.6f}")
    print(f" point {i}: {p1}")
    print(f" point {j}: {p2}")

print_closest_pairs(new_paths)

# print energy components after optimization
final_path_energy, final_barrier_energy = total_path_energy(new_paths)
print(f"Final path energy: {final_path_energy:.4f}")
print(f"Final barrier energy: {final_barrier_energy:.4f}")



# plot the new_paths after optimization with scipy in matplotlib
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for i, path in enumerate(new_paths):
    ax.plot(
        [point[0] for point in path],
        [point[1] for point in path],
        [point[2] for point in path],
        label=f'drone{i+1}'
    )
# axis labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
# set axis limits
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(0, 3)
ax.set_box_aspect([4, 4, 3])  # matches x,y,z ranges
ax.legend()
plt.show()




































# URI to the Crazyflie to connect to
uris = [
    #uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E707'),
    #uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E708'),
    #uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E703'),
    #uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701'),
    #uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701'),
    uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702'),
    uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E710'),
    uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E709'),
    
    #uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E703'),
    uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E706'),
    
    ]

position_params = {uri: [i] for i, uri in enumerate(uris)}

# Variables:
num_drones = 4
eta_safety_distance = 0.65  # meters
runtimes = []

# Change the sequence according to your setup
#             x    y    z  YAW


sequences_ = { # sequence patroll
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

sequences_ = { # cross_mission
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

sequences_ = { #Original
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

old_sequences_ = {
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

sequences = drones_new_paths
print("Running with sequences: ", sequences)
    
curr_pos = {uri: (0, 0, 0) for uri in uris}
curr_timestep_seq = {uri: 0 for uri in uris}
curr_timestep = 0

last_distances = {
    (u1, u2): float('inf')
    for u1 in uris
    for u2 in uris
    if u1 != u2
}

drone_inbound = {uri: False for uri in uris}

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
    # update the current timestep for the slowest drone
    global curr_timestep
    curr_timestep = min(curr_timestep_seq.values())
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
    start = time.perf_counter()
    me = uris[num_seq]
    
    others = list(range(0,num_drones))
    others = list(map(lambda x: uris[x], others))
    others = list(filter(lambda x: x != me, others))
   
    time.sleep(1.0)

    for i, position in enumerate(sequences[num_seq]):
        curr_timestep_seq[me] = i
        print('Setting position {}'.format(position))
        while (square_dist(curr_pos[me], position) > 0.15) or curr_timestep != curr_timestep_seq[me]:  # while we are not close enough to the target, 0.0225 is 15 cm since it is the squared distance.
            cf.commander.send_position_setpoint(position[0], position[1], position[2], 0)
        """ while (square_dist(curr_pos[me], position) > 0.05) or curr_timestep != curr_timestep_seq[me]:  # while we are not close enough to the target, 0,025 is five cm since it is the squared distance.
 
            current_vec = sub_vecs(position, curr_pos[me])
            current_vec_length = vec_length(current_vec) 
            

            force_list = []
            closest_drone_dist = float('inf')
            for other in others:
                force, dist = barrier_force_cubic_spline(curr_pos[me], curr_pos[other], eta_safety_distance)
                force_list.append(force)
                if dist < closest_drone_dist: 
                    closest_drone_dist = dist
            
            max_speed = speed_scaler(closest_drone_dist, drone_inbound[me])
            print(f'Closest drone distance: {closest_drone_dist:.2f}, max speed set to: {max_speed:.2f}')
            # Scale the current vector to unit vector
            current_vec_max_speed = max_speed
            current_vec = scale_vec(current_vec, current_vec_max_speed / current_vec_length if current_vec_length > current_vec_max_speed else 1)
            
            # Calculate the total barrier force by summing the contributions from all other drones in the swarm
            force = (0, 0, 0)
            for f in force_list:
                force = add_vecs(force, f)
            
            print(f'Calculated individual barrier forces: {force_list}')

            if force != (0, 0, 0):
                vec_to_send = add_vecs(force, current_vec)
                #Scale the total vector to the max_reactiion if it is too long
                vec_to_send_length = vec_length(vec_to_send)
                if vec_to_send_length > max_speed:
                    current_vec = scale_vec(vec_to_send, max_speed / vec_to_send_length) 
                else:
                    current_vec = vec_to_send
                
            cf.commander.send_velocity_world_setpoint(current_vec[0], current_vec[1], current_vec[2], 0) """
     
            #time.sleep(0.05)
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
        swarm.parallel_safe(run_sequence, args_dict=position_params)

    # calculate average runtimes and write to file
    avg_runtime = sum(runtimes) / len(runtimes)

    with open("timings_offline_barrier_cross_mission.txt", "a") as f:
        f.write(f"Offline barrier: {avg_runtime}\n")




