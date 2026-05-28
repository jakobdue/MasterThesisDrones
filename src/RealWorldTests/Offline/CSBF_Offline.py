import time
import math
import random
from functools import partial
from typing import List, Tuple, Set, Optional
import numpy as np
import matplotlib.pyplot as plt
import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from cflib.utils import uri_helper
import matplotlib.pyplot as plt
from scipy.optimize import minimize, check_grad
import numpy as np
type Point = tuple[float, float, float]
type Path  = list[Point] # points is a list of tuples with (x,y,z) coordinates.

# ----------------------------
# 1. Settings
# ----------------------------
TIME_LIMIT_SECONDS = 300
RESULTS_FILE = "BarrierPathPlanningAllTests.txt"
CIRCLE_RADIUS = 30/5
SAVE_FIGS = True
GRADIENT_CHECK = True
sequences = {}

PLOT = True
DEBUG = False
AUTOAXIS_IN_CIRCLEPLOT = True

random.seed(42) # For reproducibility

# Constants: 
eta_safety_distance = 0.4
T = 10
dt = 1 / (T - 1) # Normalizing to the number of timesteps.
obstacles = set() # Set of tuples with (x,y,z) coordinates of obstacles.

# Missions
mission_Cross = { # Real world coordinates
    0: [(-0.2, -0.2, 0.7), (1.0, 1.0, 0.7)], 
    1: [(-0.2,  0.2, 0.7), (1.0, -1.0, 0.7)],
    2: [(0.2, 0.2, 0.7), (-1.0, -1.0, 0.7)],
    3: [(0.2, -0.2, 0.7), (-1.0, 1.0, 0.7)],
}

mission_OG = {
    0: [(-1.0, -1.0, 0.7), (1.2, 1.0, 0.7)],
    1: [(-1.0,  1.0, 0.7), (1.0, -1.0, 0.7)],
    2: [(0.0, -1.0, 0.7), (-0.2, 1.0, 0.7)],
    3: [(0.6, -1.0, 0.7), (0.6, 1.0, 0.7)],
}
# Divide by 5 to get the coordinates in the same scale as the other missions for real world 
def scale_vec(scaler, vec):
    return (vec[0] * scaler, vec[1] * scaler, vec[2] * scaler)

# ----------------------------
# 2. Helpers
# ----------------------------

def add_vecs(vec1, vec2):
    return (vec1[0] + vec2[0], vec1[1] + vec2[1], vec1[2] + vec2[2])

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

def update_random_obstacles(num_obstacles, start, goal):
    X_MIN, X_MAX = -5, 5
    Y_MIN, Y_MAX = -5, 5
    Z_MIN, Z_MAX = 0, 9
    global obstacles
    obstacles = set()
    new_obstacles_generated = True
    # Checking that the time does not run out 
    deadline = time.perf_counter() + TIME_LIMIT_SECONDS
    while len(obstacles) < num_obstacles:
        obs = (random.randint(X_MIN, X_MAX), random.randint(Y_MIN, Y_MAX), random.randint(Z_MIN, Z_MAX))
        # Checking that the generated obstacle is not start nor goal:
        if obs in start or obs in goal:
            continue
        obstacles.add(scale_vec(1/5, obs))
        if time.perf_counter() >= deadline:
            print(f"Time limit of {TIME_LIMIT_SECONDS} seconds reached while generating obstacles.")
            new_obstacles_generated = False
            break
    return new_obstacles_generated

def compute_average_path_length(paths):
    total_length = 0
    count = 0
    for path in paths:
        if path is not None:
            length = 0
            for j in range(1, len(path)):
                length += math.sqrt((path[j][0] - path[j-1][0])**2 +
                                    (path[j][1] - path[j-1][1])**2 +
                                    (path[j][2] - path[j-1][2])**2)
            total_length += length
            count += 1
    return (total_length / count) if count > 0 else None

# Interpolating points in the paths to have T points in total.
def interpolate_path(path: Path, noise_std: float = 0.001) -> Path:
    if len(path) < 2:
        raise ValueError("Path must have at least 2 points to interpolate.")
    
    start, end = path[0], path[1]
    interpolated_path = []
    
    for t in range(T):
        alpha = t / (T - 1)  # Alpha goes from 0 to 1
        point = (
            start[0] + alpha * (end[0] - start[0]),
            start[1] + alpha * (end[1] - start[1]),
            start[2] + alpha * (end[2] - start[2])
        )
        # Adding noise only to interior points
        if 0 < t < T-1:
            noise = np.random.normal(0.0, noise_std, size=3)
            point = (
                point[0] + noise[0],
                point[1] + noise[1],
                max(point[2] + noise[2], 0.0)
            )
        interpolated_path.append(point)
    return interpolated_path

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
            path.append((x[idx], x[idx+1], x[idx+2])) 
            idx += 3
        paths.append(path)
    return paths

# Global variables to hold the current mission's paths and bounds.
x0 = flatten_paths(drones)
bounds = make_bounds(drones)

# ----------------------------
# Barrier function definitions
# ----------------------------
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
    z_safe = max(z, 1e-4)
    return h_eps(z_safe, eps) / (z_safe ** (n - 1))

def cubic_spline_B_derivative(v):
    if v < 1:
        return -2*v + 1.5 * v ** 2
    elif 1 <= v < 2:
        return -0.5 * (2 - v)**2
    else:
        return 0.0

def barrier_force_cubic_spline(pos1, pos2, eps):
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    dz = pos1[2] - pos2[2]
    z2 = dx*dx + dy*dy + dz*dz

    if z2 < 1e-12:
        return (0, 0, 0)
    
    eps2 = eps * eps
    if z2 >= eps2:
        return (0, 0, 0)

    z = math.sqrt(z2 + 1e-8)
    # Computing h(z)
    v = 2 * z / eps
    B = cubic_spline_B(v)
    h = 1.5 * B

    # Computing h'(z)
    B_prime = cubic_spline_B_derivative(v)
    h_prime = (3 / eps) * B_prime

    n = 3  # since we're using 3D

    # p_prime (Negative gradient of the penalty function with respect to z)
    denom = max(z**(n+1), 1e-6)
    p_prime = (z * h_prime - (n-1) * h) / denom

    # gradient = dp/dz * (x - y)
    grad = (p_prime * dx, p_prime * dy, p_prime * dz)
    return grad

def total_path_energy(paths:list[Path]): 
    ref_x0 = unflatten_paths(x0, len(paths))
    total_path_length_energy = 0.0
    for path in paths:
        for k in range(len(path) - 1):
            diff = sub_vecs(path[k+1], path[k])
            total_path_length_energy += vec_length(diff)

    total_barrier_energy = 0.0
    for k in range(T):
        for i in range(len(paths)): 
            for j in range(i+1, len(paths)): 
                (dx, dy, dz) = sub_vecs(paths[i][k], paths[j][k])
                distance = vec_length((dx, dy, dz))
                total_barrier_energy += dt * p_eps(distance, eta_safety_distance, 3)

    total_diversion_energy = 0.0
    for i, path in enumerate(paths):
        for k in range(len(path)):
            diff = sub_vecs(path[k], ref_x0[i][k])
            total_diversion_energy += dt * squared_vec_length(diff)

    total_obstacle_barrier_energy = 0.0
    for k in range(T):
        for i in range(len(paths)):
            for obs in obstacles: 
                (dx, dy, dz) = sub_vecs(paths[i][k], obs)
                distance = vec_length((dx, dy, dz))
                total_obstacle_barrier_energy += dt * p_eps(distance, eta_safety_distance, 3)

    return (total_path_length_energy, total_barrier_energy, total_diversion_energy, total_obstacle_barrier_energy)

def gradient_barrier_energy(paths:list[Path]):
    grad_smooth:list[Path] = [[(0,0,0) for _ in range(T)] for _ in range(len(paths))]
    grad_barrier:list[Path] = [[(0,0,0) for _ in range(T)] for _ in range(len(paths))]
    grad_divert:list[Path] = [[(0,0,0) for _ in range(T)] for _ in range(len(paths))]
    grad_obstac:list[Path] = [[(0,0,0) for _ in range(T)] for _ in range(len(paths))]
    ref_x0 = unflatten_paths(x0, len(paths))
    eps = 1e-8

    for i in range(len(paths)):
        # Smoothness gradient:
        # k = 0
        prev = sub_vecs(paths[i][1], paths[i][0])
        norm = math.sqrt(squared_vec_length(prev) + 1e-8)
        grad_smooth[i][0] = scale_vec(-1 / norm, prev)
        # k = T-1
        prev = sub_vecs(paths[i][T-1], paths[i][T-2])
        norm = math.sqrt(squared_vec_length(prev) + 1e-8)
        grad_smooth[i][T-1] = scale_vec(1 / norm, prev)
        for k in range(T):
            if k == 0 or k == T-1:
                continue
            prev = sub_vecs(paths[i][k], paths[i][k-1])
            next = sub_vecs(paths[i][k+1], paths[i][k])
            norm_prev = max(vec_length(prev), eps)
            norm_next = max(vec_length(next), eps)
            term1 = (prev[0]/norm_prev, prev[1]/norm_prev, prev[2]/norm_prev)
            term2 = (next[0]/norm_next, next[1]/norm_next, next[2]/norm_next)
            grad_smooth[i][k] = sub_vecs(term1, term2)
        
        # Diversion gradient: simply the vector from the current point to the reference point, 
        # scaled by 2 since we are using the squared distance as energy.
        for k in range(T):
            if k == 0 or k == T-1:
                continue
            else:
                grad_divert[i][k] = scale_vec(
                    2 * dt,
                    sub_vecs(
                        paths[i][k],
                        ref_x0[i][k]
                    )
                )

        # The gradient of the barrier energy 
        for k in range(T):
            per_point_barrier_force = (0, 0, 0)
            for j in range(len(paths)):
                if j == i:
                    continue
                else: 
                    per_point_barrier_force = add_vecs(per_point_barrier_force, barrier_force_cubic_spline(paths[i][k], paths[j][k], eta_safety_distance))
            grad_barrier[i][k] = scale_vec(dt, per_point_barrier_force)

        # The gradient of the obstacle barrier energy.
        for k in range(T):
            per_point_obs_force = (0, 0, 0)
            for obs in obstacles:
                per_point_obs_force = add_vecs(
                    per_point_obs_force,
                    barrier_force_cubic_spline(paths[i][k], obs, eta_safety_distance)
                )
            grad_obstac[i][k] = scale_vec(dt, per_point_obs_force)   
                
    return (grad_smooth, grad_barrier, grad_divert, grad_obstac)

def objective(x: np.ndarray, num_paths: int):
    paths = unflatten_paths(x, num_paths)
    path_energy, barrier_energy, diversion_energy, obstacle_barrier_energy = total_path_energy(paths)
    return path_energy + barrier_energy + diversion_energy + obstacle_barrier_energy

def objective_grad(x: np.ndarray, num_paths: int):
    paths = unflatten_paths(x, num_paths)
    grad_smooth, grad_barrier, grad_divert, grad_obstac = gradient_barrier_energy(paths)

    grad = []
    for i in range(len(paths)):
        for k in range(T):
            g = add_vecs(add_vecs(add_vecs(grad_smooth[i][k], grad_barrier[i][k]), grad_divert[i][k]), grad_obstac[i][k])
            grad.extend(g)

    return np.array(grad)

def update_mission(mission):
    global drones, x0, bounds
    drones_x0 = [
        interpolate_path(mission[i], noise_std=0.01)
        for i in range(len(mission))
    ]
    drones = [
        interpolate_path(mission[i])
        for i in range(len(mission))
    ] 
    x0 = flatten_paths(drones_x0)
    bounds = make_bounds(drones)


# ==========================================================================
# =================================== Testing ==============================
# ==========================================================================


# ----------------------------
# Barrier OG missions 
# ----------------------------
def compute_paths():
    num_drones = 4
    update_mission(mission_OG) 

    res = minimize(
        objective,
        x0,
        args=(len(drones),),
        method='L-BFGS-B', 
        jac=objective_grad,
        bounds=bounds,
        options={'maxiter': 100000, 'disp': True}
    )

    new_paths = unflatten_paths(res.x, len(drones))

    # Plotting the paths in matplotlib to visualize the result
    if PLOT: 
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        colors = ['r', 'g', 'b', 'm']
        for i, path in enumerate(new_paths):
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            zs = [p[2] for p in path]
            ax.plot(xs, ys, zs, color=colors[i], label=f'Drone {i}')

        
        # ploting each single waypoint in the paths as a scatter plot
        for i, path in enumerate(new_paths):
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            zs = [p[2] for p in path]
            ax.scatter(xs, ys, zs, color=colors[i], s=20)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        # set axis of 3*3*3 meter cube around the origin
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)
        ax.set_zlim(0, 1.5)
        ax.set_title('Optimized Paths with Barrier Function')
        ax.legend()
        plt.show()

    if not res.success:
        print(f"Failed to find a solution within {TIME_LIMIT_SECONDS} seconds.")
    
    # Missions: ((x,y,z), (x,y,z)) in grid coordinates
    # turn new_paths into a sequence  of waypoints for each drone, where each waypoint is a tuple of (x,y,z) coordinates. 
    global sequences
    sequences = {
        0: [(new_paths[0][k][0], new_paths[0][k][1], new_paths[0][k][2], 0.0) for k in range(T)],
        1: [(new_paths[1][k][0], new_paths[1][k][1], new_paths[1][k][2], 0.0) for k in range(T)],
        2: [(new_paths[2][k][0], new_paths[2][k][1], new_paths[2][k][2], 0.0) for k in range(T)],
        3: [(new_paths[3][k][0], new_paths[3][k][1], new_paths[3][k][2], 0.0) for k in range(T)]
    }
    # add a final landing point on the ground
    sequences[0].append((new_paths[0][-1][0], new_paths[0][-1][1], 0.0, 0.0))
    sequences[1].append((new_paths[1][-1][0], new_paths[1][-1][1], 0.0, 0.0))
    sequences[2].append((new_paths[2][-1][0], new_paths[2][-1][1], 0.0, 0.0))
    sequences[3].append((new_paths[3][-1][0], new_paths[3][-1][1], 0.0, 0.0))
    
# ----------------------------
# Crazyflie URIs (edit as needed)
# ----------------------------
uri1 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701')
uri2 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E705')
uri3 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E707')
uri4 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E709')
uris = [uri1, uri2, uri3, uri4]
curr_pos = {uri: (0, 0, 0) for uri in uris}
# This will keep track of the current timestep for each drone during the flight, so we know which point in the sequence to send next.
current_timestep_dict = {uri: 0 for uri in uris} 
last_drone_timestep = 0
num_runs = 2
if num_runs > 1:
    return_to_origin = True
else:
    return_to_origin = False

return_to_origin_paths = { # Mission 1
    0: [(1.0, 1.0, 0.0, 0.0),
        (1.0, 1.0, 0.3, 0.0),
        (-1.0, -1.0, 0.3, 0.0),
        (-1.0, -1.0, 0.0, 0.0)],
    1: [(1.0, -1.0, 0.0, 0.0),
        (1.0, -1.0, 0.6, 0.0),
        (-1.0, 1.0, 0.6, 0.0),
        (-1.0, 1.0, 0.0, 0.0)],
    2: [(0.0, 1.0, 0.0, 0.0),
        (0.0, 1.0, 0.9, 0.0),
        (0.0, -1.0, 0.9, 0.0),
        (0.0, -1.0, 0.0, 0.0)],
    3: [(0.6, 1.0, 0.0, 0.0),
        (0.6, 1.0, 1.2, 0.0),
        (0.6, -1.0, 1.2, 0.0),
        (0.6, -1.0, 0.0, 0.0)
    ]
}

return_to_origin_paths_ = { # Cross
    0: [(1.0, 1.0, 0.0, 0.0),
        (1.0, 1.0, 0.3, 0.0),
        (-0.2, -0.2, 0.3, 0.0),
        (-0.2, -0.2, 0.0, 0.0)],
    1: [(1.0, -1.0, 0.0, 0.0),
        (1.0, -1.0, 0.6, 0.0),
        (-0.2, 0.2, 0.6, 0.0),
        (-0.2, 0.2, 0.0, 0.0)],
    2: [(-1.0, -1.0, 0.0, 0.0),
        (-1.0, -1.0, 0.9, 0.0),
        (0.2, 0.2, 0.9, 0.0),
        (0.2, 0.2, 0.0, 0.0)],
    3: [(-1.0, 1.0, 0.0, 0.0),
        (-1.0, 1.0, 1.2, 0.0),
        (0.2, -0.2, 1.2, 0.0),
        (0.2, -0.2, 0.0, 0.0)
    ]
}

runtimes = []
backup_sequences = {}

position_params = {
    uri1: [0],
    uri2: [1],
    uri3: [2],
    uri4: [3],
}

def path_to_sequence(path: List[Tuple[int, int, int]]) -> List[Tuple[float, float, float, float]]:
    seq: List[Tuple[float, float, float, float]] = []
    for x, y, z in path:
        seq.append((float(x)/100*20, float(y)/100*20, (float(z)+2)/100*20, 0.0))
    last_x, last_y, _ = path[-1]
    seq.append((float(last_x)/100*20, float(last_y)/100*20, 0.0, 0.0))
    return seq

def square_dist(pos1, pos2):
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    dz = pos1[2] - pos2[2]
    return dx*dx + dy*dy + dz*dz

# ----------------------------
# Crazyflie control helpers
# ----------------------------
def take_off(cf, position):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position[2] / take_off_time

    print(f"take off at {position[2]}")
    for _ in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

def position_callback(uri, timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    curr_pos[uri] = (x, y, z)
    # find the minimum timestep across all drones
    global last_drone_timestep
    last_drone_timestep = min(current_timestep_dict.values())
    print(f'[{uri}] pos: ({x:.3f}, {y:.3f}, {z:.3f})')

def start_position_printing(scf):
    uri = scf.cf.link_uri
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(partial(position_callback, uri))
    log_conf.start()

def run_sequence(scf, num_seq):
    cf = scf.cf

    cf.platform.send_arming_request(True)
    time.sleep(1.0)
    me = uris[num_seq]

    take_off(cf, sequences[num_seq][0])
    time.sleep(1.0)
    start = time.perf_counter()

    for position in sequences[num_seq]:
        print(f"[{scf.cf.link_uri}] Setting position {position}")
        while (square_dist(curr_pos[me], position) > 0.025) and not (current_timestep_dict[me] > last_drone_timestep + 1):
            cf.commander.send_position_setpoint(position[0],
                                                 position[1], 
                                                 position[2], 
                                                 position[3])
            time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    cf.commander.send_notify_setpoint_stop()

    end = time.perf_counter()
    runtime = end - start
    runtimes.append(runtime)

    time.sleep(0.1)

def update_to_return():
    global sequences, backup_sequences
    backup_sequences = sequences
    sequences = return_to_origin_paths

def update_to_old_sequences():
    global sequences, runtimes
    sequences = backup_sequences
    runtimes = []

# ----------------------------
# Main
# ----------------------------
if __name__ == "__main__":
    backup_sequences = sequences.copy()
    # Fly
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    # Build args_dict for swarm.parallel_safe: each URI gets (index, sequences)
    args_dict = {
        uri1: [0],
        uri2: [1],
        uri3: [2],
        uri4: [3]
        }

    for _ in range(num_runs):  # Running the sequence num times
        compute_paths()
        with Swarm(uris, factory=factory) as swarm:
            swarm.parallel_safe(start_position_printing)
            swarm.parallel_safe(run_sequence, args_dict=args_dict)

        # calculating average runtimes and write to file
        avg_runtime = sum(runtimes) / len(runtimes)

        with open("timings_Barrier_Path_plan_18_may.txt", "a") as f:
            f.write(f"Barrier path planning algorithm runs in: {avg_runtime}\n")

        if return_to_origin:
            update_to_return()
            with Swarm(uris, factory=factory) as swarm:
                swarm.parallel_safe(start_position_printing)
                swarm.parallel_safe(run_sequence, args_dict=args_dict)
            update_to_old_sequences()
