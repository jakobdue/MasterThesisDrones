from turtle import fd
import matplotlib.pyplot as plt
from scipy.optimize import minimize, check_grad
import numpy as np
import random
import math
import time




type Point = tuple[float, float, float]
type Path  = list[Point] # points is a list of tuples with (x,y,z) coordinates.


# ----------------------------
# 1. Settings
# ----------------------------
TIME_LIMIT_SECONDS = 15
RESULTS_FILE = "BarrierPathPlanningAllTests.txt"
CIRCLE_RADIUS = 3/5
PLOT_OBSTACLE = False

random.seed(42)  # For reproducibility

# constants: 
eta_safety_distance = 0.40
T = 10
dt = 1 / (T - 1) # Normalize to the number of timesteps.
obstacles = set() # Set of tuples with (x,y,z) coordinates of obstacles. This is global so it can be accessed in the barrier function.

# Missions
mission_Cross = { # This is real world coordinates
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

missions = [mission_OG, mission_Cross]

mission_start_obstacle_run = [(-5, -5, 4), (-5, 5, 4), (5, 0, 2),  (5,  0, 7)]
mission_goal_obstacle_run  = [(5,   5, 4), (5, -5, 4), (-5, 0, 7), (-5, 0, 2)]

# Divide by 5 to get the coordinates in the same scale as the other missions for real world 
def scale_vec(scaler, vec):
    return (vec[0] * scaler, vec[1] * scaler, vec[2] * scaler)

obstacle_run_mission = {
    0: [scale_vec(1/5, mission_start_obstacle_run[0]), scale_vec(1/5, mission_goal_obstacle_run[0])],
    1: [scale_vec(1/5, mission_start_obstacle_run[1]), scale_vec(1/5, mission_goal_obstacle_run[1])],
    2: [scale_vec(1/5, mission_start_obstacle_run[2]), scale_vec(1/5, mission_goal_obstacle_run[2])],
    3: [scale_vec(1/5, mission_start_obstacle_run[3]), scale_vec(1/5, mission_goal_obstacle_run[3])],
}

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

def generate_random_missions(num_drones=4):
    X_MIN, X_MAX = -5, 5 # We use the same bounds as for A* to make it compareble.
    Y_MIN, Y_MAX = -5, 5
    Z_MIN, Z_MAX = 0, 9
    while True:
        starts, goals = [], []
        for _ in range(num_drones):
            start = (random.randint(X_MIN + 1, X_MAX - 1), random.randint(Y_MIN + 1, Y_MAX - 1), random.randint(Z_MIN + 1, Z_MAX - 1))
            goal = (random.randint(X_MIN + 1, X_MAX - 1), random.randint(Y_MIN + 1, Y_MAX - 1), random.randint(Z_MIN + 1, Z_MAX - 1))
            starts.append(start)
            goals.append(goal)
        # Add the two lists and check for dublicates, if there are any, generate new ones
        if len(set(starts + goals)) == 2 * num_drones:
            break
    sequences = {i: [scale_vec(1/5,starts[i]), scale_vec(1/5,goals[i])] for i in range(num_drones)}

    return sequences

def update_random_obstacles(num_obstacles, start, goal):
    X_MIN, X_MAX = -5, 5
    Y_MIN, Y_MAX = -5, 5
    Z_MIN, Z_MAX = 0, 9
    global obstacles
    obstacles = set()
    new_obstacles_generated = True
    # Also check that the time does not run out 
    deadline = time.perf_counter() + TIME_LIMIT_SECONDS
    while len(obstacles) < num_obstacles:
        obs = (random.randint(X_MIN + 1, X_MAX - 1), random.randint(Y_MIN + 1, Y_MAX - 1), random.randint(Z_MIN + 1, Z_MAX - 1))
        # Check that the generated obstacle is not start nor goal:
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


def generate_circle_crossing(num_drones, radius=CIRCLE_RADIUS, z=0.7):
    sequences = {}
    for i in range(num_drones):
        theta = 2 * np.pi * i / num_drones
        start = (
            radius * np.cos(theta),
            radius * np.sin(theta),
            z)
        
        end = (
            -start[0],
            -start[1],
            z)
        sequences[i] = [start, end]

    return sequences


# Interpolate points in the paths to have T points in total. There is always two points in the path, so we can interpolate T-2 points between them.
def interpolate_path(path: Path, noise_std: float = 0.001) -> Path:
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

# Global variables to hold the current mission's paths and bounds, which will be updated before each optimization run.
drones = []
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
    # Compute h(z)
    v = 2 * z / eps
    B = cubic_spline_B(v)
    h = 1.5 * B

    # Compute h'(z)
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
    # calculate the total lenght of a path by summing the distances between consecutive points only taking the square root in the end to save computation time.
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
                total_barrier_energy += p_eps(distance, eta_safety_distance, 3)

    total_diversion_energy = 0.0
    for i, path in enumerate(paths):
        for k in range(len(path)):
            diff = sub_vecs(path[k], ref_x0[i][k])
            total_diversion_energy += squared_vec_length(diff)

    total_obstacle_barrier_energy = 0.0
    for k in range(T):
        for i in range(len(paths)):
            for obs in obstacles: # Push away from obstacles in the same way as from other drones, we can use the same p_eps function since it only depends on the distance.
                (dx, dy, dz) = sub_vecs(paths[i][k], obs)
                distance = vec_length((dx, dy, dz))
                total_obstacle_barrier_energy += p_eps(distance, eta_safety_distance, 3)

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
        
        # Diversion gradient: simply the vector from the current point to the reference point, scaled by 2 since we are using the squared distance as energy.
        for k in range(T):
            if k == 0 or k == T-1:
                continue
            else:
                grad_divert[i][k] = scale_vec(
                    2,
                    sub_vecs(
                        paths[i][k],
                        ref_x0[i][k]
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

        # compute the gradient of the obstacle barrier energy with respect to the position of each point in the path.
        for k in range(T):
            per_point_obs_force = (0, 0, 0)
            for obs in obstacles:
                per_point_obs_force = add_vecs(
                    per_point_obs_force,
                    barrier_force_cubic_spline(paths[i][k], obs, eta_safety_distance)
                )
            grad_obstac[i][k] = per_point_obs_force   
                
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
    global drones
    drones = [
        interpolate_path(mission[i])
        for i in range(len(mission))
    ]
    global x0, bounds
    x0 = flatten_paths(drones)
    bounds = make_bounds(drones)


# ==========================================================================
# =================================== Testing ==============================
# ==========================================================================



# ----------------------------
# Barrier OG missions 
# ----------------------------
def Barrier_missions(missions):
    num_drones = 4
    
    # Store results for each successful run
    results = []
    
    for mission in missions: 
        update_mission(mission)

        start_time = time.perf_counter()
        res = minimize(
            objective,
            x0,
            args=(len(drones),),
            method='L-BFGS-B', 
            jac=objective_grad,
            bounds=bounds,
            options={'maxiter': 100000, 'disp': True}
        )
        elapsed = time.perf_counter() - start_time

        new_paths = unflatten_paths(res.x, len(drones))

        if not res.success:
            print(f"Failed to find a solution within {TIME_LIMIT_SECONDS} seconds.")
            break

        avg_length = compute_average_path_length(new_paths)

        results.append({
            "num_drones": num_drones,
            "steps": len(new_paths[0]),
            "time": elapsed,
            "Mission": "OG" if mission == mission_OG else "Cross",
            "average_length": avg_length
        })

    # Save results to file
    with open(RESULTS_FILE, "a") as f:
        f.write("\nBarrier pathplanning results: Missions:\n")
        f.write(f"Results for original missions, 4 drones:\n")
        for result in results:
            f.write(
                f"Drones: {result['num_drones']}, "
                f"Steps: {result['steps']}, "
                f"Time: {result['time']:.3f} s, "
                f"Mission: {result['Mission']}, "
                f"Average length: {result['average_length']:.2f}\n"
            )

        f.write("\n")



# ----------------------------
# Barrier Circle Crossing missions
# ----------------------------
def Barrier_Circle_crossing():
    num_drones = 1
    last_successful_num_drones = 0
    reason_for_failure = ""
    results = []

    deadline = time.perf_counter() + TIME_LIMIT_SECONDS
    while True:
        pairs = generate_circle_crossing(num_drones)
        update_mission(pairs)
        start_time = time.perf_counter()

        def time_callback(xk):
            if time.perf_counter() > deadline:
                raise TimeoutError

        try:
            res = minimize(
                objective,
                x0,
                args=(len(drones),),
                method='L-BFGS-B',
                jac=objective_grad,
                bounds=bounds,
                callback=time_callback,
                options={'maxiter': 100000, 'disp': False}
            )
            success = res.success
        except TimeoutError:
            success = False
            reason_for_failure = f"Stopped due to time limit of {TIME_LIMIT_SECONDS} seconds."
        elapsed = time.perf_counter() - start_time
        
        if not success:
            print(f"No more solutions can be found within the time limit of {TIME_LIMIT_SECONDS} seconds.")
            break

        new_paths = unflatten_paths(res.x, len(drones))
        avg_length = compute_average_path_length(new_paths)
        results.append({
            "num_drones": num_drones,
            "steps": len(new_paths[0]),
            "time": elapsed,
            "average_length": avg_length
        })
        
        last_successful_num_drones = num_drones
        num_drones += 1

    # Save results to file
    with open(RESULTS_FILE, "a") as f:
        f.write("\nBarrier Path Planning: Circle crossing:\n")
        for result in results:
            f.write(
                f"Drones: {result['num_drones']}, "
                f"Steps: {result['steps']}, "
                f"Time: {result['time']:.3f} s, "
                f"Average length: {result['average_length']:.2f}\n"
            )

        f.write("\n")
        f.write(f"Maximum number of drones: {last_successful_num_drones}\n")
        f.write(f"{reason_for_failure}")
        f.write("\n\n")


# ----------------------------
# Barrier Random missions
# ----------------------------
def Barrier_Random_Missions():
    num_drones = 4
    num_successful_path = 0
    lenghts = []

    deadline = time.perf_counter() + TIME_LIMIT_SECONDS
    while True:
        pairs = generate_random_missions(num_drones)
        update_mission(pairs)

        res = minimize(
            objective,
            x0,
            args=(len(drones),),
            method='L-BFGS-B',
            jac=objective_grad,
            bounds=bounds,
            options={'maxiter': 100000, 'disp': False}
        )
        
        if deadline - time.perf_counter() <= 0:
            print(f"No more solutions can be found within the time limit of {TIME_LIMIT_SECONDS} seconds.")
            break

        new_paths = unflatten_paths(res.x, len(drones))
        avg_length = compute_average_path_length(new_paths)
        lenghts.append(avg_length)
        num_successful_path += 1

    # Save results to file
    with open(RESULTS_FILE, "a") as f:
        f.write("\nBarrier Path Planning results: Random missions:\n")
        f.write(
                f"Total successful paths found: {num_successful_path}\n"
                f"Average path length across successful runs: {sum(lenghts) / len(lenghts):.2f}\n"
            )

        f.write("\n")



# ----------------------------
# Barrier with increasing number of obstacles
# ----------------------------
def Barrier_Increasing_Obstacles():
    num_drones = 4
    num_obstacles = 1
    new_obstacles_generated = True
    lenghts = []
    results = []

    deadline = time.perf_counter() + TIME_LIMIT_SECONDS
    while True:
        update_mission(obstacle_run_mission)
        new_obstacles_generated = update_random_obstacles(num_obstacles, mission_start_obstacle_run, mission_goal_obstacle_run)
        
        start_time = time.perf_counter()

        def time_callback(xk):
            if time.perf_counter() > deadline:
                raise TimeoutError

        try:
            res = minimize(
                objective,
                x0,
                args=(len(drones),),
                method='L-BFGS-B',
                jac=objective_grad,
                bounds=bounds,
                callback=time_callback,
                options={'maxiter': 100000, 'disp': False}
            )
            success = res.success
        except TimeoutError:
            success = False
        elapsed = time.perf_counter() - start_time

        global obstacles
        local_obs = obstacles.copy()
        new_paths = unflatten_paths(res.x, num_drones)
        if success:
            results.append((new_paths, local_obs, elapsed))
        
        if not success:
            print(f"No more solutions can be found within the time limit of {TIME_LIMIT_SECONDS} seconds.")
            break

        avg_length = compute_average_path_length(new_paths)
        lenghts.append(avg_length)
        num_obstacles += 1
        # Option to plot path with 20 obstacles
        if PLOT_OBSTACLE and num_obstacles == 20 and new_paths is not None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

            # Plot obstacles
            if local_obs:
                obs_x, obs_y, obs_z = zip(*local_obs)
                ax.scatter(obs_x, obs_y, obs_z, c='red', marker='x', label='Obstacles')

            # Plot paths
            for j, path in enumerate(new_paths):
                x, y, z = zip(*path)
                ax.plot(x, y, z, label=f'Drone {j+1}')

            ax.set_title(f'Barrier Path Planning with {num_obstacles} Obstacles')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.legend()
            plt.show()
            
    # Save results to file
    with open(RESULTS_FILE, "a") as f:
        f.write("\nBarrier Path Planning results: Random obstacles:\n")
        f.write(
            f"Max number of obstacles solved within {TIME_LIMIT_SECONDS} seconds: {num_obstacles - 1}\n"
            f"Average path length for all successful runs: {sum(lenghts) / len(lenghts):.2f}\n" 
        )
        
        f.write("Every tenth path plotted in ObstaclesFiguresBarrier/ as Barrier_path_i.png\n")
        if new_obstacles_generated:
            f.write("Stopped due to time limit\n")
        else:
            f.write("Stopped due to space limit, could not generate more unique obstacles\n")
        f.write("\n")

    # plot every tenth path figs and save as pngs
    for i, (paths, obs_set, elapsed) in enumerate(results):

        if i % 10 == 0 and paths is not None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

            # Plot obstacles
            if obs_set:
                obs_x, obs_y, obs_z = zip(*obs_set)
                ax.scatter(obs_x, obs_y, obs_z, c='red', marker='x', label='Obstacles')

            # Plot paths
            for j, path in enumerate(paths):
                x, y, z = zip(*path)
                ax.plot(x, y, z, label=f'Drone {j+1}')

            ax.set_title(f'Barrier Path Planning with {len(obs_set)} Obstacles')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.legend()
            plt.savefig(f'ObstaclesFiguresBarrier/Barrier_path_{i}.png')
            plt.close()
        


# ----------------------------
# Gradient check
# ----------------------------
def check_gradient():
    # --- 1. Use a small deterministic test ---
    global T
    T_backup = T
    T = 5  # smaller problem for stability

    # Temporary mission (small, simple)
    test_mission = {
        0: [(-0.5, -0.5, 0.7), (0.5, 0.5, 0.7)],
        1: [(-0.5,  0.6, 0.7), (0.5, -0.4, 0.7)],  # small offset
        }

    # No obstacles for first test
    global obstacles
    obstacles = set()
    update_mission(test_mission)

    # --- 2. Wrap functions (remove extra args) ---
    def f_wrapped(x):
        return objective(x, len(drones))

    def grad_wrapped(x):
        return objective_grad(x, len(drones))

    # --- 3. Run check ---
    error = check_grad(f_wrapped, grad_wrapped, x0)

    # --- 4. Test WITH obstacles ---
    update_random_obstacles(1,[],[])  # add one obstacle

    error_obs = check_grad(f_wrapped, grad_wrapped, x0)

    # --- 5. Restore state ---
    T = T_backup

    return error, error_obs


# ----------------------------
# Gradient check per component
# ----------------------------
def check_gradient_components():

    # small deterministic setup
    global obstacles
    obstacles = set()

    # --- OBSTACLES ---
    missions = generate_random_missions(2)
    update_mission(missions)
    # collapse missions to one list of all start and goal positions to check that the generated obstacles do not overlap with any of them, if they do, generate new ones until they don't.
    missions_list = [pos for drone in missions.values() for pos in drone]
    update_random_obstacles(5, missions_list,[])

    def wrap(f):
        return lambda x: f(x, len(drones))

    x = x0.copy()

    # --- FULL ---
    full = check_grad(wrap(objective), wrap(objective_grad), x)

    # --- PATH LENGTH ONLY ---
    def f_path(x):
        paths = unflatten_paths(x, len(drones))
        return total_path_energy(paths)[0]

    def g_path(x):
        paths = unflatten_paths(x, len(drones))
        g, _, _, _ = gradient_barrier_energy(paths)
        return np.array([c for path in g for p in path for c in p])

    path = check_grad(f_path, g_path, x)

    # --- DRONE BARRIER ---
    def f_barrier(x):
        paths = unflatten_paths(x, len(drones))
        return total_path_energy(paths)[1]

    def g_barrier(x):
        paths = unflatten_paths(x, len(drones))
        _, g, _, _ = gradient_barrier_energy(paths)
        return np.array([c for path in g for p in path for c in p])

    barrier = check_grad(f_barrier, g_barrier, x)

    # --- DIVERSION ---
    def f_div(x):
        paths = unflatten_paths(x, len(drones))
        return total_path_energy(paths)[2]

    def g_div(x):
        paths = unflatten_paths(x, len(drones))
        _, _, g, _ = gradient_barrier_energy(paths)
        return np.array([c for path in g for p in path for c in p])

    diversion = check_grad(f_div, g_div, x)

    # --- OBSTACLE ---
    def f_obs(x):
        paths = unflatten_paths(x, len(drones))
        return total_path_energy(paths)[3]

    def g_obs(x):
        paths = unflatten_paths(x, len(drones))
        _, _, _, g = gradient_barrier_energy(paths)
        return np.array([c for path in g for p in path for c in p])

    obstacle = check_grad(f_obs, g_obs, x)
    return full, path, barrier, diversion, obstacle




# ----------------------------
# Call all the functions and save results
# ----------------------------
if __name__ == "__main__":

    error, error_obs = check_gradient()
    avg = []
    for _ in range(100):
        avg.append(check_gradient_components())
    avg_full = sum([a[0] for a in avg]) / len(avg)
    avg_path = sum([a[1] for a in avg]) / len(avg)
    avg_barrier = sum([a[2] for a in avg]) / len(avg)
    avg_diversion = sum([a[3] for a in avg]) / len(avg)
    avg_obstacle = sum([a[4] for a in avg]) / len(avg)

    with open(RESULTS_FILE, "w") as f:
        f.write(f"Barrier pathplanning Benchmark Results with time limit: {TIME_LIMIT_SECONDS} seconds\n")
        f.write("====================================================\n\n")
        f.write("Gradient check results:\n")
        f.write("First single run with full gradient check none and 1 obstacle:\n")
        f.write(f"Gradient error (no obstacles): {error:.6e}\n")
        f.write(f"Gradient error (with obstacle): {error_obs:.6e}\n\n")
        f.write("Average gradient check results over 100 runs:\n")
        f.write(f"FULL: {avg_full:.6e}\n")
        f.write(f"PATH: {avg_path:.6e}\n")
        f.write(f"BARRIER: {avg_barrier:.6e}\n")
        f.write(f"DIVERSION: {avg_diversion:.6e}\n")
        f.write(f"OBSTACLE: {avg_obstacle:.6e}\n")
        f.write("====================================================\n\n")
    
    print(f"Results file {RESULTS_FILE} initialized.")
    
    update_random_obstacles(0,[],[])  # initialize obstacles to empty set
    print("Starting Barrier pathplanning benchmarks...\n")
    Barrier_missions(missions)
    print("\n\n")

    print("Starting Barrier Circle Crossing benchmarks...\n")
    Barrier_Circle_crossing()
    print("\n\n")

    print("Starting Barrier Random Missions benchmarks...\n")
    Barrier_Random_Missions()
    print("\n\n")   

    print("Starting Barrier Increasing Obstacles benchmarks...\n")
    Barrier_Increasing_Obstacles()
    print("\n\n") 

    print(f"Results saved to {RESULTS_FILE}")
    




