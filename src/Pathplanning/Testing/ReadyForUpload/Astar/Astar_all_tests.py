import heapq
import math
import random
import time
from itertools import product
from typing import Tuple, Set, Optional

import matplotlib.pyplot as plt
import numpy as np
 

# ----------------------------
# 1. Settings
# ----------------------------
ROOM_SCARLAR = 1
TIME_LIMIT_SECONDS = 60
RESULTS_FILE = "AstarAllTest.txt"
CIRCLE_RADIUS = 3
PLOT = True
SHOW_PLOTS = False
PRINT_FAILED_PATHS = False

random.seed(42)  # For reproducibility

X_MIN, X_MAX = -5, 5
Y_MIN, Y_MAX = -5, 5
Z_MIN, Z_MAX = 0, 9

JointState = Tuple[Tuple[int, int, int], ...]

SINGLE_DRONE_ACTIONS = [
    (0, 0, 0),
    (1, 0, 0), (-1, 0, 0),
    (0, 1, 0), (0, -1, 0),
    (0, 0, 1), (0, 0, -1),
]

# This is grid coordinates, not real world coordinates, divide by 5 to get 
# real world drone coordinates in meters, and add 0.7 to z to get the real
# world altitude in meters, since we want the drones to have some minimum 
# distance to the ground. 
missionOG_start = [(-5, -5, 0), (-5, 5, 0), (0, -5, 0), (3, -5, 0)]
missionOG_goal  = [(5, 5, 0), (5, -5, 0), (0, 5, 0), (3, 5, 0)]
mission_OG = (missionOG_start, missionOG_goal)

missions_start = [(-1, -1, 0), (-1, 1, 0), (1, 1, 0), (1, -1, 0)]
missions_goal  = [(5, 5, 0), (5, -5, 0), (-5, -5, 0), (-5, 5, 0)]
mission_Cross = (missions_start, missions_goal) 

mission_start_obstacle_run = [(-5, -5, 4), (-5, 5, 4), (5, 0, 2),  (5,  0, 7)]
mission_goal_obstacle_run  = [(5,   5, 4), (5, -5, 4), (-5, 0, 7), (-5, 0, 2)]
obstacle_run_mission = (mission_start_obstacle_run, mission_goal_obstacle_run)


# ----------------------------
# 2. Helpers
# ----------------------------
def in_bounds(p):
    x, y, z = p
    return X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX and Z_MIN <= z <= Z_MAX

def split_joint_path(joint_path, num_drones): # Convert from list of joint states to list of individual drone paths
    drone_paths = [[] for _ in range(num_drones)]
    for state in joint_path:
        for i in range(num_drones):
            drone_paths[i].append(state[i])
    return drone_paths

def compute_average_path_length(paths):
    total_length = 0
    count = 0
    # First scale all numbers by dividing by 5 to get real world coordinates in meters.
    paths = [[(p[0] / 5, p[1] / 5, p[2] / 5) for p in path] if path is not None else None for path in paths]

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

def generate_circle_crossing(num_drones, radius=None, z=1):
    if radius is None:
        radius = CIRCLE_RADIUS
    if num_drones <= 0:
        return []

    # ----------------------------
    # 1. Get integer circle boundary points
    # ----------------------------
    boundary = []

    for x in range(-radius, radius + 1):
        for y in range(-radius, radius + 1):
            dist = math.sqrt(x*x + y*y)
            if abs(dist - radius) <= 0.5:  # thin ring
                boundary.append((x, y, z))

    if len(boundary) < 2 * num_drones:
        raise ValueError(
            f"Not enough boundary points. Increase radius. "
            f"Have {len(boundary)}, need {2*num_drones}"
        )

    # ----------------------------
    # 2. Sort by angle
    # ----------------------------
    boundary.sort(key=lambda p: math.atan2(p[1], p[0]))

    # ----------------------------
    # 3. Evenly sample 2*n points
    # ----------------------------
    N = len(boundary)
    step = N / (2 * num_drones)

    sampled = []
    for i in range(2 * num_drones):
        idx = int(round(i * step)) % N
        sampled.append(boundary[idx])

    # Remove accidental duplicates
    sampled = list(dict.fromkeys(sampled))

    if len(sampled) < 2 * num_drones:
        raise ValueError(
            "Sampling collapsed due to duplicates. Increase radius."
        )

    # ----------------------------
    # 4. Pair opposites
    # ----------------------------
    pairs = []
    half = len(sampled) // 2

    for i in range(num_drones):
        start = sampled[i]
        goal  = sampled[i + half]
        pairs.append((start, goal))

    return pairs

def generate_random_missions(num_drones=4):
    while True:
        starts, goals = [], []
        for i in range(num_drones):
            start = (random.randint(X_MIN + 1, X_MAX - 1), 
                     random.randint(Y_MIN + 1, Y_MAX - 1), 
                     random.randint(Z_MIN + 1, Z_MAX - 1))
            goal = (random.randint(X_MIN + 1, X_MAX - 1), 
                    random.randint(Y_MIN + 1, Y_MAX - 1), 
                    random.randint(Z_MIN + 1, Z_MAX - 1))
            starts.append(start)
            goals.append(goal)
        # Add the two lists and check for duplicates, if there are any, generate new ones
        if len(set(starts + goals)) == 2 * num_drones:
            break
    return starts, goals

def generate_random_obstacles(num_obstacles):
    obstacles = set()
    new_obstacles_generated = True
    # Also check that the time does not run out 
    deadline = time.perf_counter() + TIME_LIMIT_SECONDS
    while len(obstacles) < num_obstacles:
        obs = (random.randint(X_MIN, X_MAX), random.randint(Y_MIN, Y_MAX), random.randint(Z_MIN, Z_MAX))
        # Check that not start or goal:
        if obs in mission_start_obstacle_run or obs in mission_goal_obstacle_run:
            continue
        obstacles.add(obs)
        if time.perf_counter() >= deadline:
            print(f"Time limit of {TIME_LIMIT_SECONDS} seconds reached while generating obstacles.")
            new_obstacles_generated = False
            break
    return (obstacles, new_obstacles_generated)

def update_room_size(room_scalar):
    global X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX, ROOM_SCARLAR, CIRCLE_RADIUS
    ROOM_SCARLAR = room_scalar
    CIRCLE_RADIUS = 3 * room_scalar
    X_MIN, X_MAX = -5 * room_scalar, 5 * room_scalar
    Y_MIN, Y_MAX = -5 * room_scalar, 5 * room_scalar
    Z_MIN, Z_MAX = 0, 9 * room_scalar

# ----------------------------
# 3. Joint A*
# ----------------------------
class JointAStar3D:
    def __init__(self, obstacles: Set[Tuple[int, int, int]] = None):
        self.obstacles = obstacles if obstacles else set()

    def is_valid(self, state: JointState):
        for p in state:
            if not in_bounds(p) or p in self.obstacles:
                return False

        for i in range(len(state)):
            for j in range(i + 1, len(state)):
                if state[i] == state[j]:
                    return False
        return True

    def no_swap(self, old, new):
        for i in range(len(old)):
            for j in range(i + 1, len(old)):
                if old[i] == new[j] and old[j] == new[i]:
                    return False
        return True

    def neighbors(self, state):
        for joint_action in product(SINGLE_DRONE_ACTIONS, repeat=len(state)):
            if all(a == (0, 0, 0) for a in joint_action):
                continue

            new_state = tuple(
                (
                    state[i][0] + joint_action[i][0],
                    state[i][1] + joint_action[i][1],
                    state[i][2] + joint_action[i][2],
                )
                for i in range(len(state))
            )

            if not self.is_valid(new_state):
                continue
            if not self.no_swap(state, new_state):
                continue

            yield new_state

    def heuristic(self, state, goal):
        return sum(
            abs(state[i][0] - goal[i][0]) +
            abs(state[i][1] - goal[i][1]) +
            abs(state[i][2] - goal[i][2])
            for i in range(len(state))
        )

    def solve(self, start, goal, time_limit: float) -> Optional[list]:
        deadline = time.perf_counter() + time_limit

        open_heap = []
        heapq.heappush(open_heap, (0, start))

        came_from = {}
        g_score = {start: 0}
        closed = set()

        while open_heap:
            if time.perf_counter() >= deadline:
                return None

            _, current = heapq.heappop(open_heap)

            if current in closed:
                continue
            closed.add(current)

            if current == goal:
                return self.reconstruct(came_from, current)

            for neighbor in self.neighbors(current):
                if time.perf_counter() >= deadline:
                    return None

                g = g_score[current] + 1

                if g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = g
                    f = g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (f, neighbor))

        return None

    def reconstruct(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]


# ----------------------------
# Hirachical A*
# ----------------------------
class AStar3D:
    def __init__(self, obstacles: Set[Tuple[int, int, int]]):
        self.obstacles = obstacles

    def in_bounds(self, node):
        x, y, z = node
        return (X_MIN <= x <= X_MAX and
                Y_MIN <= y <= Y_MAX and
                Z_MIN <= z <= Z_MAX)

    def neighbors(self, node):
        x, y, z = node
        directions = [
            (1, 0, 0), (-1, 0, 0),
            (0, 1, 0), (0, -1, 0),
            (0, 0, 1), (0, 0, -1)
        ]
        result = []
        for dx, dy, dz in directions:
            nxt = (x + dx, y + dy, z + dz)
            if self.in_bounds(nxt) and nxt not in self.obstacles:
                result.append(nxt)
        return result

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])

    def solve(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.neighbors(current):
                tentative_g = g_score[current] + 1

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

def plan_multiple_paths(pairs: list[Tuple[Tuple[int, int, int],
                                           Tuple[int, int, int]]], 
                        obstacles_in: Set[Tuple[int, int, int]] = None):

    obstacles = obstacles_in if obstacles_in else set()
    
    # Add missions start and end as obstacles to avoid collisions at start and end
    
    all_paths = []

    for start, goal in pairs:
        local_obstacles = set()
        for missions in pairs:
            if missions[0] != start or missions[1] != goal:
                local_obstacles.add(missions[0])  # start
                local_obstacles.add(missions[1])  # goal
        planner = AStar3D(local_obstacles.union(obstacles))
        path = planner.solve(start, goal)

        if path is None:
            if PRINT_FAILED_PATHS:
                print(f"No path found for {start} -> {goal}")
            all_paths.append(None)
        else:
            all_paths.append(path)
            obstacles.update(path)  # Path becomes obstacle
        
    return all_paths


# ==========================================================================
# =================================== Testing ==============================
# ==========================================================================


# ----------------------------
# Test OG missions joint A*
# ----------------------------

def Astar_Joint_missions(missions):
    num_drones = 4

    # Store results for each successful run
    results = []
    failed_runs = 0
    
    for mission in missions:  # You can add more missions here
        starts, goals = mission
        joint_start = tuple(starts)
        joint_goal = tuple(goals)

        planner = JointAStar3D()

        start_time = time.perf_counter()
        joint_path = planner.solve(joint_start, joint_goal, TIME_LIMIT_SECONDS)
        elapsed = time.perf_counter() - start_time

        if joint_path is None:
            print(f"Failed to find a solution within {TIME_LIMIT_SECONDS} seconds.")
            failed_runs += 1
            break

        avg_length = compute_average_path_length(split_joint_path(joint_path, num_drones))

        results.append({
            "num_drones": num_drones,
            "steps": len(joint_path),
            "time": elapsed,
            "Mission": "OG" if mission == mission_OG else "Cross",
            "average_length": avg_length
        })

    # Save results to file
    with open(RESULTS_FILE, "a") as f:
        f.write("\nJoint A* results: Missions:\n")
        f.write(f"Results for original missions, 4 drones:\n")
        for result in results:
            f.write(
                f"Drones: {result['num_drones']}, "
                f"Steps: {result['steps']}, "
                f"Time: {result['time']:.3f} s, "
                f"Mission: {result['Mission']}, "
                f"Average length: {result['average_length']:.2f}\n"
            )

        f.write(f"Failed runs: {failed_runs}\n")


# ----------------------------
# Hirachical A* for missions
# ----------------------------
def Astar_hirachical_missions(missions):
    failed_runs = 0
    with open(RESULTS_FILE, "a") as f:
        f.write("\nHirachical A* results: Missions:\n")
        f.write(f"Results for original missions, 4 drones:\n")
        for i, mission in enumerate(missions):
            mission_name = "OG" if i == 0 else "Cross"

            start_time = time.perf_counter()
            paths = plan_multiple_paths(mission)
            elapsed = time.perf_counter() - start_time

            if paths is None or any(p is None for p in paths):
                print(f"Mission {mission_name} failed to find paths for all pairs.")
                failed_runs += 1
                continue

            avg_length = compute_average_path_length(paths)
            lines = []
            lines.append({
                "num_drones": 4,
                "steps": max(len(p) for p in paths if p is not None),
                "time": elapsed,
                "Mission": mission_name,
                "average_length": avg_length
            })

            # write results
            for result in lines:
                f.write(
                    f"Drones: {result['num_drones']}, "
                    f"Steps: {result['steps']}, "
                    f"Time: {result['time']:.3f} s, "
                    f"Mission: {result['Mission']}, "
                    f"Average length: {result['average_length']:.2f}\n"
                )

        f.write(f"Failed runs: {failed_runs}\n")
            

# ----------------------------
# Circle crossing with joint A*
# ----------------------------
def joint_Astar_circle_crossing():
    update_room_size(10)
    num_drones = 1
    last_successful_num_drones = 0
    results = []
    last_path = None
    last_pairs = None
    failed_runs = 0

    deadline = time.perf_counter() + TIME_LIMIT_SECONDS
    while True:
        starts, goals = list(zip(*(generate_circle_crossing(num_drones))))  # Unzip pairs into separate lists
        joint_start = tuple(starts)
        joint_goal = tuple(goals)

        planner = JointAStar3D()

        start_time = time.perf_counter()
        joint_path = planner.solve(joint_start, joint_goal, TIME_LIMIT_SECONDS)
        elapsed = time.perf_counter() - start_time

        if joint_path is None or time.perf_counter() >= deadline:
            print(f"Failed to find a solution within {TIME_LIMIT_SECONDS} seconds.")
            failed_runs += 1
            break
        last_path = joint_path
        last_pairs = list(zip(starts, goals))

        avg_length = compute_average_path_length(split_joint_path(joint_path, num_drones))
        results.append({
            "num_drones": num_drones,
            "steps": len(joint_path),
            "time": elapsed,
            "average_length": avg_length
        })

        last_successful_num_drones = num_drones
        num_drones += 1

    # plot the last successful path
    if PLOT and last_path is not None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        drone_paths = split_joint_path(last_path, last_successful_num_drones)
        for j, path in enumerate(drone_paths):
            x, y, z = zip(*path)
            ax.plot(x, y, z, label=f'Drone {j+1}')

        # plot the circle for reference
        theta = np.linspace(0, 2 * np.pi, 100)
        x_circle = CIRCLE_RADIUS * np.cos(theta)
        y_circle = CIRCLE_RADIUS * np.sin(theta)
        z_circle = np.full_like(theta, 1)  # Circle at z=1
        ax.plot(x_circle, y_circle, z_circle, 'r--', label='Circle')

        for i in range(last_successful_num_drones):
            ax.scatter(last_pairs[i][0][0], last_pairs[i][0][1], last_pairs[i][0][2], c='green', marker='o', label=f'Start {i+1}' if i == 0 else "")
            ax.scatter(last_pairs[i][1][0], last_pairs[i][1][1], last_pairs[i][1][2], c='blue', marker='x', label=f'Goal {i+1}' if i == 0 else "")
        
        ax.set_title(f'Joint A* Circle Crossing with {last_successful_num_drones} Drones')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        plt.savefig(f'JointAstarCircleCrossing_{last_successful_num_drones}_drones.png')
        if SHOW_PLOTS:
            plt.show()
        

    # Save results to file
    with open(RESULTS_FILE, "a") as f:
        f.write("\nJoint A* results: Circle crossing:\n")
        for result in results:
            f.write(
                f"Drones: {result['num_drones']}, "
                f"Steps: {result['steps']}, "
                f"Time: {result['time']:.3f} s, "
                f"Average length: {result['average_length']:.2f}\n"
            )

        f.write("\n")
        f.write(f"Maximum number of drones: {last_successful_num_drones}\n")
        if time.perf_counter() >= deadline:
            f.write("failed due to time limit\n")
        else:
            f.write("failed due to feasibillity of path\n") 
        f.write(f"Failed runs: {failed_runs}\n")
        f.write("\n")
    update_room_size(1)

# ----------------------------
# Circle crossing with hirachical A*
# ----------------------------
def hirachical_Astar_circle_crossing():
    update_room_size(10)
    results = []
    num_drones = 1
    last_successful_num_drones = 0
    last_paths = None
    last_pairs = None
    failed_runs = 0
    room_size = 10

    deadline = time.perf_counter() + TIME_LIMIT_SECONDS
    while True:
        pairs = list(generate_circle_crossing(num_drones))
        start_time = time.perf_counter()
        paths = plan_multiple_paths(pairs)
        elapsed = time.perf_counter() - start_time
        
        if time.perf_counter() >= deadline:
            print(f"Failed to find a solution within {TIME_LIMIT_SECONDS} seconds.")
            failed_runs += 1
            break

        if paths is None or any(p is None for p in paths):
            print(f"Failed to find a solution for {num_drones} drones, since the room is too small and the paths are not feasible.")
            print(f"making the room bigger new size: {room_size * 2}")
            room_size *= 2
            update_room_size(room_size)
            failed_runs += 1
            continue

        avg_length = compute_average_path_length(paths)
        results.append({
            "num_drones": num_drones,
            "steps": len(paths),
            "time": elapsed,
            "average_length": avg_length,
            "room_size": room_size
        })

        last_paths = paths              
        last_pairs = pairs
        last_successful_num_drones = num_drones
        num_drones += 1

    with open(RESULTS_FILE, "a") as f:
        f.write("\nHirachical A* results: Circle crossing:\n")
        for result in results:
            f.write(
                f"Drones: {result['num_drones']}, "
                f"Steps: {result['steps']}, "
                f"Time: {result['time']:.3f} s, "
                f"Average length: {result['average_length']:.2f}, "
                f"Room size: {result['room_size']}\n"
            )

        f.write("\n")
        f.write(f"Maximum number of drones: {last_successful_num_drones}\n")
        if time.perf_counter() >= deadline:
            f.write("failed due to time limit\n")
        else:
            f.write("failed due to feasibillity of path\n")
        f.write(f"Failed runs: {failed_runs}\n")
        f.write("\n")

    if PLOT and last_paths is not None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        for j, path in enumerate(last_paths):
            if path is None:
                continue
            x, y, z = zip(*path)
            ax.plot(x, y, z, label=f'Drone {j+1}')

        # plot the circle for reference
        theta = np.linspace(0, 2 * np.pi, 100)
        x_circle = CIRCLE_RADIUS * np.cos(theta)
        y_circle = CIRCLE_RADIUS * np.sin(theta)
        z_circle = np.full_like(theta, 1)  # Circle at z=1
        ax.plot(x_circle, y_circle, z_circle, 'r--', label='Circle')
        
        for i in range(last_successful_num_drones):
            ax.scatter(last_pairs[i][0][0], last_pairs[i][0][1], last_pairs[i][0][2], c='green', marker='o', label=f'Start {i+1}' if i == 0 else "")
            ax.scatter(last_pairs[i][1][0], last_pairs[i][1][1], last_pairs[i][1][2], c='blue', marker='x', label=f'Goal {i+1}' if i == 0 else "")

        ax.set_title(f'Hirachical A* Circle Crossing with {last_successful_num_drones} Drones')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        plt.savefig(f'HirachicalAstarCircleCrossing_{last_successful_num_drones}_drones.png')
        if SHOW_PLOTS:
            plt.show()
        
    update_room_size(1)


# ----------------------------
# Joint A* random missions 4 Drones
# ----------------------------
def Joint_Astar_random_missions():
    num_drones = 4
    num_successful_path = 0
    lenghts = []
    failed_runs = 0

    # Store results for each successful run
    deadline = time.perf_counter() + TIME_LIMIT_SECONDS
    while True:
        starts, goals = generate_random_missions(num_drones)
        joint_start = tuple(starts)
        joint_goal = tuple(goals)

        planner = JointAStar3D()
        joint_path = planner.solve(joint_start, joint_goal, TIME_LIMIT_SECONDS)
        if joint_path is None:
            failed_runs += 1
            continue

        if deadline - time.perf_counter() <= 0:
            print(f"No more solutions can be found within the time limit of {TIME_LIMIT_SECONDS} seconds.")
            break

        avg_length = compute_average_path_length(split_joint_path(joint_path, num_drones))
        lenghts.append(avg_length)
        num_successful_path += 1

    # Save results to file
    with open(RESULTS_FILE, "a") as f:
        f.write("\nJoint A* results: Random missions:\n")

        f.write(
                f"Total successful paths found: {num_successful_path}\n"
                f"Average path length across successful runs: {sum(lenghts) / len(lenghts):.2f}\n"
                f"Failed runs: {failed_runs}\n"
            )

        f.write("\n")



# ----------------------------
# Hirachical A* random missions 4 Drones
# ----------------------------
def Hirachical_Astar_random_missions():
    num_drones = 4
    num_successful_path = 0
    lenghts = []
    failed_runs = 0

    # Store results for each successful run
    deadline = time.perf_counter() + TIME_LIMIT_SECONDS
    while True:
        if deadline - time.perf_counter() <= 0:
            print(f"No more solutions can be found within the time limit of {TIME_LIMIT_SECONDS} seconds.")
            break

        starts, goals = generate_random_missions(num_drones)
        paths = plan_multiple_paths(list(zip(starts, goals)))

        if paths is None or any(p is None for p in paths):
            failed_runs += 1
            continue

        avg_length = compute_average_path_length(paths)
        lenghts.append(avg_length)
        num_successful_path += 1

    # Save results to file
    with open(RESULTS_FILE, "a") as f:
        f.write("\nHirachical A* results: Random missions:\n")
        f.write(
                f"Total successful paths found: {num_successful_path}\n"
                f"Average path length across successful runs: {sum(lenghts) / len(lenghts):.2f}\n"
                f"Failed runs: {failed_runs}\n"
            )
        f.write("\n")

    
# ----------------------------
# Joint A* with increasing number of obstacles
# ----------------------------
def Joint_Astar_obstacle_run(mission):
    num_drones = 4
    num_obstacles = 1
    new_obstacles_generated = True
    results = []
    lenghts = []
    failed_runs = 0
    
    deadline = time.perf_counter() + TIME_LIMIT_SECONDS
    while True:
        if deadline - time.perf_counter() <= 0:
            print(f"No more solutions can be found within the time limit of {TIME_LIMIT_SECONDS} seconds.")
            break

        starts, goals = mission
        joint_start = tuple(starts)
        joint_goal = tuple(goals)
        obstacles, new_obstacles_generated = generate_random_obstacles(num_obstacles)

        planner = JointAStar3D(obstacles)

        start_time = time.perf_counter()
        joint_path = planner.solve(joint_start, joint_goal, TIME_LIMIT_SECONDS)
        elapsed = time.perf_counter() - start_time

        if joint_path is None:
            failed_runs += 1
            continue
        
        results.append((joint_path, obstacles, elapsed))
        
        avg_length = compute_average_path_length(split_joint_path(joint_path, num_drones))
        lenghts.append(avg_length)
        num_obstacles += 1

    # Save results to file
    with open(RESULTS_FILE, "a") as f:
        f.write("\nJoint A* results: Random obstacles:\n")
        f.write(
            f"Max number of obstacles solved within {TIME_LIMIT_SECONDS} seconds: {num_obstacles - 1}\n"
            f"Average path length for all successful runs: {sum(lenghts) / len(lenghts):.2f}\n" 
        )
        
        f.write("Every tenth path plotted in ObstaclesFiguresJoint/ as joint_astar_path_i.png\n")
        if new_obstacles_generated:
            f.write("Stopped due to time limit\n")
        else:
            f.write("Stopped due to space limit, could not generate more unique obstacles\n")
        f.write(f"Failed runs: {failed_runs}\n")
        f.write("\n")

    # Plot every tenth path figs and save as pngs
    if PLOT:
        for i, (joint_path, obstacles, elapsed) in enumerate(results):
            if i % 10 == 0 and joint_path is not None:
                fig = plt.figure()
                ax = fig.add_subplot(111, projection='3d')

                # Plot obstacles
                if obstacles:
                    obs_x, obs_y, obs_z = zip(*obstacles)
                    ax.scatter(obs_x, obs_y, obs_z, c='red', marker='x', label='Obstacles')

                # Plot paths
                drone_paths = split_joint_path(joint_path, num_drones)
                for j, path in enumerate(drone_paths):
                    x, y, z = zip(*path)
                    ax.plot(x, y, z, label=f'Drone {j+1}')

                ax.set_title(f'Joint A* Path with {len(obstacles)} Obstacles')
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.legend()
                plt.savefig(f'ObstaclesFiguresJoint/joint_astar_path_{i}.png')
                if SHOW_PLOTS:
                    plt.show()
                
                plt.close()


# ----------------------------
# Hirachical A* with increasing number of obstacles
# ----------------------------
def Hirachical_Astar_obstacle_run(mission):
    num_obstacles = 1
    new_obstacles_generated = True

    # Store results for each successful run
    results = []
    lenghts = []
    failed_runs = 0
    
    deadline = time.perf_counter() + TIME_LIMIT_SECONDS
    while True:
        if deadline - time.perf_counter() <= 0:
            print(f"No more solutions can be found within the time limit of {TIME_LIMIT_SECONDS} seconds.")
            break

        starts, goals = mission
        obstacles, new_obstacles_generated = generate_random_obstacles(num_obstacles)
        # Make copy of the obstacles set to avoid modifying the original one in the planner
        obs_copy = set(obstacles) if obstacles else set()
        obstacles = obstacles if obstacles else set()

        start_time = time.perf_counter()
        path = plan_multiple_paths(list(zip(starts, goals)), obstacles)
        elapsed = time.perf_counter() - start_time

        if path is None or any(p is None for p in path):
            failed_runs += 1
            continue
        
        results.append((path, obs_copy, elapsed))
        
        avg_length = compute_average_path_length(path)
        lenghts.append(avg_length)

        num_obstacles += 1

    # Save results to file
    with open(RESULTS_FILE, "a") as f:
        f.write("\nHirachical A* results: Random obstacles:\n")
        f.write(
            f"Max number of obstacles solved within {TIME_LIMIT_SECONDS} seconds: {num_obstacles - 1}\n"
            f"Average path length for all successful runs: {sum(lenghts) / len(lenghts):.2f}\n" 
        )
        
        f.write("Every tenth path plotted in ObstaclesFiguresHirachical/ as Hirachical_astar_path_i.png\n")
        if new_obstacles_generated:
            f.write("Stopped due to time limit\n")
        else:
            f.write("Stopped due to space limit, could not generate more unique obstacles\n")
        f.write(f"Failed runs: {failed_runs}\n")
        f.write("\n")

    # plot every tenth path figs and save as pngs
    if PLOT:
        for i, (paths, obstacles, elapsed) in enumerate(results):
            if i % 10 == 0 and paths is not None:
                fig = plt.figure()
                ax = fig.add_subplot(111, projection='3d')

                # Plot obstacles
                if obstacles:
                    obs_x, obs_y, obs_z = zip(*obstacles)
                    ax.scatter(obs_x, obs_y, obs_z, c='red', marker='x', label='Obstacles')

                # Plot paths
                for j, path in enumerate(paths):
                    if path is None:
                        continue
                    x, y, z = zip(*path)
                    ax.plot(x, y, z, label=f'Drone {j+1}')

                ax.set_title(f'Hirachical A* Path with {len(obstacles)} Obstacles')
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.legend()
                plt.savefig(f'ObstaclesFiguresHirachical/Hirachical_astar_path_{i}.png')
                if SHOW_PLOTS:
                    plt.show()
                
                plt.close()



# ----------------------------
# Call all the functions and save results
# ----------------------------
if __name__ == "__main__":
    with open(RESULTS_FILE, "w") as f:
        f.write(f"A* Benchmark Results with time limit: {TIME_LIMIT_SECONDS} seconds\n")
        f.write("================================================\n\n")

    print(f"Results file {RESULTS_FILE} initialized.")
    print("Starting A* benchmarks...\n")
    Astar_Joint_missions([mission_OG, mission_Cross])
    print("\n\n")
    print("Starting Hirachical A* benchmarks...\n")
    Astar_hirachical_missions([zip(mission_OG[0],mission_OG[1]), zip(mission_Cross[0],mission_Cross[1])])
    print("\n\n")
    print("Starting Circle Crossing benchmarks...\n")
    joint_Astar_circle_crossing()
    print("\n\n")
    print("Starting Hirachical Circle Crossing benchmarks...\n")
    hirachical_Astar_circle_crossing()
    print("\n\n")
    print("Starting Joint A* random missions benchmarks...\n")
    Joint_Astar_random_missions()
    print("\n\n")
    print("Starting Hirachical A* random missions benchmarks...\n")
    Hirachical_Astar_random_missions()
    print("\n\n")
    print("Starting Joint A* obstacle run benchmarks...\n")
    Joint_Astar_obstacle_run(obstacle_run_mission)
    print("\n\n")
    print("Starting Hirachical A* obstacle run benchmarks...\n")
    Hirachical_Astar_obstacle_run(obstacle_run_mission)

    print(f"Results saved to {RESULTS_FILE}")
    

