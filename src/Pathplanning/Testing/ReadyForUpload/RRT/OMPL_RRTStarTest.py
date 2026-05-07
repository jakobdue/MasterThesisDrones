from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou

ou.setLogLevel(ou.LOG_NONE)

import time
import math
import random
import json
import numpy as np

# ----------------------------
# Settings
# ----------------------------
TIME_LIMIT_SECONDS = 60
RESULTS_FILE = "OMPL_RRTStarResults.txt"
CIRCLE_RADIUS = 3 

MIN_SEPARATION = 0.3 

X_MIN, X_MAX = -1.8, 1.8
Y_MIN, Y_MAX = -1.8, 1.8
Z_MIN, Z_MAX = 0.0, 3.0
obstacles = set()
local_obstacles = set()


# ----------------------------
# Mission generators
# ----------------------------
def generate_circle_crossing(num_drones, radius=CIRCLE_RADIUS, z=0.7):
    starts = []
    goals = []
    radius = radius/5
    for i in range(num_drones):
        theta = 2 * np.pi * i / num_drones

        start = (
            radius * np.cos(theta),
            radius * np.sin(theta),
            z
        )

        goal = (
            -start[0],
            -start[1],
            z
        )

        starts.append(start)
        goals.append(goal)

    return starts, goals

def generate_circle_crossing_hierarchical(num_drones, radius=CIRCLE_RADIUS, z=1.0):
    radius = radius/5
    if num_drones <= 0:
        return [], []

    starts = []
    goals = []

    # We distribute drones over HALF the circle
    # so we never reuse opposite pairs
    for i in range(num_drones):
        theta = math.pi * i / num_drones 

        start = (
            radius * math.cos(theta),
            radius * math.sin(theta),
            float(z),
        )

        goal = (
            -start[0],
            -start[1],
            float(z),
        )

        starts.append(start)
        goals.append(goal)

    return starts, goals

def generate_random_missions(num_drones=4):
    while True:
        starts = []
        goals = []

        for _ in range(num_drones):
            start = (
                random.uniform(X_MIN, X_MAX),
                random.uniform(Y_MIN, Y_MAX),
                random.uniform(Z_MIN, Z_MAX),
            )

            goal = (
                random.uniform(X_MIN, X_MAX),
                random.uniform(Y_MIN, Y_MAX),
                random.uniform(Z_MIN, Z_MAX),
            )

            starts.append(start)
            goals.append(goal)

        if positions_are_valid(starts) and positions_are_valid(goals):
            return starts, goals

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
        obs_raw = (
            random.randint(X_MIN, X_MAX),
            random.randint(Y_MIN, Y_MAX),
            random.randint(Z_MIN, Z_MAX)
        )

        obs = (obs_raw[0]/5, obs_raw[1]/5, obs_raw[2]/5)

        # Reject if too close to start or goal
        too_close = False
        for p in start + goal:
            if math.dist(obs, p) < MIN_SEPARATION:
                too_close = True
                break

        if too_close:
            continue

        obstacles.add(obs)
        if time.perf_counter() >= deadline:
            print(f"Time limit of {TIME_LIMIT_SECONDS} seconds reached while generating obstacles.")
            new_obstacles_generated = False
            break
    return new_obstacles_generated

def update_local_obstacles(obstacles):
    global local_obstacles
    local_obstacles = set(obstacles)

def fixed_og_mission():
    starts = [
        (-1.0, -1.0, 0.7),
        (-1.0,  1.0, 0.7),
        (0.0,  -1.0, 0.7),
        (0.6,  -1.0, 0.7),
    ]

    goals = [
        (1.2, 1.0, 0.7),
        (1.0, -1.0, 0.7),
        (-0.2, 1.0, 0.7),
        (0.6, 1.0, 0.7),
    ]

    return starts, goals



def fixed_cross_mission():
    starts = [
        (-0.2, -0.2, 0.7),
        (-0.2,  0.2, 0.7),
        (0.2,   0.2, 0.7),
        (0.2,  -0.2, 0.7),
    ]

    goals = [
        (1.0, 1.0, 0.7),
        (1.0, -1.0, 0.7),
        (-1.0, -1.0, 0.7),
        (-1.0, 1.0, 0.7),
    ]

    return starts, goals

mission_start_obstacle_run = [(-5, -5, 4), (-5, 5, 4), (5, 0, 2),  (5,  0, 7)]
mission_goal_obstacle_run  = [(5,   5, 4), (5, -5, 4), (-5, 0, 7), (-5, 0, 2)]

def obstacle_run_mission():
    # scale by dividing by 5 
    starts = [(s[0]/5, s[1]/5, s[2]/5) for s in mission_start_obstacle_run]
    goals = [(g[0]/5, g[1]/5, g[2]/5) for g in mission_goal_obstacle_run]
    return starts, goals


# ----------------------------
# Helpers
# ----------------------------
def positions_are_valid(positions):
    # Drone drone separation
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):
            if math.dist(positions[i], positions[j]) < MIN_SEPARATION:
                return False
            
    # Drone obstacle separation
    for pos in positions:
        for obs in obstacles.union(local_obstacles):
            if math.dist(pos, obs) < MIN_SEPARATION:
                return False

    return True


def compute_average_path_length(drone_paths):
    total = 0.0
    for path in drone_paths:
        if path is None:
            continue
        length = sum(
            math.dist(path[i], path[i - 1])
            for i in range(1, len(path))
        )
        total += length

    return total / len(drone_paths)


def plot_paths(drone_paths, filename):
    json_filename = filename.rsplit(".", 1)[0] + ".json"

    data = {
        "num_drones": len(drone_paths),
        "paths": drone_paths
    }

    with open(json_filename, "w") as f:
        json.dump(data, f, indent=4)

    print(f"Path data saved to {json_filename}")


def validate_joint_path(joint_path, steps_per_segment=50):
    for k in range(len(joint_path) - 1):
        a = joint_path[k]
        b = joint_path[k + 1]

        for t in np.linspace(0, 1, steps_per_segment):
            interp = []

            for i in range(len(a)):
                interp.append((
                    a[i][0] + t * (b[i][0] - a[i][0]),
                    a[i][1] + t * (b[i][1] - a[i][1]),
                    a[i][2] + t * (b[i][2] - a[i][2]),
                ))

            if not positions_are_valid(interp):
                return False

    return True

def update_room_size(room_scaler):
    global X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX
    X_MIN, X_MAX = -1.8 * room_scaler, 1.8 * room_scaler
    Y_MIN, Y_MAX = -1.8 * room_scaler, 1.8 * room_scaler
    Z_MIN, Z_MAX = 0.0, 3.0 * room_scaler


# ----------------------------
# OMPL space
# ----------------------------
def create_drone_space():
    drone_space = ob.RealVectorStateSpace(3)

    bounds = ob.RealVectorBounds(3)

    bounds.setLow(0, X_MIN)
    bounds.setLow(1, Y_MIN)
    bounds.setLow(2, Z_MIN)

    bounds.setHigh(0, X_MAX)
    bounds.setHigh(1, Y_MAX)
    bounds.setHigh(2, Z_MAX)

    drone_space.setBounds(bounds)

    return drone_space


# ----------------------------
# Validity checker
# ----------------------------
class ValidityChecker(ob.StateValidityChecker):
    def __init__(self, si, num_drones):
        super().__init__(si)
        self.num_drones = num_drones

    def isValid(self, state):
        positions = []

        for i in range(self.num_drones):
            positions.append((
                state[i][0],
                state[i][1],
                state[i][2],
            ))

        return positions_are_valid(positions)


# ----------------------------
# Motion validator
# ----------------------------
class MotionValidator(ob.MotionValidator):
    def __init__(self, si, num_drones):
        super().__init__(si)
        self.num_drones = num_drones

    def checkMotion(self, s1, s2):
        steps = 50

        for t in np.linspace(0, 1, steps):
            interp = []

            for i in range(self.num_drones):
                interp.append((
                    s1[i][0] + t * (s2[i][0] - s1[i][0]),
                    s1[i][1] + t * (s2[i][1] - s1[i][1]),
                    s1[i][2] + t * (s2[i][2] - s1[i][2]),
                ))

            if not positions_are_valid(interp):
                return False

        return True

    def checkMotionWithLastValid(self, s1, s2, lastValid):
        valid = self.checkMotion(s1, s2)

        if valid:
            lastValid.second = 1.0
        else:
            lastValid.second = 0.0

        return valid


# ----------------------------
# OMPL solver
# ----------------------------
def solve_ompl_rrtstar(start_positions, goal_positions):
    num_drones = len(start_positions)

    if not positions_are_valid(start_positions):
        print("Invalid start positions")
        return None, 0

    if not positions_are_valid(goal_positions):
        print("Invalid goal positions")
        return None, 0

    space = ob.CompoundStateSpace()

    for _ in range(num_drones):
        space.addSubspace(create_drone_space(), 1.0)

    ss = og.SimpleSetup(space)
    si = ss.getSpaceInformation()

    checker = ValidityChecker(si, num_drones)
    si.setStateValidityChecker(checker)

    motion_validator = MotionValidator(si, num_drones)
    si.setMotionValidator(motion_validator)

    si.setup()

    start = space.allocState()
    goal = space.allocState()

    for i in range(num_drones):
        start[i][0], start[i][1], start[i][2] = start_positions[i]
        goal[i][0], goal[i][1], goal[i][2] = goal_positions[i]

    ss.setStartAndGoalStates(start, goal)

    planner = og.RRTstar(si)
    planner.setRange(0.2)
    planner.setGoalBias(0.05)

    ss.setPlanner(planner)

    start_time = time.perf_counter()

    status = False

    while time.perf_counter() - start_time < TIME_LIMIT_SECONDS:
        if ss.solve(0.1) and ss.haveExactSolutionPath():
            status = True
            break

    elapsed = time.perf_counter() - start_time

    if not status or not ss.haveExactSolutionPath():
        print("No exact solution found")
        return None, elapsed

    path = ss.getSolutionPath()
    path.interpolate(100)

    states = path.getStates()

    joint_path = []

    for state in states:
        joint_state = []

        for i in range(num_drones):
            joint_state.append((
                state[i][0],
                state[i][1],
                state[i][2],
            ))

        joint_path.append(joint_state)

    joint_path[0] = list(start_positions)

    if not validate_joint_path(joint_path, steps_per_segment=50):
        print("Returned path is invalid")
        return None, elapsed

    drone_paths = list(map(list, zip(*joint_path)))

    return drone_paths, elapsed

# ----------------------------
# Hirachical solver
# ----------------------------

def solve_single_drone_rrtstar(start, goal):
    if not positions_are_valid([start]):
        return None, 0
    if not positions_are_valid([goal]):
        return None, 0

    space = create_drone_space()

    ss = og.SimpleSetup(space)
    si = ss.getSpaceInformation()

    # Validity: only 1 drone
    class SingleValidity(ob.StateValidityChecker):
        def __init__(self, si):
            super().__init__(si)

        def isValid(self, state):
            pos = (state[0], state[1], state[2])
            return positions_are_valid([pos])

    si.setStateValidityChecker(SingleValidity(si))

    si.setup()

    start_state = space.allocState()
    goal_state  = space.allocState()

    start_state[0], start_state[1], start_state[2] = start
    goal_state[0], goal_state[1], goal_state[2] = goal

    ss.setStartAndGoalStates(start_state, goal_state)

    planner = og.RRTstar(si)
    planner.setRange(0.2)
    planner.setGoalBias(0.05)

    ss.setPlanner(planner)

    start_time = time.perf_counter()

    while time.perf_counter() - start_time < TIME_LIMIT_SECONDS:
        if ss.solve(0.01) and ss.haveExactSolutionPath():
            break

    elapsed = time.perf_counter() - start_time

    if not ss.haveExactSolutionPath():
        return None, elapsed

    path = ss.getSolutionPath()
    path.interpolate(100)

    states = path.getStates()

    single_path = [
        (s[0], s[1], s[2]) for s in states
    ]

    return single_path, elapsed

def convert_path_to_obstacles(path, resolution=0.25):
    obs = set()

    for i in range(len(path) - 1):
        a = path[i]
        b = path[i+1]

        dist = math.dist(a, b)
        steps = max(2, int(dist / resolution))

        for t in np.linspace(0, 1, steps):
            interp = (
                a[0] + t*(b[0]-a[0]),
                a[1] + t*(b[1]-a[1]),
                a[2] + t*(b[2]-a[2]),
            )
            obs.add(interp)

    return obs


def plan_multiple_paths_ompl(starts, goals):
    global obstacles

    obstacles = set()  # reset global obstacles

    all_paths = []
    total_time = 0.0

    for i in range(len(starts)):
        start = starts[i]
        goal = goals[i]
        # Add all the other drones start and goal as local obstacles
        local_obs = set()
        for j in range(len(starts)):
            if j != i:
                local_obs.add(starts[j])
                local_obs.add(goals[j])
        update_local_obstacles(local_obs)

        path, elapsed = solve_single_drone_rrtstar(start, goal)
        total_time += elapsed

        if path is None:
            print(f"No path found for drone {i}")
            return None, total_time

        all_paths.append(path)

        # Add path as obstacles
        path_obs = convert_path_to_obstacles(path)
        obstacles.update(path_obs)

    return all_paths, total_time



# ----------------------------
# Tests
# ----------------------------
def test_fixed_missions(results, hirachical):
    update_random_obstacles(0, [], [])
    for name, mission_func in [
        ("OG", fixed_og_mission),
        ("Cross", fixed_cross_mission),
        ]:
        print(f"\nRunning mission: {name}")

        starts, goals = mission_func()

        if hirachical:
            drone_paths, elapsed = plan_multiple_paths_ompl(starts, goals)
        else: 
            drone_paths, elapsed = solve_ompl_rrtstar(starts, goals)

        if drone_paths is None:
            print("Failed")
            continue

        avg_length = compute_average_path_length(drone_paths)

        plot_paths(drone_paths, f"ompl_{name.lower()}_paths.json")

        print(f"Success | Time: {elapsed:.3f} | Avg length: {avg_length:.2f}")

        if hirachical:
            results.append(
                f"test: {name} hirachical, drones: {len(starts)}, time: {elapsed}, avg_length: {avg_length}"
            )
        else: 
            results.append(
                f"test: {name}, Joint, drones: {len(starts)}, time: {elapsed}, avg_length: {avg_length}"
            )
    results.append("\n")



def test_circle_scaling(results, hirachical):
    update_room_size(10.0)  # scale up the room for this test
    update_random_obstacles(0, [], [])
    num_drones = 1
    last_paths = None

    deadline = time.perf_counter() + TIME_LIMIT_SECONDS
    while True:
        if time.perf_counter() > deadline:
            print("Time limit reached, stopping circle scaling test.")
            results.append("Stopped due to time limit\n")
            break

        print(f"\nTrying {num_drones} drones circle...")
        

        if hirachical:
            starts, goals = generate_circle_crossing_hierarchical(num_drones, radius = 30.0)  # use larger radius for hierarchical to fit more drones
            drone_paths, elapsed = plan_multiple_paths_ompl(starts, goals)
            
        else: 
            starts, goals = generate_circle_crossing(num_drones, radius = 30.0)
            drone_paths, elapsed = solve_ompl_rrtstar(starts, goals)

        if drone_paths is None:
            print(f"Failed at {num_drones} drones")
            break

        avg_length = compute_average_path_length(drone_paths)

        print(f"Success | Time: {elapsed:.3f} | Avg length: {avg_length:.2f}")

        if hirachical:
            results.append(
                f"test: circle hirachical, drones: {num_drones}, time: {elapsed}, avg_length: {avg_length}"
            )
        else: 
            results.append(
                f"test: circle Joint, drones: {num_drones}, time: {elapsed}, avg_length: {avg_length}"
            )

        last_paths = drone_paths
        num_drones += 1
    results.append("\n")
    if last_paths is not None:
        plot_paths(last_paths, "ompl_circle_paths.json")
    update_room_size(1.0)


def test_random_missions(results, hirachical):
    update_random_obstacles(0, [], [])
    print("\nRunning random missions...")

    deadline = time.perf_counter() + TIME_LIMIT_SECONDS
    successes = 0
    fails = 0
    lengths = []
    first_successful_paths = None

    while time.perf_counter() < deadline:
        starts, goals = generate_random_missions(4)

        if hirachical:
            drone_paths, _ = plan_multiple_paths_ompl(starts, goals)
        else: 
            drone_paths, _ = solve_ompl_rrtstar(starts, goals)

        if drone_paths is None:
            fails += 1
            continue

        avg_length = compute_average_path_length(drone_paths)

        lengths.append(avg_length)
        successes += 1

        if first_successful_paths is None:
            first_successful_paths = drone_paths

    print(f"Successes: {successes}")
    print(f"Fails: {fails}")

    if lengths:
        avg_lenght = sum(lengths) / len(lengths)
    else: 
        avg_lenght = 0.0

    if hirachical:
        results.append(f"test: random missions, \nHirachical, runs: {successes}, avg_length: {avg_lenght}, fails: {fails}\n\n")
    else: 
        results.append(f"test: random missions, \nJoint runs: {successes}, avg_length: {avg_lenght}, fails: {fails}\n\n")

    if first_successful_paths is not None:
        plot_paths(first_successful_paths, "ompl_random_paths.json")



def test_obstacles_scaling(results, mission, hirachical):
    update_random_obstacles(0, [], [])
    successes = 0
    fails = 0
    num_obstacles = 1
    new_obstacles_generated = True
    local_results = []
    lenghts = []
    
    deadline = time.perf_counter() + TIME_LIMIT_SECONDS
    while True:
        starts, goals = mission()
        new_obstacles_generated = update_random_obstacles(num_obstacles, starts, goals)
        global obstacles
        local_obs = obstacles.copy()

        if hirachical:
            drone_paths, elapsed = plan_multiple_paths_ompl(starts, goals)
        else: 
            drone_paths, elapsed = solve_ompl_rrtstar(starts, goals)

        if drone_paths is None:
            print(f"Failed with {num_obstacles} obstacles")
            fails += 1
            continue
        
        
        local_results.append((drone_paths, local_obs, elapsed))
        
        
        if deadline - time.perf_counter() <= 0:
            print(f"No more solutions can be found within the time limit of {TIME_LIMIT_SECONDS} seconds.")
            break

        successes += 1

        avg_length = compute_average_path_length(drone_paths)        
        lenghts.append(avg_length)
        num_obstacles += 1

    # Save results to file
    if hirachical:
        results.append(f"Test: random obstacles, hirachical")
    else:
        results.append(f"Test: random obstacles, Joint")
    
    avg_path_length = sum(lenghts) / len(lenghts) if lenghts else 0.0
    results.append(
        f"Max number of obstacles solved within {TIME_LIMIT_SECONDS} seconds: {num_obstacles - 1}"
        f"\nTotal successes: {successes}, fails: {fails}"
        f"\nAverage path length for all successful runs: {avg_path_length:.2f}\n" 
    )
    
    if new_obstacles_generated:
        results.append("Stopped due to time limit\n")
    else:
        results.append("Stopped due to space limit, could not generate more unique obstacles\n")
    results.append("\n")
        
    # Plot of every solution with obstacles
    for i, (paths, obstacles, elapsed) in enumerate(local_results):
        if paths is None:
            continue

        data = {
            "num_drones": len(paths),
            "paths": paths,
            "obstacles": list(obstacles),
            "elapsed_time": elapsed
        }

        filename = f"OMPLOBSTACLE/ompl_obstacle_run_{i}.json"

        with open(filename, "w") as f:
            json.dump(data, f, indent=4)

        print(f"Path and obstacle data saved to {filename}")



# ----------------------------
# Main
# ----------------------------
if __name__ == "__main__":
    random.seed(42)
    np.random.seed(42)

    results = []
    test_fixed_missions(results, hirachical=False)
    test_fixed_missions(results, hirachical=True)
    test_circle_scaling(results, hirachical=False)
    test_circle_scaling(results, hirachical=True)
    test_random_missions(results, hirachical=False)
    test_random_missions(results, hirachical=True)
    test_obstacles_scaling(results, obstacle_run_mission, hirachical=False)
    test_obstacles_scaling(results, obstacle_run_mission, hirachical=True)
   

    with open(RESULTS_FILE, "w") as f:
        f.write(
            f"OMPL RRT* Joint Benchmark Results with time limit: "
            f"{TIME_LIMIT_SECONDS} seconds\n"
        )
        f.write("============================================================\n\n")

        for r in results:
            f.write(str(r) + "\n")

    print(f"\nResults saved to {RESULTS_FILE}") 