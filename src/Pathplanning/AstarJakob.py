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
TIME_LIMIT_SECONDS = 15
RESULTS_FILE = "OMPL_RRTStarResults.txt"

MIN_SEPARATION = 0.3

X_MIN, X_MAX = -1.8, 1.8
Y_MIN, Y_MAX = -1.8, 1.8
Z_MIN, Z_MAX = 0.0, 3.0


# ----------------------------
# Mission generators
# ----------------------------
def generate_circle_crossing(num_drones, radius=1.5, z=0.7):
    starts = []
    goals = []

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


# ----------------------------
# Helpers
# ----------------------------
def positions_are_valid(positions):
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):
            if math.dist(positions[i], positions[j]) < MIN_SEPARATION:
                return False

    return True


def compute_average_path_length(drone_paths):
    total = 0.0

    for path in drone_paths:
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
        if ss.solve(0.01) and ss.haveExactSolutionPath():
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

    # Keep exact start, but DO NOT force the final goal.
    # If OMPL found an exact solution, the path should already end at the goal.
    joint_path[0] = list(start_positions)

    if not validate_joint_path(joint_path, steps_per_segment=50):
        print("Returned path is invalid")
        return None, elapsed

    drone_paths = list(map(list, zip(*joint_path)))

    return drone_paths, elapsed


# ----------------------------
# Tests
# ----------------------------
def test_circle_scaling(results):
    num_drones = 1
    last_paths = None

    deadline = time.perf_counter() + TIME_LIMIT_SECONDS
    while True:
        if time.perf_counter() > deadline:
            print("Time limit reached, stopping circle scaling test.")
            break
        if num_drones > 31:
            print("Reached 31 drones, stopping circle scaling test.")
            break

        print(f"\nTrying {num_drones} drones circle...")

        starts, goals = generate_circle_crossing(num_drones)

        drone_paths, elapsed = solve_ompl_rrtstar(starts, goals)

        if drone_paths is None:
            print(f"Failed at {num_drones} drones")
            break

        avg_length = compute_average_path_length(drone_paths)

        print(f"Success | Time: {elapsed:.3f} | Avg length: {avg_length:.2f}")

        results.append(
            f"test: circle, drones: {num_drones}, time: {elapsed}, avg_length: {avg_length}"
        )

        last_paths = drone_paths
        num_drones += 1

    if last_paths is not None:
        plot_paths(last_paths, "ompl_circle_paths.json")


def test_fixed_missions(results):
    for name, mission_func in [
        ("OG", fixed_og_mission),
        ("Cross", fixed_cross_mission),
    ]:
        print(f"\nRunning mission: {name}")

        starts, goals = mission_func()

        drone_paths, elapsed = solve_ompl_rrtstar(starts, goals)

        if drone_paths is None:
            print("Failed")
            continue

        avg_length = compute_average_path_length(drone_paths)

        plot_paths(drone_paths, f"ompl_{name.lower()}_paths.json")

        print(f"Success | Time: {elapsed:.3f} | Avg length: {avg_length:.2f}")

        results.append(
            f"test: {name}, drones: {len(starts)}, time: {elapsed}, avg_length: {avg_length}"
        )


def test_random_missions(results):
    print("\nRunning random missions...")

    deadline = time.perf_counter() + TIME_LIMIT_SECONDS
    successes = 0
    lengths = []
    first_successful_paths = None

    while time.perf_counter() < deadline:
        starts, goals = generate_random_missions(4)

        drone_paths, elapsed = solve_ompl_rrtstar(starts, goals)

        if drone_paths is None:
            continue

        avg_length = compute_average_path_length(drone_paths)

        lengths.append(avg_length)
        successes += 1

        if first_successful_paths is None:
            first_successful_paths = drone_paths

    print(f"Successes: {successes}")

    if lengths:
        results.append({
            "test": "random",
            "runs": successes,
            "avg_length": sum(lengths) / len(lengths)
        })

    if first_successful_paths is not None:
        plot_paths(first_successful_paths, "ompl_random_paths.json")


# ----------------------------
# Main
# ----------------------------
if __name__ == "__main__":
    random.seed(42)
    np.random.seed(42)

    results = []

    test_fixed_missions(results)
    test_circle_scaling(results)
    test_random_missions(results)

    with open(RESULTS_FILE, "w") as f:
        f.write(
            f"OMPL RRT* Joint Benchmark Results with time limit: "
            f"{TIME_LIMIT_SECONDS} seconds\n"
        )
        f.write("============================================================\n\n")

        for r in results:
            f.write(str(r) + "\n")

    print(f"\nResults saved to {RESULTS_FILE}")