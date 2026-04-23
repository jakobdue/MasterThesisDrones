import heapq
import math
import time
from itertools import product
from typing import Tuple, Set, Optional

import matplotlib.pyplot as plt
import numpy as np
import random


# ----------------------------
# 1. Settings
# ----------------------------
TIME_LIMIT_SECONDS = 15
RESULTS_FILE = "AstarTestRandomMission.txt"

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

random.seed(42)  # For reproducibility


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


def generate_random_missions(num_drones=4):
    while True:
        starts, goals = [], []
        for i in range(num_drones):
            start = (random.randint(X_MIN + 1, X_MAX - 1), random.randint(Y_MIN + 1, Y_MAX - 1), random.randint(Z_MIN + 1, Z_MAX - 1))
            goal = (random.randint(X_MIN + 1, X_MAX - 1), random.randint(Y_MIN + 1, Y_MAX - 1), random.randint(Z_MIN + 1, Z_MAX - 1))
            starts.append(start)
            goals.append(goal)
        # iAdd the two lists and check for dublicates, if there are any, generate new ones
        if len(set(starts + goals)) == 2 * num_drones:
            break
    return starts, goals


def compute_average_path_length(joint_path, num_drones):
    drone_paths = split_joint_path(joint_path, num_drones)

    total = 0
    for path in drone_paths:
        length = sum(
            math.dist(path[j], path[j - 1])
            for j in range(1, len(path))
        )
        total += length

    return total / num_drones


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
# 5. Benchmark loop
# ----------------------------
if __name__ == "__main__":
    num_drones = 4
    num_successful_path = 0
    lenghts = []

    # Store results for each successful run
    deadline = time.perf_counter() + TIME_LIMIT_SECONDS
    while True:
        starts, goals = generate_random_missions(num_drones)
        joint_start = tuple(starts)
        joint_goal = tuple(goals)

        planner = JointAStar3D()
        joint_path = planner.solve(joint_start, joint_goal, TIME_LIMIT_SECONDS)

        if deadline - time.perf_counter() <= 0:
            print(f"No more solutions can be found within the time limit of {TIME_LIMIT_SECONDS} seconds.")
            break

        avg_length = compute_average_path_length(joint_path, num_drones)
        lenghts.append(avg_length)
        num_successful_path += 1

    # Save results to file
    with open(RESULTS_FILE, "w") as f:
        f.write("A* Benchmark Results\n")
        f.write("====================\n\n")

        f.write(
                f"Total successful paths found: {num_successful_path}\n"
                f"Average path length across successful runs: {sum(lenghts) / len(lenghts):.2f}\n"
            )

        f.write("\n")

    print(f"Results saved to {RESULTS_FILE}")

