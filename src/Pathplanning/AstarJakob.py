import heapq
import math
from itertools import product
from typing import Tuple, List, Set

import matplotlib.pyplot as plt
import numpy as np


# ----------------------------
# 1. Settings
# ----------------------------
NUM_DRONES = 10
MIN_SEPARATION = 0.3

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


# ----------------------------
# 2. Helpers
# ----------------------------
def in_bounds(p):
    x, y, z = p
    return X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX and Z_MIN <= z <= Z_MAX


def dist_sq(a, b):
    return (a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2


def split_joint_path(joint_path, num_drones): # this function is just for plotting, it splits the joint path into individual drone paths
    drone_paths = [[] for _ in range(num_drones)]
    for state in joint_path:
        for i in range(num_drones):
            drone_paths[i].append(state[i])
    return drone_paths


def generate_circle_crossing(num_drones=4, radius=3, z=1): # generates start and goal positions for drones in a circle crossing pattern
    starts, goals = [], []
    for i in range(num_drones):
        theta = 2 * np.pi * i / num_drones
        x = int(round(radius * np.cos(theta)))
        y = int(round(radius * np.sin(theta)))
        start = (x, y, z)
        goal = (-x, -y, z)
        starts.append(start)
        goals.append(goal)
    return starts, goals


# ----------------------------
# 3. Joint A*
# ----------------------------
class JointAStar3D:
    def __init__(self, obstacles: Set[Tuple[int, int, int]] = None):
        self.obstacles = obstacles if obstacles else set()
        self.min_sep_sq = MIN_SEPARATION ** 2

    def is_valid(self, state: JointState):
        for p in state:
            if not in_bounds(p) or p in self.obstacles:
                return False

        for i in range(len(state)):
            for j in range(i + 1, len(state)):
                if dist_sq(state[i], state[j]) < self.min_sep_sq:
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
                (state[i][0] + joint_action[i][0],
                 state[i][1] + joint_action[i][1],
                 state[i][2] + joint_action[i][2])
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

    def solve(self, start, goal):
        open_heap = []
        heapq.heappush(open_heap, (0, start))

        came_from = {}
        g_score = {start: 0}
        closed = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)

            if current in closed:
                continue
            closed.add(current)

            if current == goal:
                return self.reconstruct(came_from, current)

            for neighbor in self.neighbors(current):
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
# 4. Plot
# ----------------------------
def plot_paths(drone_paths):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for i, path in enumerate(drone_paths):
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        zs = [p[2] for p in path]

        ax.plot(xs, ys, zs, label=f"Drone {i+1}")
        ax.scatter(xs[0], ys[0], zs[0], marker='o')
        ax.scatter(xs[-1], ys[-1], zs[-1], marker='x')

    ax.set_xlim(X_MIN, X_MAX)
    ax.set_ylim(Y_MIN, Y_MAX)
    ax.set_zlim(Z_MIN, Z_MAX)

    ax.set_title("Joint A*")
    ax.legend()
    plt.show()


# ----------------------------
# 5. Run
# ----------------------------
if __name__ == "__main__":
    starts, goals = generate_circle_crossing(NUM_DRONES)

    joint_start = tuple(starts)
    joint_goal = tuple(goals)

    planner = JointAStar3D()
    joint_path = planner.solve(joint_start, joint_goal)

    if joint_path is None:
        print("No solution found")
    else:
        print(f"Found solution with {len(joint_path)} steps")

        drone_paths = split_joint_path(joint_path, NUM_DRONES)

        total = 0
        for i, path in enumerate(drone_paths):
            length = sum(
                math.dist(path[j], path[j-1])
                for j in range(1, len(path))
            )
            total += length
            print(f"Drone {i+1} length: {length:.2f}")

        print(f"Average length: {total/NUM_DRONES:.2f}")

        plot_paths(drone_paths)