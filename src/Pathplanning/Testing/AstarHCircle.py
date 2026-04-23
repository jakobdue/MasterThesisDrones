import time

from functools import partial
import random
import math
import heapq
from typing import List, Tuple, Set
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import statistics
import time
import numpy as np
"""
A-star implementeted with LLM chatGPT
"""

RESULTS_FILE = "AstarHCircle.txt"


def generate_circle_crossing(num_drones, radius=3, z=1):
    points = []

    for i in range(num_drones):
        theta = 2 * np.pi * i / num_drones
        x = int(round(radius * np.cos(theta)))
        y = int(round(radius * np.sin(theta)))
        points.append((x, y, z))

    starts = points
    goals = points[1:] + [points[0]]  # shift by 1 → cycle

    return list(zip(starts, goals))


# 3D Grid Limits
X_MIN, X_MAX = -5, 5
Y_MIN, Y_MAX = -5, 5
Z_MIN, Z_MAX = 0, 9

TIME_LIMIT_SECONDS = 10

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

def inflate(point, radius=1):
    x,y,z = point
    inflated = []
    for dx in range(-radius, radius+1):
        for dy in range(-radius, radius+1):
            for dz in range(-radius, radius+1):
                inflated.append((x+dx, y+dy, z+dz))
    return inflated

def plan_multiple_paths(pairs: List[Tuple[Tuple[int, int, int],
                                           Tuple[int, int, int]]]):

    obstacles = set()
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
            print(f"No path found for {start} -> {goal}")
            all_paths.append(None)
        else:
            all_paths.append(path)
            obstacles.update(path)  # Path becomes obstacle
        
    return all_paths


def plot_paths(paths):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    colors = ['blue', 'green', 'orange', 'purple', 'cyan', 'red']

    for i, path in enumerate(paths):
        if path is None:
            continue

        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        zs = [p[2] for p in path]

        ax.plot(xs, ys, zs, color=colors[i % len(colors)], linewidth=2)
        ax.scatter(xs[0], ys[0], zs[0], marker='o')  # start
        ax.scatter(xs[-1], ys[-1], zs[-1], marker='x')  # goal

    ax.set_xlim(X_MIN, X_MAX)
    ax.set_ylim(Y_MIN, Y_MAX)
    ax.set_zlim(Z_MIN, Z_MAX)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")


    # calculate average path length
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
    if count > 0:
        print(f"Average path length: {total_length / count:.2f}")

    plt.show()

    # save to pdf
    fig.savefig("paths.pdf")


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

if __name__ == "__main__":
    results = []
    num_drones = 1

    last_successful_num_drones = 0
    last_successful_path = None

    while True:
        pairs = list(generate_circle_crossing(num_drones))  # FIX
        start_time = time.perf_counter()
        paths = plan_multiple_paths(pairs)
        elapsed = time.perf_counter() - start_time

        # FIX: detect failure properly
        if paths is None or any(p is None for p in paths):
            print(f"Failed to find a solution within {TIME_LIMIT_SECONDS} seconds.")
            break

        avg_length = compute_average_path_length(paths)

        print(f"Success with {num_drones} drone(s) in {elapsed:.3f} seconds.")
        print(f"Found solution with {len(paths)} steps")
        if avg_length is not None:
            print(f"Average length: {avg_length:.2f}")

        results.append({
            "num_drones": num_drones,
            "steps": len(paths),
            "time": elapsed,
            "average_length": avg_length
        })

        last_successful_num_drones = num_drones
        last_successful_path = paths
        num_drones += 1

    print("\n==============================")
    print(f"Last successful number of drones: {last_successful_num_drones}")
    print("==============================")

    with open(RESULTS_FILE, "w") as f:
        f.write("A* Benchmark Results\n")
        f.write("====================\n\n")

        for result in results:
            f.write(
                f"Drones: {result['num_drones']}, "
                f"Steps: {result['steps']}, "
                f"Time: {result['time']:.3f} s, "
                f"Average length: {result['average_length']:.2f}\n"
            )

        f.write("\n")
        f.write(f"Maximum number of drones: {last_successful_num_drones}\n")

    print(f"Results saved to {RESULTS_FILE}")