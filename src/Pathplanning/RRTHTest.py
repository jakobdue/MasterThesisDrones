import time
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# import plotting function


from RRTH import plan_multiple_paths_rrtstar, plot_paths


# ----------------------------
# Settings
# ----------------------------
TIME_LIMIT_SECONDS = 15
RESULTS_FILE = "RRTStarResults.txt"

X_MIN, X_MAX = -5, 5
Y_MIN, Y_MAX = -5, 5
Z_MIN, Z_MAX = 0, 9

random.seed(42)

# ----------------------------
# Helpers
# ----------------------------
def compute_average_path_length(paths):
    total = 0
    count = 0

    for path in paths:
        if path is None:
            continue

        length = sum(
            math.dist(path[i], path[i - 1])
            for i in range(1, len(path))
        )
        total += length/5  # Divide by 5 to fit real world distance
        count += 1

    return (total / count) if count > 0 else None


def generate_circle_crossing(num_drones, radius=3, z=1):
    """
    Returns n pairs of points (start, end).
    Each pair defines a diameter of the circle.
    All paths intersect exactly at the center.
    Works for any n (odd or even).
    """


    if num_drones <= 0:
        return []

    # Collect integer points close to the circle boundary
    boundary_points = []

    for x in range(-radius, radius + 1):
        for y in range(-radius, radius + 1):
            dist = math.sqrt(x * x + y * y)

            # Accept points near the circle edge
            if abs(dist - radius) <= 0.75:
                boundary_points.append((x, y, z))

    # Sort points by angle around the circle
    boundary_points.sort(key=lambda p: math.atan2(p[1], p[0]))

    if len(boundary_points) < 2 * num_drones:
        raise ValueError(
            f"Not enough unique circle boundary points for {num_drones} drones. "
            f"Increase radius. Need {2 * num_drones}, got {len(boundary_points)}."
        )

    pairs = []
    used = set()

    half = len(boundary_points) // 2

    for i in range(len(boundary_points)):
        if len(pairs) >= num_drones:
            break

        start = boundary_points[i]
        goal = boundary_points[(i + half) % len(boundary_points)]

        if start not in used and goal not in used and start != goal:
            pairs.append((start, goal))
            used.add(start)
            used.add(goal)

    if len(pairs) < num_drones:
        raise ValueError(
            f"Could only generate {len(pairs)} unique start/end pairs. "
            f"Increase radius."
        )

    return pairs

def generate_random_missions(num_drones=4):
    while True:
        pairs = []
        starts = []
        goals = []

        for _ in range(num_drones):
            start = (
                random.randint(X_MIN + 1, X_MAX - 1),
                random.randint(Y_MIN + 1, Y_MAX - 1),
                random.randint(Z_MIN + 1, Z_MAX - 1),
            )
            goal = (
                random.randint(X_MIN + 1, X_MAX - 1),
                random.randint(Y_MIN + 1, Y_MAX - 1),
                random.randint(Z_MIN + 1, Z_MAX - 1),
            )

            starts.append(start)
            goals.append(goal)

        # ensure ALL positions are unique
        if len(set(starts + goals)) == 2 * num_drones:
            pairs = list(zip(starts, goals))
            return pairs

# ----------------------------
# 1. Circle scaling test
# ----------------------------
def test_circle_scaling(results):
    num_drones = 1
    last_paths = []
    while True:
        print(f"\nTrying {num_drones} drones (circle)...")
        try:
            pairs = generate_circle_crossing(num_drones)
        except ValueError as e:
            print(str(e))
            break

        start_time = time.perf_counter()
        paths = plan_multiple_paths_rrtstar(pairs, TIME_LIMIT_SECONDS)
        elapsed = time.perf_counter() - start_time

        if paths is None or any(p is None for p in paths) or elapsed > TIME_LIMIT_SECONDS:
            print("Failed")
            break

        avg_length = compute_average_path_length(paths)

        print(f"Success | Time: {elapsed:.3f} | Avg length: {avg_length:.2f}")

        results.append({
            "test": "circle",
            "drones": num_drones,
            "time": elapsed,
            "avg_length": avg_length
        })
        last_paths = paths 
        num_drones += 1

    if last_paths:
        plot_paths(last_paths)

# ----------------------------
# 2. Fixed missions test
# ----------------------------
def test_fixed_missions(results):
    mission_OG = [
        ((-5, -5, 0), (5, 5, 0)),
        ((-5, 5, 0), (5, -5, 0)),
        ((0, -5, 0), (0, 5, 0)),
        ((3, -5, 0), (3, 5, 0))
    ]

    mission_cross = [
        ((-1, -1, 0), (5, 5, 0)),
        ((-1, 1, 0), (5, -5, 0)),
        ((1, 1, 0), (-5, -5, 0)),
        ((1, -1, 0), (-5, 5, 0))
    ]

    for name, mission in [("OG", mission_OG), ("Cross", mission_cross)]:
        print(f"\nRunning mission: {name}")

        start_time = time.perf_counter()
        paths = plan_multiple_paths_rrtstar(mission, TIME_LIMIT_SECONDS)
        elapsed = time.perf_counter() - start_time

        if paths is None or any(p is None for p in paths):
            print("Failed")
            continue

        avg_length = compute_average_path_length(paths)

        print(f"Success | Time: {elapsed:.3f} | Avg length: {avg_length:.2f}")

        results.append({
            "test": name,
            "drones": len(mission),
            "time": elapsed,
            "avg_length": avg_length
        })
        plot_paths(paths)


# ----------------------------
# 3. Random missions test
# ----------------------------
def test_random_missions(results):
    print("\nRunning random missions...")

    deadline = time.perf_counter() + TIME_LIMIT_SECONDS
    successes = 0
    fails = 0
    lengths = []

    plotted = False
    while time.perf_counter() < deadline:
        pairs = generate_random_missions()

        paths = plan_multiple_paths_rrtstar(pairs, TIME_LIMIT_SECONDS)

        if paths is None or any(p is None for p in paths):
            fails += 1
            continue

        avg_length = compute_average_path_length(paths)
        lengths.append(avg_length)
        successes += 1
        

    print(f"Successes: {successes}")

    if lengths:
        results.append({
            "test": "random",
            "runs": successes,
            "avg_length": sum(lengths) / len(lengths)
        })
    
    if not plotted:
        plot_paths(paths)
        plotted = True
    
    # print total runs
    print(f"Total runs: {successes + fails}")



# ----------------------------
# MAIN
# ----------------------------
if __name__ == "__main__":
    results = []

    test_circle_scaling(results)
    test_fixed_missions(results)
    test_random_missions(results)

    print("\nSaving results...")

    with open(RESULTS_FILE, "w") as f:
        f.write("RRT* Benchmark Results\n")
        f.write("======================\n\n")

        for r in results:
            f.write(str(r) + "\n")

    print(f"Results saved to {RESULTS_FILE}")