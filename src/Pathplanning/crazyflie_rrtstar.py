# -*- coding: utf-8 -*-
"""
Crazyflie swarm waypoint planning using RRT* (replaces A*).

- Plans one path per mission sequentially.
- Each planned path is added to a global obstacle set (like your working A* script).
- Uses a 3D bounded workspace, but the example missions are on z=0.

NOTE: This implements a lightweight, practical RRT* for your grid-like space:
- Samples are continuous (floats) within bounds.
- Collision checking is done by rounding points to grid cells and checking against obstacles.
- Segment collision is checked by discretizing along edges.

If you want more/less separation, tune:
- SAFETY_RADIUS_CELLS
- STEP_SIZE
- SEARCH_RADIUS
- GOAL_REGION_RADIUS
- MAX_ITER
"""
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

# ----------------------------
# Crazyflie URIs (edit as needed)
# ----------------------------
uri1 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E701')
uri2 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E708')
uri3 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E707')
uri4 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E703')
uris = [uri1, uri2, uri3, uri4]

# Missions: ((x,y,z), (x,y,z)) in grid coordinates
mission_1 = [
    ((-5, -5, 0), (5, 5, 0)),
    ((-5, 5, 0), (5, -5, 0)),
    ((0, -5, 0), (0, 5, 0)),
    ((3, -5, 0), (3, 5, 0)),
]

missions = [
    ((-1, -1, 0), (5, 5, 0)),
    ((-1, 1, 0), (5, -5, 0)),
    ((1, 1, 0), (-5, -5, 0)),
    ((1, -1, 0), (-5, 5, 0))
]

runtimes = []


position_params = {
    uri1: [0],
    uri2: [1],
    uri3: [2],
    uri4: [3],
}

# ----------------------------
# Planning workspace (grid bounds)
# ----------------------------
X_MIN, X_MAX = -5, 5
Y_MIN, Y_MAX = -5, 5
Z_MIN, Z_MAX = 0, 9

# ----------------------------
# RRT* tuning
# ----------------------------
STEP_SIZE = 1.0                 # steering step in workspace units
MAX_ITER = 3000                 # iterations per mission
GOAL_SAMPLE_RATE = 0.20         # probability of sampling the goal
GOAL_REGION_RADIUS = 1.2        # goal reached threshold (workspace units)
SEARCH_RADIUS = 2.5             # neighbor radius for choosing parent/rewire

SAFETY_RADIUS_CELLS = 1         # inflate obstacles by this many grid cells (0 = off)


# ----------------------------
# Utility
# ----------------------------
def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

def round_cell(p: Tuple[float, float, float]) -> Tuple[int, int, int]:
    return (int(round(p[0])), int(round(p[1])), int(round(p[2])))

def in_bounds_cell(c: Tuple[int, int, int]) -> bool:
    x, y, z = c
    return X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX and Z_MIN <= z <= Z_MAX

def inflate_cell(cell: Tuple[int, int, int], radius: int) -> List[Tuple[int, int, int]]:
    x, y, z = cell
    out = []
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            for dz in range(-radius, radius + 1):
                c = (x + dx, y + dy, z + dz)
                if in_bounds_cell(c):
                    out.append(c)
    return out


# ----------------------------
# RRT* structures
# ----------------------------
class Node:
    __slots__ = ("x", "y", "z", "parent", "cost")
    def __init__(self, x: float, y: float, z: float):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.parent: Optional["Node"] = None
        self.cost: float = 0.0

    def pos(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)

class RRTStar3D:
    def __init__(
        self,
        start: Tuple[int, int, int],
        goal: Tuple[int, int, int],
        obstacles: Set[Tuple[int, int, int]],
    ):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.obstacles = obstacles  # grid cells
        self.node_list: List[Node] = [self.start]

    # ---- collision checking ----
    def is_point_free(self, p: Tuple[float, float, float]) -> bool:
        cell = round_cell(p)
        if not in_bounds_cell(cell):
            return False
        if SAFETY_RADIUS_CELLS <= 0:
            return cell not in self.obstacles
        # inflated
        for c in inflate_cell(cell, SAFETY_RADIUS_CELLS):
            if c in self.obstacles:
                return False
        return True

    def is_segment_free(self, a: Tuple[float, float, float], b: Tuple[float, float, float]) -> bool:
        # discretize segment
        ax, ay, az = a
        bx, by, bz = b
        dist = math.dist(a, b)
        if dist == 0:
            return self.is_point_free(a)
        steps = max(2, int(math.ceil(dist / 0.25)))  # resolution
        for i in range(steps + 1):
            t = i / steps
            p = (ax + (bx - ax) * t, ay + (by - ay) * t, az + (bz - az) * t)
            if not self.is_point_free(p):
                return False
        return True

    # ---- sampling / nearest / neighbors ----
    def get_random_node(self) -> Node:
        if random.random() < GOAL_SAMPLE_RATE:
            return Node(self.goal.x, self.goal.y, self.goal.z)
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)
        z = random.uniform(Z_MIN, Z_MAX)
        return Node(x, y, z)

    def get_nearest_node(self, rand: Node) -> Node:
        rx, ry, rz = rand.x, rand.y, rand.z
        best = self.node_list[0]
        best_d = float("inf")
        for n in self.node_list:
            d = (n.x - rx) ** 2 + (n.y - ry) ** 2 + (n.z - rz) ** 2
            if d < best_d:
                best_d = d
                best = n
        return best

    def steer(self, from_node: Node, to_node: Node) -> Node:
        fx, fy, fz = from_node.x, from_node.y, from_node.z
        tx, ty, tz = to_node.x, to_node.y, to_node.z
        dx, dy, dz = tx - fx, ty - fy, tz - fz
        d = math.sqrt(dx * dx + dy * dy + dz * dz)
        if d == 0:
            new = Node(fx, fy, fz)
        else:
            scale = min(STEP_SIZE / d, 1.0)
            new = Node(
                fx + dx * scale,
                fy + dy * scale,
                fz + dz * scale,
            )
        new.cost = from_node.cost + math.dist(from_node.pos(), new.pos())
        new.parent = from_node
        return new

    def find_neighbors(self, new_node: Node) -> List[Node]:
        out = []
        r2 = SEARCH_RADIUS * SEARCH_RADIUS
        nx, ny, nz = new_node.x, new_node.y, new_node.z
        for n in self.node_list:
            d2 = (n.x - nx) ** 2 + (n.y - ny) ** 2 + (n.z - nz) ** 2
            if d2 <= r2:
                out.append(n)
        return out

    def choose_parent(self, neighbors: List[Node], nearest: Node, new_node: Node) -> Node:
        best = nearest
        best_cost = nearest.cost + math.dist(nearest.pos(), new_node.pos())

        for nb in neighbors:
            c = nb.cost + math.dist(nb.pos(), new_node.pos())
            if c < best_cost and self.is_segment_free(nb.pos(), new_node.pos()):
                best = nb
                best_cost = c

        new_node.parent = best
        new_node.cost = best_cost
        return new_node

    def rewire(self, new_node: Node, neighbors: List[Node]) -> None:
        for nb in neighbors:
            new_cost = new_node.cost + math.dist(nb.pos(), new_node.pos())
            if new_cost < nb.cost and self.is_segment_free(new_node.pos(), nb.pos()):
                nb.parent = new_node
                nb.cost = new_cost

    def reached_goal(self, node: Node) -> bool:
        return math.dist(node.pos(), self.goal.pos()) <= GOAL_REGION_RADIUS

    def generate_final_path(self, goal_node: Node) -> List[Tuple[int, int, int]]:
        # backtrack then convert to integer grid cells (rounded)
        pts: List[Tuple[int, int, int]] = []
        n: Optional[Node] = goal_node
        while n is not None:
            pts.append(round_cell(n.pos()))
            n = n.parent
        pts = pts[::-1]
        # remove immediate duplicates
        dedup = []
        for p in pts:
            if not dedup or dedup[-1] != p:
                dedup.append(p)
        return dedup

    def plan(self) -> Optional[List[Tuple[int, int, int]]]:
        # ensure start/goal are free (except they might be in "local obstacles" outside)
        if not self.is_point_free(self.start.pos()):
            print("Start in collision:", round_cell(self.start.pos()))
            return None
        if not self.is_point_free(self.goal.pos()):
            print("Goal in collision:", round_cell(self.goal.pos()))
            return None

        best_goal_node: Optional[Node] = None
        best_goal_cost = float("inf")

        for _ in range(MAX_ITER):
            rand = self.get_random_node()
            nearest = self.get_nearest_node(rand)
            new = self.steer(nearest, rand)

            # clamp to bounds (continuous)
            new.x = clamp(new.x, X_MIN, X_MAX)
            new.y = clamp(new.y, Y_MIN, Y_MAX)
            new.z = clamp(new.z, Z_MIN, Z_MAX)

            if not self.is_segment_free(nearest.pos(), new.pos()):
                continue

            neighbors = self.find_neighbors(new)
            new = self.choose_parent(neighbors, nearest, new)
            self.node_list.append(new)
            self.rewire(new, neighbors)

            if self.reached_goal(new):
                # connect to exact goal if possible
                if self.is_segment_free(new.pos(), self.goal.pos()):
                    goal_node = Node(self.goal.x, self.goal.y, self.goal.z)
                    goal_node.parent = new
                    goal_node.cost = new.cost + math.dist(new.pos(), self.goal.pos())
                    if goal_node.cost < best_goal_cost:
                        best_goal_cost = goal_node.cost
                        best_goal_node = goal_node

        if best_goal_node is None:
            return None
        return self.generate_final_path(best_goal_node)


# ----------------------------
# Multi-mission planning (RRT*)
# ----------------------------
def plan_multiple_paths_rrtstar(
    pairs: List[Tuple[Tuple[int, int, int], Tuple[int, int, int]]]
) -> List[Optional[List[Tuple[int, int, int]]]]:
    obstacles: Set[Tuple[int, int, int]] = set()
    all_paths: List[Optional[List[Tuple[int, int, int]]]] = []

    for start, goal in pairs:
        # local obstacles: other starts/goals
        local_obstacles = set()
        for s2, g2 in pairs:
            if (s2, g2) != (start, goal):
                local_obstacles.add(s2)
                local_obstacles.add(g2)

        planner = RRTStar3D(start, goal, obstacles.union(local_obstacles))
        path = planner.plan()

        if path is None:
            print(f"No path found for {start} -> {goal}")
            all_paths.append(None)
        else:
            all_paths.append(path)
            # Make the final path an obstacle for later missions (grid cells)
            obstacles.update(path)

    return all_paths


def plot_paths(paths: List[Optional[List[Tuple[int, int, int]]]]) -> None:
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
    total_length = 0.0
    count = 0
    for path in paths:
        if path is None:
            continue
        length = 0.0
        for j in range(1, len(path)):
            length += math.dist(path[j-1], path[j])
        total_length += length
        count += 1
    if count > 0:
        print(f"Average path length: {total_length / count:.2f}")

    plt.show()
    fig.savefig("paths_rrtstar.pdf")


# ----------------------------
# Convert paths to Crazyflie sequences (same mapping you used)
# ----------------------------
def compress_path_turnpoints(path: List[Tuple[int, int, int]]) -> List[Tuple[int, int, int]]:
    if len(path) <= 2:
        return path
    filtered = [path[0]]
    for j in range(1, len(path) - 1):
        prev = filtered[-1]
        curr = path[j]
        nxt = path[j + 1]
        prev_change = (prev[0] != curr[0], prev[1] != curr[1], prev[2] != curr[2])
        next_change = (curr[0] != nxt[0], curr[1] != nxt[1], curr[2] != nxt[2])
        if prev_change == next_change:
            continue
        filtered.append(curr)
    filtered.append(path[-1])
    return filtered


def path_to_sequence(path: List[Tuple[int, int, int]]) -> List[Tuple[float, float, float, float]]:
    seq: List[Tuple[float, float, float, float]] = []
    for x, y, z in path:
        seq.append((float(x)/100*20, float(y)/100*20, (float(z)+2)/100*20, 0.0))
    last_x, last_y, _ = path[-1]
    seq.append((float(last_x)/100*20, float(last_y)/100*20, 0.0, 0.0))
    return seq


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

def run_sequence(scf, num_seq, sequences):
    cf = scf.cf

    cf.platform.send_arming_request(True)
    time.sleep(1.0)

    take_off(cf, sequences[num_seq][0])
    time.sleep(1.0)
    start = time.perf_counter()

    for position in sequences[num_seq]:
        print(f"[{scf.cf.link_uri}] Setting position {position}")
        for _ in range(25):
            cf.commander.send_position_setpoint(position[0], position[1], position[2], position[3])
            time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    cf.commander.send_notify_setpoint_stop()

    end = time.perf_counter()
    runtime = end - start
    runtimes.append(runtime)

    time.sleep(0.1)


# ----------------------------
# Main
# ----------------------------
if __name__ == "__main__":
    # Plan
    paths = plan_multiple_paths_rrtstar(missions)
    #plot_paths(paths)

    # Build sequences
    sequences = {}
    for i, path in enumerate(paths):
        if path is None:
            continue
        path2 = compress_path_turnpoints(path)
        print("Path", path)
        print("Compressed path:", path2)
        sequences[i] = path_to_sequence(path2)

    # Fly
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    # Build args_dict for swarm.parallel_safe: each URI gets (index, sequences)
    args_dict = {uri: (idxs[0], sequences) for uri, idxs in position_params.items()}

    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(start_position_printing)
        swarm.parallel_safe(run_sequence, args_dict=args_dict)

    # calculate average runtimes and write to file
    avg_runtime = sum(runtimes) / len(runtimes)

    with open("timings_cross_mission.txt", "a") as f:
        f.write(f"RRT* algorithm runs in: {avg_runtime}\n")


