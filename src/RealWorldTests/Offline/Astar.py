
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator
from cflib.crazyflie.swarm import CachedCfFactory
from functools import partial
import random
import math
import heapq
from typing import List, Tuple, Set
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from cflib.crazyflie.swarm import Swarm

# URI of the Crazyflie to connect to
uri1 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E709')
uri2 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E705')
uri3 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E710')
uri4 = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E707')
uris = [uri1, uri2, uri3, uri4] 

curr_pos = {uri: (0, 0, 0) for uri in uris}
backup_sequences = {}
sequences = {}
num_runs = 3
PLOT_PATHS = False

if num_runs > 1:
    return_to_origin = True
else:
    return_to_origin = False

missions_ = [ # Mission 1
    ((-5, -5, 0), (5, 5, 0)),
    ((-5, 5, 0), (5, -5, 0)),
    ((0, -5, 0), (0, 5, 0)),
    ((3, -5, 0), (3, 5, 0))
]

return_to_origin_paths_ = { # Mission 1
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

missions = [ # Cross
    ((-1, -1, 0), (5, 5, 0)),
    ((-1, 1, 0), (5, -5, 0)),
    ((1, 1, 0), (-5, -5, 0)),
    ((1, -1, 0), (-5, 5, 0))
]

return_to_origin_paths = { # Cross
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

position_params = {
        uri1: [0],
        uri2: [1],
        uri3: [2],
        uri4: [3]
        }

# 3D Grid Limits
X_MIN, X_MAX = -5, 5
Y_MIN, Y_MAX = -5, 5
Z_MIN, Z_MAX = 0, 9

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

    # Adding missions start and end as obstacles to avoid collisions at start and end
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

    # calculating average path length
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
    fig.savefig("paths.pdf")

paths = plan_multiple_paths(missions)
if PLOT_PATHS:
    plot_paths(paths)

for i, path in enumerate(paths):
    if path is None:
        continue

    seq = []
    # Consolidate path by removing points that are on the same line (i.e. only keep points where the direction changes)
    filtered_path = [path[0]]
    for j in range(1, len(path)-2):
        prev = filtered_path[-1]
        curr = path[j]
        next = path[j+1]

        # Checking if the prev change is on x, y or z 
        prev_change = (prev[0] != curr[0], prev[1] != curr[1], prev[2] != curr[2])
        next_change = (curr[0] != next[0], curr[1] != next[1], curr[2] != next[2])

        if prev_change == next_change:
            continue  # Skip this point
        else:
            filtered_path.append(curr)
    filtered_path.append(path[-1])  # Add the last point
    print("Path", path)
    print(f"Filtered path: {filtered_path}")
    path = filtered_path

    # Converting grid path to float format and add yaw=0
    for x, y, z in path:
        seq.append((float(x)/100*20, float(y)/100*20, (float(z)+2)/100*20, 0))

    # Adding landing point (same x,y, z=0)
    last_x, last_y, _ = path[-1]
    seq.append((float(last_x)/100*20, float(last_y)/100*20, 0.0, 0))

    sequences[i] = seq

def take_off(cf, position):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position[2] / take_off_time

    print(f'take off at {position[2]}')

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)

def position_callback(uri, timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    curr_pos[uri] = (x, y, z)

def start_position_printing(scf):
    uri = scf.cf.link_uri

    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')
    scf.cf.log.add_config(log_conf)

    # Binding uri into the callback
    log_conf.data_received_cb.add_callback(partial(position_callback, uri))
    log_conf.start()

def square_dist(pos1, pos2):
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    dz = pos1[2] - pos2[2]

    return (dx**2 + dy**2 + dz**2)

def limit_velocity(cf, xy=0.25, z=0.20):
    cf.param.set_value('posCtlPid.xVelMax', xy)
    cf.param.set_value('posCtlPid.yVelMax', xy)
    cf.param.set_value('posCtlPid.zVelMax', z)

def run_sequence(scf, num_seq):
    cf = scf.cf

    # Arming the Crazyflie
    cf.platform.send_arming_request(True)
    time.sleep(1.0)

    start = time.perf_counter()
    me = uris[num_seq]

    for position in sequences[num_seq]:
        print('Setting position {}'.format(position))
        while (square_dist(curr_pos[me], position) > 0.025):
            cf.commander.send_position_setpoint(position[0],
                                                    position[1],
                                                    position[2],
                                                    position[3])
            time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    # Give control to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()
    end = time.perf_counter()
    runtime = end - start
    runtimes.append(runtime)

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

def update_to_return():
    global sequences, backup_sequences
    backup_sequences = sequences
    sequences = return_to_origin_paths
    
def update_to_old_sequences():
    global sequences, runtimes
    sequences = backup_sequences
    runtimes = []

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    safety_distance = 1.0  # meters

    factory = CachedCfFactory(rw_cache='./cache')
    for _ in range(num_runs):  # Run the sequence num times
        with Swarm(uris, factory=factory) as swarm:
            swarm.parallel_safe(start_position_printing)
            swarm.parallel_safe(run_sequence, args_dict=position_params)
        # calculating average runtimes and write to file
        avg_runtime = sum(runtimes) / len(runtimes)
        
        with open("timings_Astar_15_may_fast.txt", "a") as f:
            f.write(f"A* fast algorithm runs in: {avg_runtime}\n")
        
        if return_to_origin:
            update_to_return()
            with Swarm(uris, factory=factory) as swarm:
                swarm.parallel_safe(start_position_printing)
                swarm.parallel_safe(run_sequence, args_dict=position_params)
            update_to_old_sequences()
