import networkx as nx
from itertools import product

MOVES = [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),
         (0,0,1),(0,0,-1),(0,0,0)]

def add(p, m):
    return (p[0]+m[0], p[1]+m[1], p[2]+m[2])

class JointGraph(nx.Graph):
    def __init__(self, bounds, n_drones):
        super().__init__()
        self.bounds = bounds
        self.n = n_drones

    def neighbors(self, state):
        positions = [state[i:i+3] for i in range(0, len(state), 3)]

        for moves in product(MOVES, repeat=self.n):
            new_positions = tuple(add(p, m) for p, m in zip(positions, moves))

            # bounds check
            if not all(0 <= p[i] < self.bounds[i] for p in new_positions for i in range(3)):
                continue

            # collision: same cell
            if len(set(new_positions)) < len(new_positions):
                continue

            yield tuple(coord for p in new_positions for coord in p)



def heuristic(a, b):
    h = 0
    for i in range(0, len(a), 3):
        h += abs(a[i]   - b[i])
        h += abs(a[i+1] - b[i+1])
        h += abs(a[i+2] - b[i+2])
    return h



G = JointGraph(bounds=(10,10,9), n_drones=4)

start = (0,0,0,  1,0,0,  2,0,0,  3,0,0)
goal  = (10,10,2, 11,10,2, 12,10,2, 13,10,2)

start_positions = [
    (-1.70, -1.70, 0.10),
    (-1.70, -1.20, 0.20),
    (-1.70, -0.70, 0.30),
    (-1.70, -0.20, 0.40),
    (-1.70,  0.30, 0.50),
    (-1.70,  0.80, 0.60),
    (-1.70,  1.10, 0.70),
    (-1.20,  1.50, 0.80),
    (-0.70,  1.50, 0.90),
    (-0.20,  1.50, 1.00),
]
# collapse to single tuple
start = tuple(coord for p in start_positions for coord in p)

# 10 goal positions
goal_positions = [
    ( 1.70,  1.70, 2.80),
    ( 1.70,  1.20, 2.70),
    ( 1.70,  0.70, 2.60),
    ( 1.70,  0.20, 2.50),
    ( 1.70, -0.30, 2.40),
    ( 1.70, -0.80, 2.30),
    ( 1.70, -1.10, 2.20),
    ( 1.20, -1.50, 2.10),
    ( 0.70, -1.50, 2.00),
    ( 0.20, -1.50, 1.90),
]
goal = tuple(coord for p in goal_positions for coord in p)

path = nx.astar_path(G, start, goal, heuristic=heuristic)

# plot path in 3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for i in range(0, len(path[0]), 3):
    x = [state[i] for state in path]
    y = [state[i+1] for state in path]
    z = [state[i+2] for state in path]
    ax.plot(x, y, z)
plt.show()  
