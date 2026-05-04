from ompl import base as ob
from ompl import geometric as og
import matplotlib.pyplot as plt
import numpy as np

# The contents of this file are adapted from a chatgpt first promt for a 2D RRT* example. 



# ----------------------------
# 1. Define state space
# ----------------------------
NUM_DRONES = 10
MIN_SEPARATION = 0.3
planning_algorithm = "RRT*"

def create_drone_space():
    drone_space = ob.RealVectorStateSpace(3)

    bounds = ob.RealVectorBounds(3)
    bounds.setLow(0, -1.8)   # x
    bounds.setLow(1, -1.8)   # y
    bounds.setLow(2, 0.0)    # z

    bounds.setHigh(0, 1.8)
    bounds.setHigh(1, 1.8)
    bounds.setHigh(2, 3.0)

    drone_space.setBounds(bounds)
    return drone_space

space = ob.CompoundStateSpace()
for _ in range(NUM_DRONES):
    space.addSubspace(create_drone_space(), 1.0)

# ----------------------------
# 2. Custom validity checker
# ----------------------------
class ValidityChecker(ob.StateValidityChecker):
    def __init__(self, si):
        super().__init__(si)

    def isValid(self, state):
        for i in range(NUM_DRONES):
            for j in range(i + 1, NUM_DRONES):
                dx = state[i][0] - state[j][0]
                dy = state[i][1] - state[j][1]
                dz = state[i][2] - state[j][2]

                dist_sq = dx * dx + dy * dy + dz * dz

                if dist_sq < MIN_SEPARATION * MIN_SEPARATION:
                    return False

        return True
# ----------------------------
# 3. SimpleSetup
# ----------------------------
ss = og.SimpleSetup(space)
si = ss.getSpaceInformation()

checker = ValidityChecker(si)
si.setStateValidityChecker(checker)
si.setup()

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

def generate_circle_crossing(num_drones=20, radius=1.5, z=0.7):
    sequences = ([],[])

    for i in range(num_drones):
        theta = 2 * np.pi * i / num_drones

        start = (
            radius * np.cos(theta),
            radius * np.sin(theta),
            z
        )

        end = (
            -start[0],
            -start[1],
            z
        )

        sequences = (sequences[0] + [start], sequences[1] + [end])

    return sequences


start_positions, goal_positions = generate_circle_crossing(10)

start = ss.getStateSpace().allocState()
goal = ss.getStateSpace().allocState()

for i in range(NUM_DRONES):
    start[i][0] = start_positions[i][0]
    start[i][1] = start_positions[i][1]
    start[i][2] = start_positions[i][2]

    goal[i][0] = goal_positions[i][0]
    goal[i][1] = goal_positions[i][1]
    goal[i][2] = goal_positions[i][2]

ss.setStartAndGoalStates(start, goal)



# ----------------------------
# 4. Planner RRT*
# ----------------------------
planner_rrt = og.RRTstar(si)
planner_rrt.setRange(0.2)
planner_rrt.setGoalBias(0.05)
ss.setPlanner(planner_rrt)

# ----------------------------
# 5. Solve
# ----------------------------
solved = ss.solve(2.0) # Allow up to 2 second to find a solution

if solved:
    ss.simplifySolution()   # ← IMPORTANT

    path = ss.getSolutionPath()
    path.interpolate()

    states = path.getStates()
    print("Found solution:")
    """ for state in states:
        print(state[0], state[1]) """

    drone_paths = [[] for _ in range(NUM_DRONES)]

    for state in states:
        for i in range(NUM_DRONES):
            x = state[i][0]
            y = state[i][1]
            z = state[i][2]
            drone_paths[i].append((x, y, z))

    import json

    output_data = {
        "num_drones": NUM_DRONES,
        "paths": drone_paths
    }

    with open("drone_paths.json", "w") as f:
        json.dump(output_data, f, indent=2)

    print("Saved paths to drone_paths.json")


    fig = plt.figure()

    
    ax = fig.add_subplot(111, projection='3d')
    for i in range(NUM_DRONES):
        x_vals = [p[0] for p in drone_paths[i]]
        y_vals = [p[1] for p in drone_paths[i]]
        z_vals = [p[2] for p in drone_paths[i]]

        ax.plot(x_vals, y_vals, z_vals, label=f'Drone {i+1} Path')
        ax.scatter(x_vals[0], y_vals[0], z_vals[0], marker='o')
        ax.scatter(x_vals[-1], y_vals[-1], z_vals[-1], marker='x')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('RRT* Path Planning in 3D')
    ax.legend()
    plt.show()
    plt.savefig("rrt_star_path.png")


    del path
    del planner
    del ss
    del si
    del checker
    del start
    del goal
else:
    print("No solution found")




