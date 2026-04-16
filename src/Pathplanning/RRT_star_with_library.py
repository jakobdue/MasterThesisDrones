from ompl import base as ob
from ompl import geometric as og
import matplotlib.pyplot as plt
import numpy as np

# The contents of this file are adapted from a chatgpt first promt for a 2D RRT* example. 



# ----------------------------
# 1. Define state space
# ----------------------------
space = ob.RealVectorStateSpace(3)

bounds = ob.RealVectorBounds(3)

bounds.setLow(0, -1.8)   # x
bounds.setLow(1, -1.8)   # y
bounds.setLow(2, 0)    # z

bounds.setHigh(0, 1.8)
bounds.setHigh(1, 1.8)
bounds.setHigh(2, 3)
space.setBounds(bounds)

# ----------------------------
# 2. Custom validity checker
# ----------------------------
class ValidityChecker(ob.StateValidityChecker):
    def __init__(self, si):
        super().__init__(si)

    def isValid(self, state):
        (x,y,z) = state[0], state[1], state[2]
        y = state[1]
        z = state[2]
        return x * x + y * y + z * z > 1.0 * 1.0 * 1.0


# ----------------------------
# 3. SimpleSetup
# ----------------------------
ss = og.SimpleSetup(space)
si = ss.getSpaceInformation()

checker = ValidityChecker(si)
si.setStateValidityChecker(checker)
si.setup()

start = space.allocState()
start[0] = 0.01
start[1] = -1.5
start[2] = 0.0

goal = space.allocState()
goal[0] = 1.5
goal[1] = 1.5
goal[2] = 2.0

ss.setStartAndGoalStates(start, goal)

# ----------------------------
# 4. Planner
# ----------------------------
planner = og.RRTstar(si)
planner.setRange(0.2)
planner.setGoalBias(0.05)
ss.setPlanner(planner)

# ----------------------------
# 5. Solve
# ----------------------------
solved = ss.solve(2.0) # Allow up to 1 second to find a solution

if solved:
    ss.simplifySolution()   # ← IMPORTANT

    path = ss.getSolutionPath()
    path.interpolate()

    states = path.getStates()
    print("Found solution:")
    """ for state in states:
        print(state[0], state[1]) """

    x_vals = [state[0] for state in states]
    y_vals = [state[1] for state in states]
    z_vals = [state[2] for state in states]

    fig = plt.figure()

    
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x_vals, y_vals, z_vals, label='RRT* Path')
    # plot the ball obstacle
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = 1.5 * np.outer(np.cos(u), np.sin(v))
    y = 1.5 * np.outer(np.sin(u), np.sin(v))
    z = 1.5 * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x, y, z, color='r', alpha=0.5)

    ax.scatter(start[0], start[1], start[2], color='green', label='Start')
    ax.scatter(goal[0], goal[1], goal[2], color='red', label='Goal')
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