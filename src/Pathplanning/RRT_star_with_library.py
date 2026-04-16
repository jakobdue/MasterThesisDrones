import math
from ompl import base as ob
from ompl import geometric as og

# ----------------------------
# 1. Define state space
# ----------------------------
space = ob.RealVectorStateSpace(2)

bounds = ob.RealVectorBounds(2)
bounds.setLow(-2)
bounds.setHigh(2)
space.setBounds(bounds)

# ----------------------------
# 2. Define validity (collision checking)
# ----------------------------
def isStateValid(state):
    x = state[0]
    y = state[1]

    # circular obstacle at (0,0) radius 0.5
    if x**2 + y**2 <= 0.5**2:
        return False
    return True

# ----------------------------
# 3. Setup problem
# ----------------------------
si = ob.SpaceInformation(space)
si.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
si.setup()

start = ob.State(space)
start[0] = -1.5
start[1] = -1.5

goal = ob.State(space)
goal[0] = 1.5
goal[1] = 1.5

pdef = ob.ProblemDefinition(si)
pdef.setStartAndGoalStates(start, goal)

# ----------------------------
# 4. Choose planner (RRT*)
# ----------------------------
planner = og.RRTstar(si)
planner.setProblemDefinition(pdef)
planner.setup()

# ----------------------------
# 5. Solve
# ----------------------------
solved = planner.solve(1.0)  # seconds

if solved:
    print("Found solution:")
    path = pdef.getSolutionPath()
    path.interpolate()  # make it smoother

    for state in path.getStates():
        print(state[0], state[1])
else:
    print("No solution found")