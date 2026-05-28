import json
import matplotlib.pyplot as plt
import numpy as np

# Load data
with open("ompl_circle_paths.json", "r") as f:
    data = json.load(f)

NUM_DRONES = data["num_drones"]
drone_paths = data["paths"]

# Create figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot each drone
for i in range(NUM_DRONES):
    path = drone_paths[i]

    x_vals = [p[0] for p in path]
    y_vals = [p[1] for p in path]
    z_vals = [p[2] for p in path]

    ax.plot(x_vals, y_vals, z_vals, label=f'Drone {i+1}')
    ax.scatter(x_vals[0], y_vals[0], z_vals[0], marker='o')
    ax.scatter(x_vals[-1], y_vals[-1], z_vals[-1], marker='x')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Drone Paths')
ax.legend()

plt.show()
