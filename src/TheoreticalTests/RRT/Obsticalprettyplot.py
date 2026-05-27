import json
import glob
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def plot_from_json(filename):
    with open(filename, "r") as f:
        data = json.load(f)

    paths = data["paths"]
    obstacles = data.get("obstacles", [])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot obstacles
    if obstacles:
        obs_x, obs_y, obs_z = zip(*obstacles)
        ax.scatter(obs_x, obs_y, obs_z, c='red', marker='x', label='Obstacles')

    # Plot drone paths
    for i, path in enumerate(paths):
        x, y, z = zip(*path)
        ax.plot(x, y, z, label=f'Drone {i+1}')

    ax.set_title(f"{filename}")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()

    plt.show()


if __name__ == "__main__":
    files = sorted(glob.glob("OMPLOBSTACLE/ompl_obstacle_run_*.json"))

    ## plot every 10 files to avoid too many plots
    for i, file in enumerate(files):
        if i % 5 == 0:
            plot_from_json(file)
