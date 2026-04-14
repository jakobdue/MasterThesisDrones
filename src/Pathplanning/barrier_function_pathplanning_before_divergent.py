import matplotlib.pyplot as plt
from scipy.optimize import minimize
import numpy as np



type Point = tuple[float, float, float]
type Path  = list[Point] # points is a list of tuples with (x,y,z) coordinates.

# constants: 
eta_safety_distance = 0.40
T = 50
dt = 1 / (T - 1) # normalize to the number of timesteps.


cross_sequences = {
    0: [(-0.2, -0.2, 0.7), (1.0, 1.0, 0.7)],
    1: [(-0.2,  0.2, 0.7), (1.0, -1.0, 0.7)],
    2: [(0.2, 0.2, 0.7), (-1.0, -1.0, 0.7)],
    3: [(0.2, -0.2, 0.7), (-1.0, 1.0, 0.7)],
}



original_sequences = {
    0: [(-1.0, -1.0, 0.7), (1.2, 1.0, 0.7)],
    1: [(-1.0,  1.0, 0.7), (1.0, -1.0, 0.7)],
    2: [(0.0, -1.0, 0.7), (-0.2, 1.0, 0.7)],
    3: [(0.6, -1.0, 0.7), (0.6, 1.0, 0.7)],
}



def generate_circle_crossing(num_drones=20, radius=1.5, z=0.7):
    sequences = {}

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

        sequences[i] = [start, end]

    return sequences


#original_sequences = generate_circle_crossing(50)


# interpolate points in the paths to have T points in total. there is always two points in the path, so we can interpolate T-2 points between them.
def interpolate_path(path: Path, noise_std: float = 0.01) -> Path:
    if len(path) < 2:
        raise ValueError("Path must have at least 2 points to interpolate.")
    
    start, end = path[0], path[1]
    interpolated_path = []
    
    for t in range(T):
        alpha = t / (T - 1)  # alpha goes from 0 to 1
        point = (
            start[0] + alpha * (end[0] - start[0]),
            start[1] + alpha * (end[1] - start[1]),
            start[2] + alpha * (end[2] - start[2])
        )
        # add noise only to interior points
        if 0 < t < T-1:
            noise = np.random.normal(0.0, noise_std, size=3)
            point = (
                point[0] + noise[0],
                point[1] + noise[1],
                max(point[2] + noise[2], 0.0)  # optional: keep above ground
            )
        interpolated_path.append(point)
    
    return interpolated_path




drones = [
    interpolate_path(cross_sequences[i])
    for i in range(len(cross_sequences))
]



def make_bounds(paths):
    bounds = []
    for path in paths:
        for k, point in enumerate(path):
            for d in range(3):
                if k == 0 or k == T-1:
                    bounds.append((point[d], point[d]))  # fixed
                elif d == 2:
                    bounds.append((0.0, None))  # z >= 0
                else:
                    bounds.append((None, None))
    return bounds

def flatten_paths(paths: list[Path]) -> np.ndarray:
    return np.array([coord for path in paths for point in path for coord in point])

def unflatten_paths(x: np.ndarray, num_paths: int) -> list[Path]:
    paths = []
    idx = 0
    for _ in range(num_paths):
        path = []
        for _ in range(T):
            path.append((x[idx], x[idx+1], x[idx+2]))  # clamp z here
            idx += 3
        paths.append(path)
    return paths

x0 = flatten_paths(drones)
bounds = make_bounds(drones)


def add_vecs(vec1, vec2):
    return (vec1[0] + vec2[0], vec1[1] + vec2[1], vec1[2] + vec2[2])

def scale_vec(scaler, vec):
    return (vec[0] * scaler, vec[1] * scaler, vec[2] * scaler)

def sub_vecs(vec1, vec2):
    return (vec1[0] - vec2[0], vec1[1] - vec2[1], (vec1[2] - vec2[2]))

def vec_length(vec):
    return (vec[0]**2 + vec[1]**2 + vec[2]**2)**0.5

def squared_vec_length(vec):
    return (vec[0]**2 + vec[1]**2 + vec[2]**2)

def clip(vec, max_norm=40.0):
    norm = vec_length(vec)
    if norm > max_norm:
        return scale_vec(max_norm / norm, vec)
    return vec

def cubic_spline_B(v): 
    if abs(v) < 1:
        return (2/3) - v**2 + 0.5 * abs(v)**3
    elif 1 <= abs(v) < 2:
        return (1/6) * (2 - abs(v))**3
    else:
        return 0

def h_eps(z, eps):
    return 1.5 * cubic_spline_B(2 * z / eps)

def p_eps(z, eps, n):
    if n not in (2, 3):
        raise ValueError("n must be 2 or 3")
    return h_eps(z, eps) / (z ** (n - 1))



def cubic_spline_B_derivative(v):
    if v < 1:
        return -2*v + 1.5 * v ** 2
    elif 1 <= v < 2:
        return -0.5 * (2 - v)**2
    else:
        return 0.0

def barrier_force_cubic_spline(pos1, pos2, eps):
    dx, dy, dz = sub_vecs(pos1, pos2)
    z = vec_length((dx, dy, dz))

    if z < 1e-6:
        return (0, 0, 0)

    # Only apply force inside interaction radius
    if z >= eps:
        return (0, 0, 0)

    # Compute h(z)
    v = 2 * z / eps
    B = cubic_spline_B(v)
    h = 1.5 * B

    # Compute h'(z)
    B_prime = cubic_spline_B_derivative(v)
    h_prime = (3 / eps) * B_prime

    n = 3  # since we're using 3D

    # p_prime (Negative gradient of the penalty function with respect to z)
    p_prime = (z * h_prime - (n-1) * h) / z**(n+1) 

    # gradient = dp/dz * (x - y)
    grad = (p_prime * dx, p_prime * dy, p_prime * dz)

    return grad



def total_path_energy(paths:list[Path]): 
    total_path_length_energy = 0.0
    # calculate the total lenght of a path by summing the distances between consecutive points only taking the square root in the end to save computation time.
    for path in paths: 
        for k in range(len(path)-1): 
            diff = sub_vecs(path[k+1], path[k])
            total_path_length_energy += squared_vec_length(diff) / dt
    
    total_barrier_energy = 0.0
    for k in range(T):
        for i in range(len(paths)): 
            for j in range(i+1, len(paths)): 
                (dx, dy, dz) = sub_vecs(paths[i][k], paths[j][k])

                distance = vec_length((dx, dy, dz))
                total_barrier_energy += p_eps(distance, eta_safety_distance, 3)

    total_diversion_energy = 0.0

         
    return (total_path_length_energy, total_barrier_energy)


def gradient_barrier_energy(paths:list[Path]):
    grad_smooth:list[Path] = [[(0,0,0) for _ in range(T)] for _ in range(len(paths))]
    grad_barrier:list[Path] = [[(0,0,0) for _ in range(T)] for _ in range(len(paths))]
    for i in range(len(paths)):
        for k in range(T):
            if k == 0 or k == T-1:
              continue
            else:
                grad_smooth[i][k] = scale_vec(
                    2 / dt,
                    sub_vecs(
                        sub_vecs(scale_vec(2, paths[i][k]), paths[i][k-1]),
                        paths[i][k+1]
                    )
                )
            
        # compute the gradient of the barrier energy with respect to the position of each point in the path.
        for k in range(T):
            per_point_barrier_force = (0, 0, 0)
            for j in range(len(paths)):
                if j == i:
                    continue
                else: 
                    per_point_barrier_force = add_vecs(per_point_barrier_force, barrier_force_cubic_spline(paths[i][k], paths[j][k], eta_safety_distance))
            grad_barrier[i][k] = per_point_barrier_force
                
    return (grad_smooth, grad_barrier)


    
def objective(x: np.ndarray, num_paths: int):
    paths = unflatten_paths(x, num_paths)
    path_energy, barrier_energy = total_path_energy(paths)
    return barrier_energy + path_energy

def objective_grad(x: np.ndarray, num_paths: int):
    paths = unflatten_paths(x, num_paths)
    grad_smooth, grad_barrier = gradient_barrier_energy(paths)

    grad = []
    for i in range(len(paths)):
        for k in range(T):
            g = add_vecs(grad_smooth[i][k], grad_barrier[i][k])
            grad.extend(g)

    return np.array(grad)






res = minimize(
    objective,
    x0,
    args=(len(drones),),
    method='L-BFGS-B',   # good default
    jac=objective_grad,
    bounds=bounds,
    options={'maxiter': 200, 'disp': True}
)

new_paths = unflatten_paths(res.x, len(drones))



def print_closest_pairs(paths: list[Path]):
    global_min_dist = float('inf')
    global_info = None

    for k in range(T):
        for i in range(len(paths)):
            for j in range(i+1, len(paths)):
                diff = sub_vecs(paths[i][k], paths[j][k])
                dist = vec_length(diff)

                if dist < global_min_dist:
                    global_min_dist = dist
                    global_info = (k, i, j, paths[i][k], paths[j][k])

    k, i, j, p1, p2 = global_info

    print("\nClosest pair overall:")
    print(f" timestep k = {k}")
    print(f" drones ({i}, {j})")
    print(f" distance = {global_min_dist:.6f}")
    print(f" point {i}: {p1}")
    print(f" point {j}: {p2}")

print_closest_pairs(new_paths)

# print energy components after optimization
final_path_energy, final_barrier_energy = total_path_energy(new_paths)
print(f"Final path energy: {final_path_energy:.4f}")
print(f"Final barrier energy: {final_barrier_energy:.4f}")



# plot the new_paths after optimization with scipy in matplotlib
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for i, path in enumerate(new_paths):
    ax.plot(
        [point[0] for point in path],
        [point[1] for point in path],
        [point[2] for point in path],
        label=f'drone{i+1}'
    )
# axis labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
# set axis limits
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(0, 3)
ax.set_box_aspect([4, 4, 3])  # matches x,y,z ranges
ax.legend()
plt.show()










