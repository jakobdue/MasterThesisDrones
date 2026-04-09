import math

# constants: 
eta_safety_distance = 0.4
T = 20

type Point = tuple[float, float, float]

type Path  = list[Point] # points is a list of tuples with (x,y,z) coordinates.

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
    dx, dy, dz = sub_vecs(pos2, pos1)
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
            total_path_length_energy += squared_vec_length(sub_vecs(path[k+1], path[k]))
    
    total_barrier_energy = 0.0
    for k in range(T):
        for i in range(len(paths)): 
            for j in range(i+1, len(paths)): 
                (dx, dy, dz) = sub_vecs(paths[i][k], paths[j][k])

                distance = vec_length((dx, dy, dz))
                total_barrier_energy += p_eps(distance, eta_safety_distance, 3)
         
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
                    2,
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


     
    




# interpolate points in the paths to have T points in total. there is always two points in the path, so we can interpolate T-2 points between them.
def interpolate_path(path: Path) -> Path:
    if len(path) < 2:
        raise ValueError("Path must have at least 2 points to interpolate.")
    
    start, end = path[0], path[1]
    interpolated_path = []
    
    for t in range(T):
        alpha = t / (T - 1)  # alpha goes from 0 to 1
        interpolated_point = (
            start[0] + alpha * (end[0] - start[0]),
            start[1] + alpha * (end[1] - start[1]),
            start[2] + alpha * (end[2] - start[2])
        )
        interpolated_path.append(interpolated_point)
    
    return interpolated_path
        

def barrier_path_optimizer(paths: list[Path], alpha: float, iterations: int) -> list[Path]:
    for _ in range(iterations):
        prev_paths = paths.copy()
        prev_path_energy, prev_barrie_energy = total_path_energy(paths)

        grad_smooth, grad_barrier = gradient_barrier_energy(paths)

        for i in range(len(paths)):
            for k in range(1,T-1):
                # update the path points using gradient descent
                paths[i][k] = add_vecs(paths[i][k], scale_vec(-alpha, add_vecs(grad_smooth[i][k], grad_barrier[i][k])))
        
        new_path_energy, new_barrie_energy = total_path_energy(paths)
        # look at the difference in total energy and look for change, if there is not so much change in the energy, we can stop the optimization early.
        if abs(prev_path_energy - new_path_energy) < 1e-6 and abs(prev_barrie_energy - new_barrie_energy) < 1e-6:
            break
    
    return paths








drone1: Path = interpolate_path([(-1, -1, 0), (1, 1, 0)])
drone2: Path = interpolate_path([(-1, 1, 0.0), (1, -1, 0.0)])
drone3: Path = interpolate_path([(1, -1, 0.0), (-1, 1, 0.0)])
drone4: Path = interpolate_path([(1, 1, 0.0), (-1, -1, 0.0)])



#plot a graph of the paths of the drones in 3D space using matplotlib
import matplotlib.pyplot as plt
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot([point[0] for point in drone1], [point[1] for point in drone1], [point[2] for point in drone1], label='drone1')
ax.plot([point[0] for point in drone2], [point[1] for point in drone2], [point[2] for point in drone2], label='drone2')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.show()






drones: list[Path] = [drone1, 
                      drone2,
                      drone3,
                      drone4]

print(total_path_energy(drones))

new_paths = barrier_path_optimizer(drones, alpha=0.01, iterations=1000)

# print the new paths after optimization in matplotlib
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for i, path in enumerate(new_paths):
    ax.plot([point[0] for point in path], [point[1] for point in path], [point[2] for point in path], label=f'drone{i+1}')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.show()

