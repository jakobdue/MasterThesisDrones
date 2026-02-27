
"""
def barrier_force(pos1, pos2, eta_safety_distance, k_stiffness, activation_distance):
    vec, g = quadratic_penalty_g(pos1, pos2, eta_safety_distance)

    if g <= 0:
        print(f'COLLISION IMMINENT! g={g:.2f}')
        val = -(1/2) * k_stiffness * g
        return (vec[0] * val, vec[1] * val, vec[2] * val)

    elif g >= activation_distance:
        print(f"No barrier force. g={g:.2f}")
        return (0, 0, 0)

    else:
        print(f'Barrier active! g={g:.2f}')
        val = -eta_safety_distance * (1/g) 
        return (vec[0] * val, vec[1] * val, vec[2] * val)
         
"""