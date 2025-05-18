import numpy as np
from itertools import permutations, product


ANGLE_PENALTY = 1.0                    # Penalty for path choice with high turning angles


def sort_wp_min_energy(wp, gates, gate_ids, gate_to_idx, angle_penalty_weight=ANGLE_PENALTY):
    """
    Finds the most energy-efficient order to visit a series of gates, where:
    - Each gate has 2 normal points (entry/exit) that must be consecutive, but can flip.
    - The order of gates can be permuted.
    - The path starts at either normal point 1 or 2 (1st gate)
    - Energy = distance + angle penalties (to avoid sharp turns).
    
    Args:
        wp (list): Waypoints. List of len(gates_found) tuples, 1 (np.array(entry), np.array(exit)) for each gate.
        gate_ids (list of int): Parallel list to wp, gate_ids[i] is the gate ID for wp[i].
        gate_to_idx (dict): Mapping from gate ID to indices of its normal points in wp.
        angle_penalty_weight (float): Weight applied to angle penalties relative to distance.

    Returns:
        best_path (list): List of np.array waypointspoints representing the optimal path.
        best_indices (list): List of indices (int) representing the updated order of waypoints.
        best_wp_gate_ids (list of int): Gate IDs, parallel to 'best_indices', showing which gate each waypoint belongs to.
        min_cost (float): Total energy cost (distance + angle penalties).
    """

    num_gates = len(wp) // 2  # Number of gates
    gate_indices = [ (2*i, 2*i+1) for i in range(num_gates) ]  # Gate groupings

    best_path = None
    best_indices = None    
    best_wp_gate_ids = None
    min_cost = float('inf')

    first_found_gate = gates[0]
    start_pair = gate_to_idx[first_found_gate]

    for gate_order in permutations(gate_indices):
        for flip_config in product([False, True], repeat=num_gates):
            path = []
            indices = []
            for i, gate in enumerate(gate_order):
                idx1, idx2 = gate
                if flip_config[i]:
                    path.extend([wp[idx2], wp[idx1]])      # Flip order
                    indices.extend([idx2, idx1])
                else:
                    path.extend([wp[idx1], wp[idx2]])      # Normal order
                    indices.extend([idx1, idx2])

            # Ensure starting point is waypoint 0 or 1 (Gate 1's normal points)
            if indices[0] not in start_pair:
                continue

            # Compute distance + angle penalties
            dist = sum(np.linalg.norm(path[i+1] - path[i]) for i in range(len(path)-1))
            angle_penalty = 0
            for i in range(1, len(path)-1):
                vec1 = path[i] - path[i-1]
                vec2 = path[i+1] - path[i]
                if np.linalg.norm(vec1) > 1e-6 and np.linalg.norm(vec2) > 1e-6:
                    cos_theta = np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))
                    angle_penalty += (1 - cos_theta)

            cost = dist + angle_penalty_weight * angle_penalty

            if cost < min_cost:
                min_cost = cost
                best_path = path
                best_indices = indices
                best_wp_gate_ids = [gate_ids[i] for i in indices]

    return best_path, best_indices, best_wp_gate_ids, min_cost