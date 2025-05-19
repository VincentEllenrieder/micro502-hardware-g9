import numpy as np
from itertools import permutations, product
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


N_GATES = 4                             # Number of gates
GATES_DATA = {
    f"GATE{i+1}": {
        "centroid": None,               # Centroid position, ndarray of shape (3,)
        "corners": [],                  # Corner positions, list of 4 ndarrays of shape (3,)
        "normal points": [],            # Normal points on each side of the gate, list of 2 ndarrays of shape (3,)
        "theta": None,                  # Orientation angle, float
        "size": None,                   # Size of the gate, float
    }
    for i in range(N_GATES)
}
DISTANCE_FROM_GATE = 0.15               # Distance from the normal point to the centroid of the gate
ANGLE_PENALTY = 1.0                     # Penalty for path choice with high turning angles
X_START, Y_START, Z_START = 0.0, 0.0, 1.0

def csv_to_waypoints(csv_file_path):
    """
    Convert a CSV file to a list of dictionary of gates data and waypoints list.

    """

    # Read the CSV file
    csv_read = pd.read_csv(csv_file_path)
    # Convert the DataFrame to a list of dictionaries
    waypoints_init = csv_read.to_dict(orient='records')
    
    # init dictionary
    wp = []
    gate_ids = []
    
    # Convert the list of dictionaries to a dictionary with Gate as the key
    for w in waypoints_init:
        x = float(w['x'])
        y = float(w['y'])
        z = float(w['z'])
        theta = float(w['theta'])
        size = float(w['size'])

        # Centroid position
        centroid = np.array([x, y, z])
        
        # Corners coordinates
        corner_x1 = float(x + np.cos(theta) * size / 2)
        corner_y1 = float(y + np.sin(theta) * size / 2)
        
        corner_x2 = float(x - np.cos(theta) * size / 2)
        corner_y2 = float(y - np.sin(theta) * size / 2)
        
        # 4 corners
        corners = []                                                             # with theta = 0 is when the gate is aligned with the x axis
        corners.append(np.array([corner_x1, corner_y1, float(z - size / 2)]))    # bottom left
        corners.append(np.array([corner_x1, corner_y1, float(z + size / 2)]))    # top left
        corners.append(np.array([corner_x2, corner_y2, float(z + size / 2)]))    # top right
        corners.append(np.array([corner_x2, corner_y2, float(z - size / 2)]))    # bottom right

        # normal vector from corners
        normal_vect = np.cross(
            np.array(corners[1]) - np.array(corners[0]),
            np.array(corners[3]) - np.array(corners[0])
        )
        # normalizing
        normal_vect = normal_vect / np.linalg.norm(normal_vect)

        # normal points on each side of the gate
        np1 = centroid + DISTANCE_FROM_GATE * normal_vect
        np2 = centroid - DISTANCE_FROM_GATE * normal_vect
        
        # Attributing the values to the global dictionary
        idx = int(w['Gate'])
        GATES_DATA[f"GATE{idx}"] = {
            'centroid': centroid,                 # centroid position
            'corners': corners,                   # corners positions
            'normal points': (np1, np2),          # normal points on each side of the gate
            'theta': theta,                       # orientation angle
            'size': size,                         # size of the gate
        }

        # Extend the waypoints list
        wp.extend([np1, np2])
        gate_ids.extend([idx, idx])

    # print("GATES_DATA:", GATES_DATA)
    # print("Unsorted waypoints:", wp)
    # print("Unsorted gate_ids:", gate_ids)

    return wp, gate_ids

def sort_wp_min_energy(wp, gate_ids, angle_penalty_weight=ANGLE_PENALTY):
    """
    Finds the most energy-efficient order to visit a series of gates, where:
    - Each gate has 2 normal points (entry/exit) that must be consecutive, but can flip.
    - The order of gates can be permuted.
    - The path starts at either normal point 1 or 2 (1st gate)
    - Energy = distance + angle penalties (to avoid sharp turns).
    
    Args:
        wp (list): Waypoints. List of N_GATES tuples, 1 (np.array(normal_point_1), np.array(normal_point_2)) for each gate.
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

    start_pair = (0, 1)  # Starting pair of indices (normal points of the first gate)

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

def visualize_gates(GATES_DATA, best_path=None):
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot map plane
    xx = [0, 8, 8, 0, 0]
    yy = [0, 0, 8, 8, 0]
    zz = [0]*5
    ax.plot(xx, yy, zz, color='gray', linestyle='--', label='Map Boundary')

    # Legend flags
    plotted = {"centroid": False, "corner": False, "normal": False}

    for i, (key, data) in enumerate(GATES_DATA.items(), start=1):
        centroid = data['centroid']
        corners = data['corners']
        normal_pts = data['normal points']

        # Plot centroid
        if centroid is not None:
            ax.scatter(*centroid, color='magenta', s=30, label='Centroid' if not plotted["centroid"] else "")
            ax.text(*centroid, f"{i}", color='magenta', fontsize=8, ha='left')
            plotted["centroid"] = True

        # Plot corners
        for corner in corners:
            ax.scatter(*corner, color='blue', s=10, label='Corner' if not plotted["corner"] else "")
        plotted["corner"] = True

        # Plot normal points
        for np_pt in normal_pts:
            ax.scatter(*np_pt, color='green', s=10, label='Normal point' if not plotted["normal"] else "")
        plotted["normal"] = True

    # Plot best_path if provided
    if best_path is not None:
        for idx, wp in enumerate(best_path):
            ax.scatter(*wp, color='red', s=15, label='Optimized path' if idx == 0 else "")
            ax.text(*wp, str(idx + 1), color='black', fontsize=7, ha='center')

    # Axes labels and limits
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_xlim(0, 8)
    ax.set_ylim(0, 8)
    ax.set_zlim(-0.5, 2.0)  # allow grid below zero

    # Grid: force full visibility
    ax.grid(True)

    # Clean legend
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys())

    plt.tight_layout()
    plt.show()

def extract_best_path(csv_path="gates_info_example.csv"):
    """
    Extract the best path from the CSV file that minimizes sharp turns + visualize the path and the gates data.

    """

    wp, gate_ids = csv_to_waypoints(csv_path)    
    best_wp_order, best_wp_indices, best_gate_ids_order, min_cost = sort_wp_min_energy(wp, gate_ids)

    # visualize_gates(GATES_DATA, best_wp_order)
    # print("Best path:", best_wp_order)
    # print("Best indices:", best_wp_indices)
    # print("Best gate IDs order:", best_gate_ids_order)
    # print("Minimum cost:", min_cost)

    best_wp_order_with_centroids = []
    for i in range(0, len(best_wp_order), 2):
        np1 = best_wp_order[i]
        np2 = best_wp_order[i+1]
        gate_id = best_gate_ids_order[i]      # Get the gate id of the first sorted normal point

        best_wp_order_with_centroids.append(np1)

        centroid = GATES_DATA[f"GATE{gate_id}"]["centroid"]
        best_wp_order_with_centroids.append(centroid)

        best_wp_order_with_centroids.append(np2)

    best_path = best_wp_order_with_centroids
    best_path.insert(0, np.array([X_START, Y_START, Z_START])) # add takeoff point at beginning of the path
    best_path.extend(best_wp_order_with_centroids)   # add second lap
    best_path.extend([np.array([X_START, Y_START, Z_START])])  # add final point at the end of the path

    for i in range(len(best_path)):
        best_path[i] = best_path[i].tolist()  # Convert to list for easier manipulation

    return best_path

if __name__ == "__main__":
    best_path = extract_best_path()
    print("Best path with centroids:", best_path)
