import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

OFFSET_GATE = 0.15 


def csv_to_waypoints_simple_planner(csv_file_path):
    # Read the CSV file
    csv_read = pd.read_csv(csv_file_path)
    # Convert the DataFrame to a list of dictionaries
    waypoints_init = csv_read.to_dict(orient='records')
    waypoints_motion_planner = []
    
    # Convert the list of dictionaries to a dictionary with Gate as the key
    for w in waypoints_init:
        x = float(w['x'])
        y = float(w['y'])
        z = float(w['z'])
        theta = float(w['theta']) - np.pi/2  # to have the same orientation as the motion planner
        
        waypoints_motion_planner.append([round(x, 2), round(y, 2), round(z, 2), round(theta,4)])  # keep theta as rad

    return waypoints_motion_planner 


def csv_to_waypoints_motion_planner(csv_file_path):
    global OFFSET_GATE

    # Read the CSV file
    csv_read = pd.read_csv(csv_file_path)
    gates_init = csv_read.to_dict(orient='records')

    # Convert to array with initial yaw (perpendicular to gate)
    gates_uncertain_yaw = []
    for w in gates_init:
        x = float(w['x'])
        y = float(w['y'])
        z = float(w['z'])
        theta = float(w['theta']) - np.pi / 2  # Perpendicular to the gate
        gates_uncertain_yaw.append([x, y, z, theta])

    N = len(gates_uncertain_yaw)
    take_off_coord = [0.0, 0.0, 1.0, gates_uncertain_yaw[0][3]]

    # Create the full list of waypoints for 2 laps plus takeoff and landing
    waypoints2laps = np.zeros((2 * 3 * N + 2, 3))  # Each gate has 3 waypoints (leading, center, trailing) + 2 takeoff points

    # Start point
    waypoints2laps[0] = take_off_coord[0:3]

    def correct_yaw(i):
        """Corrects yaw for gate i based on average of incoming and outgoing direction vectors."""
        p0 = np.array(gates_uncertain_yaw[(i - 1) % N][:2])
        p1 = np.array(gates_uncertain_yaw[i][:2])
        p2 = np.array(gates_uncertain_yaw[(i + 1) % N][:2])

        direction = (p1 - p0) + (p2 - p1)
        if np.linalg.norm(direction) < 1e-6:
            direction = np.array([np.cos(gates_uncertain_yaw[i][3]), np.sin(gates_uncertain_yaw[i][3])])  # fallback

        direction /= np.linalg.norm(direction)

        yaw = gates_uncertain_yaw[i][3]
        yaw_vector = np.array([np.cos(yaw), np.sin(yaw)])

        if np.dot(direction, yaw_vector) < 0:
            yaw += np.pi

        return yaw

    # Generate waypoints for two laps
    for lap in range(2):
        for i in range(N):
            gate_x, gate_y, gate_z, _ = gates_uncertain_yaw[i]
            yaw = correct_yaw(i)

            leading_gate = [
                gate_x - OFFSET_GATE * np.cos(yaw),
                gate_y - OFFSET_GATE * np.sin(yaw),
                gate_z
            ]
            trailing_gate = [
                gate_x + OFFSET_GATE * np.cos(yaw),
                gate_y + OFFSET_GATE * np.sin(yaw),
                gate_z
            ]

            idx = lap * 3 * N + 1 + 3 * i
            waypoints2laps[idx] = leading_gate
            waypoints2laps[idx + 1] = [gate_x, gate_y, gate_z]
            waypoints2laps[idx + 2] = trailing_gate

    # Final return to takeoff position
    waypoints2laps[2 * 3 * N + 1] = take_off_coord[0:3]

    # Plot the waypoints
    plot_waypoints(waypoints2laps, csv_file_path)

    return waypoints2laps, take_off_coord


def plot_waypoints(waypoints, csv_file_path):
    fig = plt.figure(figsize=(18, 6))

    # Read the CSV file
    csv_read = pd.read_csv(csv_file_path)
    gates_init = csv_read.to_dict(orient='records')

    # Convert to array with initial yaw (perpendicular to gate)
    true_gates = []
    sizes = []
    for w in gates_init:
        x = float(w['x'])
        y = float(w['y'])
        z = float(w['z'])
        theta = float(w['theta'])
        size = float(w['size'])
        true_gates.append([x, y, z, theta])
        sizes.append(size)
    
    print("true_gates", true_gates)

    # Unpack coordinates
    waypoint_x, waypoint_y, waypoint_z = zip(*waypoints)

    # Separate points into leading, gate, and trailing
    leading_x, leading_y, leading_z = [], [], []
    gate_x, gate_y, gate_z = [], [], []
    trailing_x, trailing_y, trailing_z = [], [], []

    for i, (x, y, z) in enumerate(waypoints):
        if i == 0 or i == len(waypoints) - 1:
            gate_x.append(x)
            gate_y.append(y)
            gate_z.append(z)
        elif (i % 3) == 1:
            leading_x.append(x)
            leading_y.append(y)
            leading_z.append(z)
        elif (i % 3) == 2:
            gate_x.append(x)
            gate_y.append(y)
            gate_z.append(z)
        elif (i % 3) == 0:
            trailing_x.append(x)
            trailing_y.append(y)
            trailing_z.append(z)

    views = [
        ("Top View", 90, -90),
        ("Front View", 0, -90),
        ("3D View", None, None)
    ]

    def draw_rotated_gate(ax, x, y, z, theta, size):
        # Gate is a vertical square centered at (x, y, z)
        # First, define the corners of the gate in its local YZ frame
        print(size)
        half_size = size / 2
        local_corners = np.array([
            [-half_size, 0, -half_size],
            [ half_size, 0, -half_size],
            [ half_size, 0,  half_size],
            [-half_size, 0,  half_size]
        ])

        # Rotate corners around the Z-axis by yaw
        R = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,            0,           1]
        ])  # shape (3, 3)

        rotated_corners = (R @ local_corners.T).T + np.array([x, y, z])  # shape (4, 3)

        gate_face = Poly3DCollection([rotated_corners], color='green', alpha=0.3)
        ax.add_collection3d(gate_face)

    for i, (title, elev, azim) in enumerate(views, start=1):
        ax = fig.add_subplot(1, 3, i, projection='3d')
        ax.scatter(leading_x, leading_y, leading_z, color='red', label='Leading Gate')
        ax.scatter(gate_x, gate_y, gate_z, color='green', label='Gate')
        ax.scatter(trailing_x, trailing_y, trailing_z, color='blue', label='Trailing Gate')

        # Draw the actual gate faces using the yaw angles from true_gates
        for idx, (x, y, z, theta) in enumerate(true_gates, start=1):
            draw_rotated_gate(ax, x, y, z, theta, sizes[idx-1])
            ax.text(x+0.25, y, z+0.25, f"{idx}", color='black', fontsize=10, weight='bold')

        if elev is not None and azim is not None:
            ax.view_init(elev=elev, azim=azim)

        ax.set_title(title)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim([-3, 3])
        ax.set_ylim([-3, 3])
        ax.set_zlim([0, 3])
        ax.legend()

    plt.tight_layout()
    plt.savefig("demo_waypoints_motion_planner.png")
    plt.close()


if __name__ == "__main__":
    current_dir = os.path.dirname(__file__)
    csv_file_path = os.path.join(current_dir, "demo_gates_info.csv")
    
    waypoints, take_off_coord = csv_to_waypoints_motion_planner(csv_file_path)