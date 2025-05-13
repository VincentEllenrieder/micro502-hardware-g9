
import os
import pandas as pd
import numpy as np


def csv_to_waypoints(csv_file_path):
    """
    Convert a CSV file to a list of waypoints.

    Args:
        csv_file_path (str): Path to the CSV file containing 4 gates.
            CSV file example:
                                Gate,x,y,z,theta,size
                                1,0,-0.2,1,0,0.4
                                2,0.5,-0.5,1.5,1.57075,0.4
                                3,1,0,0.8,3.1415,0.4
                                4,-0.5,0.5,1.2,0,0.4

    Returns:
        waypoints (dict): dictionary of waypoints with values 'Gate' and each the following keys:
        'x', 'y', 'z', 'theta', 'size', 'corners', 'normal_vect'
        
        with x,y,z,theta,size as floats, corners as a tuple of 4 array and normal_vect as a tuple.
    """

    # Read the CSV file
    csv_read = pd.read_csv(csv_file_path)
    # Convert the DataFrame to a list of dictionaries
    waypoints = csv_read.to_dict(orient='records')
    
    # init dictionary
    waypoints_dict = {}
    
    # Convert the list of dictionaries to a dictionary with Gate as the key
    for w in waypoints:
        x = float(w['x'])
        y = float(w['y'])
        z = float(w['z'])
        theta = float(w['theta'])
        size = float(w['size'])
        
        # Corners coordinates
        corner_x1 = float(x + np.sin(theta) * size / 2)
        corner_y1 = float(y + np.cos(theta) * size / 2)
        
        corner_x2 = float(x - np.sin(theta) * size / 2)
        corner_y2 = float(y - np.cos(theta) * size / 2)
        
        # 4 corners
        corners = []                                            # with theta = 0 looking from the x axis
        corners.append([corner_x1, corner_y1, float(z - size / 2)])    # bottom right
        corners.append([corner_x1, corner_y1, float(z + size / 2)])    # top right
        corners.append([corner_x2, corner_y2, float(z + size / 2)])    # top left
        corners.append([corner_x2, corner_y2, float(z - size / 2)])    # bottom left
        
        # normal vector from corners
        normal_vect = np.cross(
            np.array(corners[1]) - np.array(corners[0]),
            np.array(corners[2]) - np.array(corners[0])
        )
        # normalizing
        normal_vect = normal_vect / np.linalg.norm(normal_vect)
        # correcting the normal vector to be in the same direction as the y axis
        normal_vect[1] = 0 - normal_vect[1]
        
        # Attributing the values to the dictionary
        waypoints_dict[int(w['Gate'])] = {
            'x': x, 'y': y, 'z': z, 'theta': theta,'size': size,
            'corners': (                # reordered:
                np.round(corners[3],4).tolist(),     # bottom right
                np.round(corners[2],4).tolist(),     # top right
                np.round(corners[1],4).tolist(),     # top left
                np.round(corners[0],4).tolist()),    # bottom left
            'normal_vect': (
                round(float(normal_vect[0]),4),  # x
                round(float(normal_vect[1]),4),  # y
                round(float(normal_vect[2]),4) ) # z
        }
        waypoints = waypoints_dict

    return waypoints


if __name__ == "__main__":
    
    # Example 
    current_dir = os.path.dirname(__file__)
    csv_file_path = os.path.join(current_dir, "csv_files", "gates_info.csv")
    csv_file_path = os.path.join(current_dir, "csv_files", "test.csv")
    
    # Convert the CSV file to waypoints with the function
    waypoints = csv_to_waypoints(csv_file_path)
    
    # Print the waypoints
    for gate, data in waypoints.items():
        print(f"\nGate {gate}:")
        print(f"  x: {data['x']}, y: {data['y']}, z: {data['z']}, theta: {data['theta']}, size: {data['size']}")
        print(f"  corners: {data['corners']}")
        print(f"  normal_vect: {data['normal_vect']}")
    print("\n")