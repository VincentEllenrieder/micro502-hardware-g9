
import os
import pandas as pd
import numpy as np


def csv_to_waypoints(csv_file_path):
    """
    Convert a CSV file to a list of dictionaries of the waypoints.

    Args:
        csv_file_path (str): Path to the CSV file containing 4 gates.
            CSV file example:
                                Gate,x,y,z,theta,size
                                1,0,-1,1,0.0,0.4
                                2,1,0,1.5,1.57075,0.4
                                3,0,1,0.8,3.1415,0.4
                                4,1,0,1.2,4.71225,0.4

    Returns:
        waypoints (dict): dictionary of dictionaries of waypoints with each the following keys:
        'x', 'y', 'z', 'theta', 'size', 'corners', 'normal_vect'

        example (the "[1]" is the name of the gate not necessarily the index):
            waypoints = {
                waypoints[1]: {
                    'x': 0.0, 'y': -1.0, 'z': 1.0,      # as floats
                    'theta': 0.0, 'size': 0.4,          # as floats
                    'corners':                          # as a tuple of 4 arrays    
                        ( [0.0, -1.2, 0.8],
                        [0.0, -1.2, 1.2],
                        [0.0, -0.8, 1.2],
                        [0.0, -0.8, 0.8] ),
                    'normal_vect':                      # as a tuple 
                        (1.0, 0.0, -0.0)
                },
                waypoints[2]: { 
                    ...
    """

    # Read the CSV file
    csv_read = pd.read_csv(csv_file_path)
    # Convert the DataFrame to a list of dictionaries
    waypoints_init = csv_read.to_dict(orient='records')
    
    # init dictionary
    waypoints_simple_planner = {}
    waypoints_motion_planner = []
    
    # Convert the list of dictionaries to a dictionary with Gate as the key
    for w in waypoints_init:
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
        waypoints_simple_planner[int(w['Gate'])] = {
            'x': x, 'y': y, 'z': z, 'theta': theta,'size': size,
            'corners': (                           # reordered:
                np.round(corners[3],4).tolist(),     # bottom right
                np.round(corners[2],4).tolist(),     # top right
                np.round(corners[1],4).tolist(),     # top left
                np.round(corners[0],4).tolist()),    # bottom left
            'normal_vect': (
                round(float(normal_vect[0]),4),  # x
                round(float(normal_vect[1]),4),  # y
                round(float(normal_vect[2]),4) ) # z
        }
        waypoints_motion_planner.append([round(x, 2), round(y, 2), round(z, 2), theta])  # keep theta as rad

    return waypoints_simple_planner, waypoints_motion_planner 


if __name__ == "__main__":
    
    # Example 
    current_dir = os.path.dirname(__file__)
    csv_file_path = os.path.join(current_dir, "csv_files", "gates_info.csv")
    # csv_file_path = os.path.join(current_dir, "csv_files", "test.csv")
    
    # Convert the CSV file to waypoints with the function
    waypoints,motion = csv_to_waypoints(csv_file_path)
    
    # Print the waypoints
    # ATTENTION: les gates sont numérotés selon le nom "Gate" dans le csv donc dans notre cas 1,2,3,4
    # ce sont les "noms" des gates et pas un numéro d'ordre qui commence à 0
    for i in range(1,5):
        print(f"\nGate {i}:")
        print(f"  x,y,z: {waypoints[i]['x']}, {waypoints[i]['y']}, {waypoints[i]['z']}")
        print(f"  theta: {waypoints[i]['theta']}")
        print(f"  size:  {waypoints[i]['size']}")
        print(f"  corners:")
        print(f"   ({waypoints[i]['corners'][0]},")
        print(f"    {waypoints[i]['corners'][1]},")
        print(f"    {waypoints[i]['corners'][2]},")
        print(f"    {waypoints[i]['corners'][3]})")
        print(f"  normal_vect:\n    {waypoints[i]['normal_vect']}")
    print("\n\n")
    
    # Print the type of "waypoints"
    print(f"Type of waypoints: {type(waypoints)}")
    # Print the type of "waypoints[1]"
    print(f"Type of waypoints[1]: {type(waypoints[1])}")
    print("\n")

    print(f"motion planner: {motion}")