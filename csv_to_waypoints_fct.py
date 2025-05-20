
import os
import pandas as pd
import numpy as np


def csv_to_waypoints(csv_file_path):
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


if __name__ == "__main__":
    
    # Example 
    current_dir = os.path.dirname(__file__)
    csv_file_path = os.path.join(current_dir, "csv_files", "gates_info_example.csv")
    
    # Convert the CSV file to waypoints with the function
    waypoints = csv_to_waypoints(csv_file_path)

    print(f"motion planner: {waypoints}")