
import os
import pandas as pd


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
        waypoints (dict): list of waypoints (Gate,x,y,z,theta,size)
    """

    # Read the CSV file
    df = pd.read_csv(csv_file_path)

    # Convert the DataFrame to a list of dictionaries
    waypoints = df.to_dict(orient='records')
    
    # Convert the list of dictionaries to a list of tuples
    waypoints_tuples = []
    for point in waypoints:
        waypoints_tuples.append((int(point['Gate']), float(point['x']), float(point['y']), 
                                 float(point['z']), float(point['theta']), float(point['size'])))
    waypoints = waypoints_tuples

    return waypoints


if __name__ == "__main__":
    
    # Example 
    current_dir = os.path.dirname(__file__)
    csv_file_path = os.path.join(current_dir, "csv_files", "gates_info.csv")
    
    waypoints = csv_to_waypoints(csv_file_path)
    for i in range(len(waypoints)):
        print(f"Gate {i+1}:")
        print(f"  x: {waypoints[i][1]}")
        print(f"  y: {waypoints[i][2]}")
        print(f"  z: {waypoints[i][3]}")
        print(f"  theta: {waypoints[i][4]}")
        print(f"  size: {waypoints[i][5]}")