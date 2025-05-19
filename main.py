# ================================================= #
# Crazyflie Hardware Assignment                     #
# Aerial robotics (MICRO-502)                       #
#                                                   #
#                      Property of the EPFL LIS LAB #
#                              2025 - Fall semester #
# ------------------------------------------------- #
# Assignment done by group 9 :                      #
#                                                   #
#                          Charles Proffit (324624) #
#                           Clément Chalut (326251) #
#                             Cyril Goffin (373937) #
#                          Jeremy Serillon (326033) #
#                       Vincent Ellerieder (329051) #
# ================================================= #

import logging
import time
from threading import Timer
import threading
from pynput import keyboard 
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib
import os
import pandas as pd
from csv_to_waypoints_fct import csv_to_waypoints


# Global variables
URI = uri_helper.uri_from_env(default='radio://0/90/2M/E7E7E7E709')
logging.basicConfig(level=logging.ERROR) # Only output errors from the logging framework

# csv file path
current_dir = os.path.dirname(__file__)
csv_file_path = os.path.join(current_dir, "csv_files", "gates_info.csv")

STATE = {
    "TAKE_OFF": 0,
    "RACING": 1,
    "LANDING": 2,
}
GATE_THRESHOLD = 0.05                           # Threshold for the position check, in meters
DT = 0.01                                     # Time step for the main loop, in seconds
LANDING_COORD = [0, 0, 0, 0]                    # Landing position of the drone, in [m, m, m, rad]
TAKE_OFF_COORD = [0, 0, 1.0,  np.deg2rad(-60)]                 # Take off position of the drone, in [m, m, m, rad]
GATES = [[0.2, -0.35, 1.3, np.deg2rad(-60)],   # [x, y, z, yaw] of each true gate, in [m, m, m, rad]
         [0.8, -0.6, 1.15, np.deg2rad(-1.5)],
         [2.2, 0.3, 1.33, np.deg2rad(120)],
         [-0.4, 0.82, 0.87, np.deg2rad(-123)]]
# SIMPLE_GATES, GATES = csv_to_waypoints(csv_file_path) # Load the gates from the CSV file
# print(SIMPLE_GATES)
print(GATES)
OFFSET_GATE = 0.15                              # Offset to the leading and trailing gate, in meters
RACING_VELOCITY = 1.0                           # Velocity goal during the racing, in m/s
PLANNER_STEP = 10

class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link URI and disconnects after 5s.
    """

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self.sensor_data = {}

        # self.sensor_data['t'] = 0
        self.sensor_data["x"] = 0
        self.sensor_data["y"] = 0
        self.sensor_data["z"] = 0
        self.sensor_data["roll"] = 0
        self.sensor_data["pitch"] = 0
        self.sensor_data["yaw"] = 0
        self.sensor_data["vbat"] = 0

        # Accumulators for smoothing / hand detection 
        self.accumulator_z = [0]*10

        # Boolean states
        self.emergency_stop = False
        self.block_callback = False 

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stabilizer.roll', 'float')
        self._lg_stab.add_variable('stabilizer.pitch', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')
        # The fetch-as argument can be set to FP16 to save space in the log packet
        self._lg_stab.add_variable('pm.vbat', 'FP16')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        # Start a timer to disconnect in 60s
        t = Timer(500, self._cf.close_link) #WTF
        t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        if not(self.block_callback):
            self.block_callback = True  # Prevents the callback from being called again while processing

            # Update the other states
            for name, value in data.items():
                if name == 'stateEstimate.x':
                    self.sensor_data['x'] = value
                if name == 'stateEstimate.y':
                    self.sensor_data['y'] = value
                if name == 'stateEstimate.z':
                    self.sensor_data['z'] = value
                    # Update the accumulator
                    self.accumulator_z.append(value)
                    self.accumulator_z.pop(0)
                if name == 'stabilizer.roll':
                    self.sensor_data['roll'] = value
                if name == 'stabilizer.pitch':
                    self.sensor_data['pitch'] = value
                if name == 'stabilizer.yaw':
                    self.sensor_data['yaw'] = value
                if name == 'pm.vbat':
                    self.sensor_data['vbat'] = value

            self.block_callback = False

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False


class MotionPlanner3D():
    def __init__(self, path, obstacles):
        # Inputs:
        # - start: The sequence of input path waypoints2laps provided by the path-planner, including the start and final goal position: Vector of m waypoints2laps, consisting of a tuple with three reference positions each as provided by AStar 
        # - obstacles: 2D array with obstacle locations and obstacle widths [x, y, z, dx, dy, dz]*n_obs
        # - bounds: The bounds of the environment [x_min, x_max, y_min, y_max, z_min, z_max]
        # - grid_size: The grid size of the environment (scalar)
        # - goal: The final goal position of the drone (tuple of 3) 
        self.path = path
        self.trajectory_setpoints = None
        self.init_params(self.path)
        self.run_planner(obstacles, self.path)

    def run_planner(self, obs, path_waypoints):    
        # Run the subsequent functions to compute the polynomial coefficients and extract and visualize the trajectory setpoints
        poly_coeffs = self.compute_poly_coefficients(path_waypoints)
        self.trajectory_setpoints, self.time_setpoints = self.poly_setpoint_extraction(poly_coeffs, obs, path_waypoints)

    def init_params(self, path_waypoints):
        global PLANNER_STEP, RACING_VELOCITY
        # Inputs:
        # - path_waypoints: The sequence of input path waypoints2laps provided by the path-planner, including the start and final goal position: Vector of m waypoints2laps, consisting of a tuple with three reference positions each as provided by AStar

        # TUNE THE FOLLOWING PARAMETERS (PART 2) ----------------------------------------------------------------- ##
        self.disc_steps = PLANNER_STEP #Integer number steps to divide every path segment into to provide the reference positions for PID control # IDEAL: Between 10 and 20
        self.vel_lim = 7.0 #Velocity limit of the drone (m/s)
        self.acc_lim = 50.0 #Acceleration limit of the drone (m/s²)

        distances = np.zeros(len(path_waypoints))

        for i in range(len(path_waypoints)-1):
            distances[i+1] = np.linalg.norm(np.array(path_waypoints[i]) - np.array(path_waypoints[i+1]))
        
        self.times = distances/RACING_VELOCITY
        

        for i in range(len(self.times)-1):
            self.times[i+1] = self.times[i] + self.times[i+1]
                
    def compute_poly_matrix(self, t):
        # Inputs:
        # - t: The time of evaluation of the A matrix (t=0 at the start of a path segment, else t >= 0) [Scalar]
        # Outputs: 
        # - The constraint matrix "A_m(t)" [5 x 6]
        # The "A_m" matrix is used to represent the system of equations [x, \dot{x}, \ddot{x}, \dddot{x}, \ddddot{x}]^T  = A_m(t) * poly_coeffs (where poly_coeffs = [c_0, c_1, c_2, c_3, c_4, c_5]^T and represents the unknown polynomial coefficients for one segment)
        A_m = np.zeros((5,6))
        A_m = np.array([
            [t**5, t**4, t**3, t**2, t, 1], #pos
            [5*(t**4), 4*(t**3), 3*(t**2), 2*t, 1, 0], #vel
            [20*(t**3), 12*(t**2), 6*t, 2, 0, 0], #acc  
            [60*(t**2), 24*t, 6, 0, 0, 0], #jerk
            [120*t, 24, 0, 0, 0, 0] #snap
        ])
        return A_m

    def compute_poly_coefficients(self, path_waypoints):
        
        # Computes a minimum jerk trajectory given time and position waypoints2laps.
        # Inputs:
        # - path_waypoints: The sequence of input path waypoints2laps provided by the path-planner, including the start and final goal position: Vector of m waypoints2laps, consisting of a tuple with three reference positions each as provided by AStar
        # Outputs:
        # - poly_coeffs: The polynomial coefficients for each segment of the path [6(m-1) x 3]

        # Use the following variables and the class function self.compute_poly_matrix(t) to solve for the polynomial coefficients
        
        seg_times = np.diff(self.times) #The time taken to complete each path segment
        m = len(path_waypoints) #Number of path waypoints2laps (including start and end)
        poly_coeffs = np.zeros((6*(m-1),3))

        for dim in range(3):  # Compute for x, y, and z separately
            A = np.zeros((6*(m-1), 6*(m-1)))
            b = np.zeros(6*(m-1))
            pos = np.array([p[dim] for p in path_waypoints])
            A_0 = self.compute_poly_matrix(0) # A_0 gives the constraint factor matrix A_m for any segment at t=0, this is valid for the starting conditions at every path segment
            row = 0
            for i in range(m-1):
                pos_0 = pos[i] #Starting position of the segment
                pos_f = pos[i+1] #Final position of the segment
                # The prescribed zero velocity (v) and acceleration (a) values at the start and goal position of the entire path
                v_0, a_0 = 0, 0
                v_f, a_f = 0, 0
                A_f = self.compute_poly_matrix(seg_times[i]) # A_f gives the constraint factor matrix A_m for a segment i at its relative end time t=seg_times[i]
                if i == 0: # First path segment
                #     # 1. Implement the initial constraints here for the first segment using A_0
                #     # 2. Implement the final position and the continuity constraints for velocity, acceleration, snap and jerk at the end of the first segment here using A_0 and A_f (check hints in the exercise description)
                    A[row, i*6:(i+1)*6] = A_0[0] #Initial position constraint
                    b[row] = pos_0
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[0] #Final position constraint
                    b[row] = pos_f
                    row += 1
                    A[row, i*6:(i+1)*6] = A_0[1] #Initial velocity constraint
                    b[row] = v_0
                    row += 1
                    A[row, i*6:(i+1)*6] = A_0[2] #Initial acceleration constraint
                    b[row] = a_0
                    row += 1
                    #Continuity of velocity, acceleration, jerk, snap
                    A[row:row+4, i*6:(i+1)*6] = A_f[1:]
                    A[row:row+4, (i+1)*6:(i+2)*6] = -A_0[1:]
                    b[row:row+4] = np.zeros(4)
                    row += 4
                elif i < m-2: # Intermediate path segments
                #     # 1. Similarly, implement the initial and final position constraints here for each intermediate path segment
                #     # 2. Similarly, implement the end of the continuity constraints for velocity, acceleration, snap and jerk at the end of each intermediate segment here using A_0 and A_f
                    A[row, i*6:(i+1)*6] = A_0[0] #Initial position constraint
                    b[row] = pos_0
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[0] #Final position constraint
                    b[row] = pos_f
                    row += 1
                    #Continuity of velocity, acceleration, jerk and snap
                    A[row:row+4, i*6:(i+1)*6] = A_f[1:]
                    A[row:row+4, (i+1)*6:(i+2)*6] = -A_0[1:]
                    b[row:row+4] = np.zeros(4)
                    row += 4
                elif i == m-2: #Final path segment
                #     # 1. Implement the initial and final position, velocity and accelerations constraints here for the final path segment using A_0 and A_f
                    A[row, i*6:(i+1)*6] = A_0[0] #Initial position constraint
                    b[row] = pos_0
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[0] #Final position constraint
                    b[row] = pos_f
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[1] #Final velocity constraint
                    b[row] = v_f
                    row += 1
                    A[row, i*6:(i+1)*6] = A_f[2] #Final acceleration constraint
                    b[row] = a_f
                    row += 1
            # Solve for the polynomial coefficients for the dimension dim

            poly_coeffs[:,dim] = np.linalg.solve(A, b)   
        return poly_coeffs

    def poly_setpoint_extraction(self, poly_coeffs, obs, path_waypoints):
        nb_segments = len(self.times) - 1 # No sure of that but that solves the problem
        self.disc_steps -= 1 # The number of discrete steps is the number of segments - 1
        seg_intervals = (self.disc_steps*nb_segments) + 1

        # Initialize the setpoint arrays
        x_vals, y_vals, z_vals = np.zeros((seg_intervals, 1)), np.zeros((seg_intervals, 1)), np.zeros((seg_intervals, 1))
        v_x_vals, v_y_vals, v_z_vals = np.zeros((seg_intervals, 1)), np.zeros((seg_intervals, 1)), np.zeros((seg_intervals, 1))
        a_x_vals, a_y_vals, a_z_vals = np.zeros((seg_intervals, 1)), np.zeros((seg_intervals, 1)), np.zeros((seg_intervals, 1))
        yaw_vals = np.zeros((seg_intervals, 1))

        # Initialize the time setpoints
        time_setpoints = np.zeros(seg_intervals)

        for i in range(len(self.times)-1):
            time_setpoints[i*self.disc_steps:(i+1)*self.disc_steps] = np.linspace(self.times[i], self.times[i+1], self.disc_steps, endpoint=False)
        
        time_setpoints[-1] = self.times[-1] # Add the last time setpoint

        # Extract the x,y and z direction polynomial coefficient vectors
        coeff_x = poly_coeffs[:,0]
        coeff_y = poly_coeffs[:,1]
        coeff_z = poly_coeffs[:,2]

        for i,t in enumerate(time_setpoints):
            seg_idx = min(max(np.searchsorted(self.times, t)-1,0), len(coeff_x) - 1)
            # Determine the x,y and z position reference points at every refernce time
            x_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[0],coeff_x[seg_idx*6:(seg_idx+1)*6])
            y_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[0],coeff_y[seg_idx*6:(seg_idx+1)*6])
            z_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[0],coeff_z[seg_idx*6:(seg_idx+1)*6])
            # Determine the x,y and z velocities at every reference time
            v_x_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[1],coeff_x[seg_idx*6:(seg_idx+1)*6])
            v_y_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[1],coeff_y[seg_idx*6:(seg_idx+1)*6])
            v_z_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[1],coeff_z[seg_idx*6:(seg_idx+1)*6])
            # Determine the x,y and z accelerations at every reference time
            a_x_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[2],coeff_x[seg_idx*6:(seg_idx+1)*6])
            a_y_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[2],coeff_y[seg_idx*6:(seg_idx+1)*6])
            a_z_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[2],coeff_z[seg_idx*6:(seg_idx+1)*6])

        for i in range(len(time_setpoints)-1):
            # Compute the angle (yaw) between successive setpoints
            yaw_vals[i,:] = np.arctan2(y_vals[i + 1,:] - y_vals[i,:], x_vals[i + 1,:] - x_vals[i,:])
                    
        # Last yaw value
        yaw_vals[-1,:] = yaw_vals[-2,:]

        trajectory_setpoints = np.hstack((x_vals, y_vals, z_vals, yaw_vals))

        self.plot(obs, path_waypoints, trajectory_setpoints)
        self.plot_with_yaw(obs, path_waypoints, trajectory_setpoints)
            
        # Find the maximum absolute velocity during the segment
        vel_max = np.max(np.sqrt(v_x_vals**2 + v_y_vals**2 + v_z_vals**2))
        vel_mean = np.mean(np.sqrt(v_x_vals**2 + v_y_vals**2 + v_z_vals**2))
        acc_max = np.max(np.sqrt(a_x_vals**2 + a_y_vals**2 + a_z_vals**2))
        acc_mean = np.mean(np.sqrt(a_x_vals**2 + a_y_vals**2 + a_z_vals**2))

        # print("Maximum flight speed: " + str(vel_max))
        # print("Average flight speed: " + str(vel_mean))
        # print("Average flight acceleration: " + str(acc_mean))
        # print("Maximum flight acceleration: " + str(acc_max))
        
        # Check that it is less than an upper limit velocity v_lim
        assert vel_max <= self.vel_lim, "The drone velocity exceeds the limit velocity : " + str(vel_max) + " m/s"
        assert acc_max <= self.acc_lim, "The drone acceleration exceeds the limit acceleration : " + str(acc_max) + " m/s²"

        return trajectory_setpoints, time_setpoints
    
    def plot(self, obs, path_waypoints, trajectory_setpoints):
        # trajectory_setpoints = np.hstack((x_vals, y_vals, z_vals, yaw_vals))
        # Plot 3D trajectory
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection='3d')

        ax.plot(trajectory_setpoints[:,0], trajectory_setpoints[:,1], trajectory_setpoints[:,2], label="Minimum-Jerk Trajectory", linewidth=2)
        ax.set_xlim(-2.5, 2.5)
        ax.set_ylim(-2.5, 2.5)
        ax.set_zlim(0, 2.5)

        # Plot waypoints2laps
        waypoints_x = [p[0] for p in path_waypoints]
        waypoints_y = [p[1] for p in path_waypoints]
        waypoints_z = [p[2] for p in path_waypoints]
        ax.scatter(waypoints_x, waypoints_y, waypoints_z, color='red', marker='o', label="Waypoints")

        # Labels and legend
        ax.set_xlabel("X Position")
        ax.set_ylabel("Y Position")
        ax.set_zlabel("Z Position")
        ax.set_title("3D Motion planning trajectories")
        ax.legend()
        plt.savefig("plot_trajectoy.png")
        plt.close()
    
    def plot_with_yaw(self, obs, path_waypoints, trajectory_setpoints):
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection='3d')

        # Trajectory positions
        x = trajectory_setpoints[:, 0]
        y = trajectory_setpoints[:, 1]
        z = trajectory_setpoints[:, 2]
        yaw = trajectory_setpoints[:, 3]  # Assuming last column is yaw in radians

        # Draw yaw direction vectors at each point
        length = 0.2  # Arrow length
        u = np.cos(yaw) * length  # X component
        v = np.sin(yaw) * length  # Y component
        w = np.zeros_like(yaw)    # Z component (arrows in X-Y plane only)

        ax.quiver(x, y, z, u, v, w, label='Yaw Direction',
                   length=2.0, normalize=False, arrow_length_ratio=0.1, linewidth=1, color='black', alpha=0.4)
        ax.plot(x, y, z, label="Minimum-Jerk Trajectory", linewidth=2)


        ax.set_xlim(-2.5, 2.5)
        ax.set_ylim(-2.5, 2.5)
        ax.set_zlim(0, 2.5)

        # Waypoints
        waypoints_x = [p[0] for p in path_waypoints]
        waypoints_y = [p[1] for p in path_waypoints]
        waypoints_z = [p[2] for p in path_waypoints]
        ax.scatter(waypoints_x, waypoints_y, waypoints_z, color='red', marker='o', label="Waypoints")

        ax.set_xlabel("X Position")
        ax.set_ylabel("Y Position")
        ax.set_zlabel("Z Position")
        ax.set_title("3D Motion Planning Trajectories")
        ax.legend()
        
        plt.savefig("plot_trajectory_with_yaw.png")
        #plt.show()
        plt.close()


def emergency_stop_callback(le):
    cf = le._cf  # Access the Crazyflie instance from the LoggingExample
    def on_press(key):
        try:
            if key.char == 'q':  # Check if the "space" key is pressed
                print("Emergency stop triggered!")
                le.emergency_stop = True
                return False     # Stop the listener
        except AttributeError:
            pass

    # Start listening for key presses
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

    # Send the stop setpoint to the Crazyflie if the emergency stop is triggered
    if le.emergency_stop:
        cf.commander.send_stop_setpoint()
        cf.close_link()
    

def at_position(current_pos, desired_pos):
    global GATE_THRESHOLD
    if np.linalg.norm(np.array(current_pos) - np.array(desired_pos)) < GATE_THRESHOLD:
        return True
    else:   
        return False


def plot_waypoints(waypoints):
    global GATES
    fig = plt.figure(figsize=(18, 6))  # Wider to fit 3 subplots

    # Unpack coordinates
    det_x, det_y, det_z, det_yaw = zip(*GATES)
    waypoint_x, waypoint_y, waypoint_z = zip(*waypoints)

    # --- Top View (XY) ---
    ax1 = fig.add_subplot(1, 3, 1, projection='3d')
    ax1.scatter(det_x, det_y, det_z, color='red', label='Gates')
    ax1.scatter(waypoint_x, waypoint_y, waypoint_z, color='green', label='Waypoints')
    ax1.view_init(elev=90, azim=-90)  # Top-down
    ax1.set_title("Top View")
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.legend()

    # --- Front View (XZ) ---
    ax2 = fig.add_subplot(1, 3, 2, projection='3d')
    ax2.scatter(det_x, det_y, det_z, color='red', label='Gates')
    ax2.scatter(waypoint_x, waypoint_y, waypoint_z, color='green', label='Waypoints')
    ax2.view_init(elev=0, azim=-90)  # Front view
    ax2.set_title("Front View")
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.legend()

    # --- 3D Perspective View ---
    ax3 = fig.add_subplot(1, 3, 3, projection='3d')
    ax3.scatter(det_x, det_y, det_z, color='red', label='Gates')
    ax3.scatter(waypoint_x, waypoint_y, waypoint_z, color='green', label='Waypoints')
    # No view_init → default perspective
    ax3.set_title("3D View")
    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')
    ax3.set_zlabel('Z')
    ax3.legend()

    plt.tight_layout()
    plt.savefig("plot_gates_with_waypoints.png")
    plt.close()


def create_trajectory(waypoints2laps):
    global GATES, TAKE_OFF_COORD, OFFSET_GATE
    # Create the waypoints
    N = len(GATES)
    # Start
    waypoints2laps[0] = TAKE_OFF_COORD[0:3]  # Start of lap 1
    # First lap
    for i in range(N):
        gate_x, gate_y, gate_z, yaw = GATES[i]
        leading_gate = [gate_x - OFFSET_GATE*np.cos(yaw), gate_y - OFFSET_GATE*np.sin(yaw), gate_z]
        trailing_gate = [gate_x + OFFSET_GATE*np.cos(yaw), gate_y + OFFSET_GATE*np.sin(yaw), gate_z]

        waypoints2laps[3*i + 1] = leading_gate
        waypoints2laps[3*i + 2] = [gate_x, gate_y, gate_z]
        waypoints2laps[3*i + 3] = trailing_gate
    # Second lap
    for i in range(N):
        gate_x, gate_y, gate_z, yaw = GATES[i]
        leading_gate = [gate_x - OFFSET_GATE*np.cos(yaw), gate_y - OFFSET_GATE*np.sin(yaw), gate_z]
        trailing_gate = [gate_x + OFFSET_GATE*np.cos(yaw), gate_y + OFFSET_GATE*np.sin(yaw), gate_z]

        idx = 3*N + 1 + 3*i
        waypoints2laps[idx] = leading_gate
        waypoints2laps[idx + 1] = [gate_x, gate_y, gate_z]
        waypoints2laps[idx + 2] = trailing_gate
    # End
    waypoints2laps[2*3*N + 1] = TAKE_OFF_COORD[0:3]  # End of lap 2

    print("Waypoints2laps: ", waypoints2laps)
    plot_waypoints(waypoints2laps)

    motion_planner = MotionPlanner3D(waypoints2laps, [])
    setpoints = motion_planner.trajectory_setpoints
    time_setpoints = motion_planner.time_setpoints

    return setpoints, time_setpoints


def trajectory_tracking(timer, index_current_setpoint, setpoints, time_setpoints):
    global DT
    if timer is None:
        # Begin timer and start trajectory
        timer = 0
        index_current_setpoint = 1
    else:
        timer += DT

    #print(f"Timer: {timer:.2f}, Index: {index_current_setpoint}")

    # Determine the current setpoint based on the time
    if timer is not None:
        if index_current_setpoint < len(time_setpoints) - 1:
            # Update new setpoint
            if timer >= time_setpoints[index_current_setpoint]:
                index_current_setpoint += 1
            current_setpoint = setpoints[index_current_setpoint,:]
        else:
            # Hover at the final setpoint
            current_setpoint = setpoints[-1]
    return current_setpoint, timer, index_current_setpoint


if __name__ == "__main__":
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    le = LoggingExample(URI)
    cf = le._cf

    # Reset the Kalman filter
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

    # Emergency stop thread
    emergency_stop_thread = threading.Thread(target=emergency_stop_callback, args=(le,))
    emergency_stop_thread.start()

    # Local variables
    state = STATE["TAKE_OFF"]
    take_off_reached = False
    waypoint_index = 0
    waypoints2laps = np.zeros((26, 3)) # 2 * 4 gates with leading and trailing gates (24 points) + take off position in between at start and end (2 points)
    timer = None
    index_current_setpoint = None
    setpoints = None
    time_setpoints = None

    # Creating the trajectory for racing
    setpoints, time_setpoints = create_trajectory(waypoints2laps)

    while le.is_connected:
        time_start = time.time()

        # Get the current state of the drone
        current_pos = [le.sensor_data['x'], le.sensor_data['y'], le.sensor_data['z']]
        current_orientation= [le.sensor_data['roll'], le.sensor_data['pitch'], le.sensor_data['yaw']]
        vbat = le.sensor_data['vbat']
        # print(f"X: {current_pos[0]:.2f}, Y: {current_pos[1]:.2f}, Z: {current_pos[2]:.2f}, "f"Roll: {current_orientation[0]:.2f}, Pitch: {current_orientation[1]:.2f}, Yaw: {current_orientation[2]:.2f}, "f"VBat: {vbat:.2f}")

        # Send position setpoint based on the current state     
        if state == STATE["TAKE_OFF"]:
            if at_position(current_pos, TAKE_OFF_COORD[0:3]):
                state = STATE["RACING"]
                print("Take-off complete. Transitioning to racing.")
            else:
                cf.commander.send_position_setpoint(TAKE_OFF_COORD[0], TAKE_OFF_COORD[1], TAKE_OFF_COORD[2], TAKE_OFF_COORD[3])
                print("Taking off...")
            
        elif state == STATE["RACING"]:
            control_command, timer, index_current_setpoint = trajectory_tracking(timer, index_current_setpoint, setpoints, time_setpoints)
            print(f"Control command: {control_command}")
            cf.commander.send_position_setpoint(control_command[0], control_command[1], control_command[2], np.rad2deg(control_command[3]))
            print("Racing...")

        elif state == STATE["LANDING"]:
            if at_position(current_pos, LANDING_COORD[0:3]):
                print("Landing complete.")
                cf.commander.send_stop_setpoint()
                break
            else:
                print("Landing...")
                cf.commander.send_position_setpoint(LANDING_COORD[0], LANDING_COORD[1], LANDING_COORD[2], LANDING_COORD[3])

        # Sleep to respect the desired loop time
        time_end = time.time()
        print(f"Time taken for loop: {time_end - time_start:.4f} seconds")
        if time_end - time_start < DT:
            time.sleep(DT - (time_end - time_start))
