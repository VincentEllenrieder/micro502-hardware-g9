# ================================================= #
# Crazyflie Hardware Assignment                     #
# Aerial robotics (MICRO-502)                       #
#                                                   #
#                      Property of the EPFL LIS LAB #
#                              2025 - Fall semester #
#                                                   #
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

class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
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
        t = Timer(120, self._cf.close_link) #WTF
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


# def TransitionToPhase(phase_name):
#     """
#     Transition to a new phase of the project while setting the phase_transition flag.
    
#     Parameters:
#     - phase_name: The name of the new phase to transition to. It can be
#                   "takeoff", "wait_go", "speed_run" or "end".
#     """
    
#     global phase, phase_transition
    
#     # Check if valid phase name
#     if phase_name not in ["takeoff", "wait_go", "speed_run", "end"]:
#         print("\n.")
#         raise ValueError("Invalid phase name. Cannot transition to phase: " + phase_name)

#     if VERBOSE:
#         print(f"  - Transitioning from '{phase}' to '{phase_name}'")
#     phase = phase_name
#     phase_transition = True

# def get_command(arg1, arg2, arg3, arg4, dt):
#     """
#     This function holds the logic for the different phases of the simulation.
#     The function is called every simulation step and should return the control command for the drone.
    
#     Phases:
#      - 0: "takeoff"
#      - 1: "wait_go"
#      - 2: "speed_run"
#      - 3: "end"
#     """
    
#     global phase, phase_transition
    
#     x,    y,     z   = 0.0 , 0.0 , 0.0
#     roll, pitch, yaw = 0.0 , 0.0 , 0.0
    
#     control_command = [x,y,z, roll,pitch,yaw]
    
    
    
#     # ---- PHASE 0 : TAKE OFF ----
#     if phase == "takeoff":
#         if phase_transition:
#             phase_transition = False
#             if VERBOSE: 
#                 print("\n-----------------------------------------------")
#                 print("\n              PHASE 0 - TAKE OFF               ")
#                 print("\n-----------------------------------------------"); print("\n")
        
        
#         # Take off sequence to z height of 1m
#         # ... complete here ...
        
#         # Transition to next phase if reached z height
#         if True:    # if z >= 0.9:
#             TransitionToPhase("wait_go")
            
#         return control_command

    
    
#     # ---- PHASE 1 : GATE DETECTION - LAP 1 ----
#     elif phase == "wait_go":
#         if phase_transition:
#             phase_transition = False
#             if VERBOSE:
#                 print("."); print("."); print("."); print(".")
#                 print("\n-----------------------------------------------")
#                 print("\n           PHASE 1 - WAITING FOR GO            ")
#                 print("\n-----------------------------------------------"); print("\n")
            
#         # Wait for user input to transition to speed_run phase
#         user_input = input("Type 'go' to start the speed run: ").strip().lower()
#         if user_input == "go":
#             TransitionToPhase("speed_run")
            
#         # Stay at take off position
#         control_command = [x,y,z, roll,pitch,yaw]

#         return control_command
    
    
    
#     # ---- PHASE 2 : SPEED RUN - LAP 2 & 3 ----
#     elif phase == "speed_run":
#         if phase_transition:
#             phase_transition = False
#             if VERBOSE:
#                 print("\n-----------------------------------------------")
#                 print("\n              PHASE 2 - SPEED RUN              ")
#                 print("\n-----------------------------------------------"); print("\n")
            
            
#         control_command =[x,y,z, roll,pitch,yaw]
        
#         if True:
#             # If drone has completed the laps, transition to "end" phase
            
#             TransitionToPhase("end")
        
#         return control_command

    
    
#     # ---- PHASE 3 : END OF SIMULATION ----
#     elif phase == "end":
#         if phase_transition:
#             phase_transition = False
#             if VERBOSE:
#                 print("."); print("."); print("."); print("."); 
#                 print("\n-----------------------------------------------")
#                 print("\n               END OF SIMULATION               ")
#                 print("\n-----------------------------------------------"); print("\n")
            
#         control_command = [x,y,z, roll,pitch,yaw]
            
#         return control_command
    
    
    
#     # ---- Out of bounds phase state ----
#     else:
#         if phase_transition == True:
#             phase_transition = False
#             print("\n.")
#             print("\nxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
#             print("\nERROR: FSM OUT OF BOUNDS. Invalide phase state.")
#             print("\nxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
#         control_command = [x,y,z, roll,pitch,yaw]
#         return control_command


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
    

def is_on_position(x, y, z, x_goal, y_goal, z_goal):
    global GOAL_THRESHOLD
    """
    Check if the drone is on position within a given GOAL_THRESHOLD.
    
    Parameters:
    - x, y, z: Current position of the drone
    - x_goal, y_goal, z_goal: Goal position
    - GOAL_THRESHOLD: Tolerance for the position check
    
    Returns:
    - True if the drone is on position, False otherwise
    """
    return abs(x - x_goal) < GOAL_THRESHOLD and abs(y - y_goal) < GOAL_THRESHOLD and abs(z - z_goal) < GOAL_THRESHOLD

# -------- General global variables --------
VERBOSE = True  # Set "True" for printing debug information.

phase = "takeoff"           # Phases: "takeoff", "wait_go", "speed_run" or "end"
phase_transition = True     # True if the phase is changing


uri = uri_helper.uri_from_env(default='radio://0/90/2M/E7E7E7E709')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


GOAL_THRESHOLD = 0.2 # in m
TAKE_OFF_HEIGHT = 0.5 # in m

STATE = {
    "TAKE_OFF": 0,
    "RACING": 1,
    "LANDING": 2,
}
DT = 0.1 # in seconds
GOALS = [[0.2, 0, 0.5], [0.5, 0.5, 1.0], [1.5, 1.5, 0.5]] # Example goals for the drone to reach


if __name__ == "__main__":
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    le = LoggingExample(uri)
    cf = le._cf

    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

    # Emergency stop thread
    emergency_stop_thread = threading.Thread(target=emergency_stop_callback, args=(le,))
    emergency_stop_thread.start()

    print("Starting control")

    state = STATE["TAKE_OFF"]
    take_off_reached = False
    waypoint_index = 0

    
    while le.is_connected:
        while True :
        
            x_pos = le.sensor_data['x']
            y_pos = le.sensor_data['y']
            z_pos = le.sensor_data['z']
            roll = le.sensor_data['roll']
            pitch = le.sensor_data['pitch']
            yaw = le.sensor_data['yaw']
            vbat = le.sensor_data['vbat']

            print(f"X: {x_pos:.2f}, Y: {y_pos:.2f}, Z: {z_pos:.2f}, "f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}, "f"VBat: {vbat:.2f}")

            if state == STATE["TAKE_OFF"]:
                if is_on_position(x_pos, y_pos, z_pos, 0, 0, TAKE_OFF_HEIGHT):
                    state = STATE["RACING"]
                    print("Take-off complete. Transitioning to racing.")
                else:
                    cf.commander.send_position_setpoint(0, 0, TAKE_OFF_HEIGHT, 0)
                    print("Takeing off...")
                    

            elif state == STATE["RACING"]:
                x_goal, y_goal, z_goal = GOALS[waypoint_index]

                if is_on_position(x_pos, y_pos, z_pos, x_goal, y_goal, z_goal):
                    waypoint_index += 1
                    if waypoint_index >= len(GOALS):
                        state = STATE["LANDING"]
                        print("All waypoints reached. Transitioning to landing.")
                    else:
                        print(f"Waypoint {waypoint_index} reached.")
                else :
                    print("Racing to gate", f"X: {x_goal:.2f}, Y: {y_goal:.2f}, Z: {z_goal:.2f}")
                    cf.commander.send_position_setpoint(x_goal, y_goal, z_goal, 0)

            elif state == STATE["LANDING"]:
                if is_on_position(x_pos, y_pos, z_pos, 0, 0, 0):
                    print("Landing complete.")
                    cf.commander.send_stop_setpoint()
                    break
                else:
                    # Send a landing command
                    cf.commander.send_position_setpoint(x_pos, y_pos, 0, 0)

            time.sleep(DT)

    
    