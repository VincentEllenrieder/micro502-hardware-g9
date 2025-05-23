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
from demo_simple_planner import extract_best_path

# Global variables
uri = uri_helper.uri_from_env(default='radio://0/90/2M/E7E7E7E709')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Group defined parameters
GOAL_THRESHOLD = 0.05 # in m
# TAKE_OFF_HEIGHT = 0.4 # in m

STATE = {
    "RACING": 1,
    "LANDING": 2,
}

GOALS = extract_best_path(csv_path="demo_gates_info.csv")
print("Best total path :", GOALS)

MAX_VEL_X = 0.5 # in m/s
MAX_VEL_Y = 0.5 # in m/s
MAX_VEL_Z = 0.2 # in m/s

DT = 0.1 # in seconds

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

if __name__ == "__main__":
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    le = LoggingExample(uri)
    cf = le._cf

    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

    # Set maximum velocities
    cf.param.set_value('posCtlPid.xVelMax', '0.5')  # default = 1.0 m/s
    cf.param.set_value('posCtlPid.yVelMax', '0.5')   # default = 1.0 m/s
    cf.param.set_value('posCtlPid.zVelMax', '0.2')   # default = 0.5 m/s
    time.sleep(0.5)

    # Emergency stop thread
    emergency_stop_thread = threading.Thread(target=emergency_stop_callback, args=(le,))
    emergency_stop_thread.start()

    print("Starting control")

    state = STATE["RACING"]
    waypoint_index = 0
    
    while le.is_connected:
        
        x_pos = le.sensor_data['x']
        y_pos = le.sensor_data['y']
        z_pos = le.sensor_data['z']
        roll = le.sensor_data['roll']
        pitch = le.sensor_data['pitch']
        yaw = le.sensor_data['yaw']
        vbat = le.sensor_data['vbat']

        print(f"X: {x_pos:.2f}, Y: {y_pos:.2f}, Z: {z_pos:.2f}, "f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}, "f"VBat: {vbat:.2f}")
              
        if state == STATE["RACING"]:
            x_goal, y_goal, z_goal = GOALS[waypoint_index]

            if is_on_position(x_pos, y_pos, z_pos, x_goal, y_goal, z_goal):
                waypoint_index += 1
                if waypoint_index >= len(GOALS):
                    state = STATE["LANDING"]
                    print("All waypoints reached. Transitioning to landing.")
                else:
                    print(f"Waypoint {waypoint_index} reached.")
            else :
                print("Racing to waypoint", f"X: {x_goal:.2f}, Y: {y_goal:.2f}, Z: {z_goal:.2f}")
                cf.commander.send_position_setpoint(x_goal, y_goal, z_goal, 0)

        elif state == STATE["LANDING"]:
            if is_on_position(x_pos, y_pos, z_pos, x_pos, y_pos, 0):
                print("Landing complete.")
                cf.commander.send_stop_setpoint()
                break
            else:
                # Send a landing command
                cf.commander.send_position_setpoint(x_pos, y_pos, 0, 0)

        time.sleep(DT)


