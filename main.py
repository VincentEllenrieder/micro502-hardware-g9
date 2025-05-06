# ================================================= #
# Crazyflie Hardware Assignment                     #
# Aerial robotics (MICRO-502)                       #
#                                                   #
#                      Property of the EPFL LIS LAB #
#                              2025 - Fall semester #
#                                                   #
# ------------------------------------------------- #
# Assignemet done by group 9 :                      #
#                                                   #
#                          Charles Proffit (324624) #
#                           Clément Chalut (326251) #
#                             Cyril Goffin (373937) #
#                          Jeremy Serillon (326033) #
#                       Vincent Ellerieder (329051) #
# ================================================= #




#################################### #   GLOBAL VARIABLES   # ######################################


# -------- General global variables --------
VERBOSE = True  # Set "True" for printing debug information.

phase = "takeoff"           # Phases: "takeoff", "wait_go", "speed_run" or "end"
phase_transition = True     # True if the phase is changing











############################ #   PHASE 1 FUNCTIONS - Vision Module   # #############################




############################ #   PHASE 2 FUNCTIONS - Motion Planner   # ############################






#################################### #   UTILITY FUNCTIONS   # #####################################
def TransitionToPhase(phase_name):
    """
    Transition to a new phase of the project while setting the phase_transition flag.
    
    Parameters:
    - phase_name: The name of the new phase to transition to. It can be
                  "takeoff", "wait_go", "speed_run" or "end".
    """
    
    global phase, phase_transition
    
    # Check if valid phase name
    if phase_name not in ["takeoff", "wait_go", "speed_run", "end"]:
        print("\n.")
        raise ValueError("Invalid phase name. Cannot transition to phase: " + phase_name)

    if VERBOSE:
        print(f"  - Transitioning from '{phase}' to '{phase_name}'")
    phase = phase_name
    phase_transition = True








###################################### #   MAIN COMMAND   # ########################################
def get_command(arg1, arg2, arg3, arg4, dt):
    """
    This function holds the logic for the different phases of the simulation.
    The function is called every simulation step and should return the control command for the drone.
    
    Phases:
     - 0: "takeoff"
     - 1: "wait_go"
     - 2: "speed_run"
     - 3: "end"
    """
    
    global phase, phase_transition
    
    x,    y,     z   = 0.0 , 0.0 , 0.0
    roll, pitch, yaw = 0.0 , 0.0 , 0.0
    
    control_command = [x,y,z, roll,pitch,yaw]
    
    
    
    # ---- PHASE 0 : TAKE OFF ----
    if phase == "takeoff":
        if phase_transition:
            phase_transition = False
            if VERBOSE: 
                print("\n-----------------------------------------------")
                print("\n              PHASE 0 - TAKE OFF               ")
                print("\n-----------------------------------------------"); print("\n")
        
        
        # Take off sequence to z height of 1m
        # ... complete here ...
        
        # Transition to next phase if reached z height
        if True:    # if z >= 0.9:
            TransitionToPhase("wait_go")
            
        return control_command

    
    
    # ---- PHASE 1 : GATE DETECTION - LAP 1 ----
    elif phase == "wait_go":
        if phase_transition:
            phase_transition = False
            if VERBOSE:
                print("."); print("."); print("."); print(".")
                print("\n-----------------------------------------------")
                print("\n           PHASE 1 - WAITING FOR GO            ")
                print("\n-----------------------------------------------"); print("\n")
            
        # Wait for user input to transition to speed_run phase
        user_input = input("Type 'go' to start the speed run: ").strip().lower()
        if user_input == "go":
            TransitionToPhase("speed_run")
            
        # Stay at take off position
        control_command = [x,y,z, roll,pitch,yaw]

        return control_command
    
    
    
    # ---- PHASE 2 : SPEED RUN - LAP 2 & 3 ----
    elif phase == "speed_run":
        if phase_transition:
            phase_transition = False
            if VERBOSE:
                print("\n-----------------------------------------------")
                print("\n              PHASE 2 - SPEED RUN              ")
                print("\n-----------------------------------------------"); print("\n")
            
            
        control_command =[x,y,z, roll,pitch,yaw]
        
        if True:
            # If drone has completed the laps, transition to "end" phase
            
            TransitionToPhase("end")
        
        return control_command

    
    
    # ---- PHASE 3 : END OF SIMULATION ----
    elif phase == "end":
        if phase_transition:
            phase_transition = False
            if VERBOSE:
                print("."); print("."); print("."); print("."); 
                print("\n-----------------------------------------------")
                print("\n               END OF SIMULATION               ")
                print("\n-----------------------------------------------"); print("\n")
            
        control_command = [x,y,z, roll,pitch,yaw]
            
        return control_command
    
    
    
    # ---- Out of bounds phase state ----
    else:
        if phase_transition == True:
            phase_transition = False
            print("\n.")
            print("\nxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
            print("\nERROR: FSM OUT OF BOUNDS. Invalide phase state.")
            print("\nxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
        control_command = [x,y,z, roll,pitch,yaw]
        return control_command
    
    
    
    
    
    
if __name__ == "__main__":
    """
    Main function to run the project
    """
    print("\n")
    print("===============================================")
    print("          CRAZYFLIE HARDWARE ASSIGNMENT        ")
    print("            group 9 - Aerial Robotics          ")
    print("===============================================")
    
    print("Launching...")
    
    arg1 = 0.0
    arg2 = 0.0
    arg3 = 0.0
    arg4 = 0.0
    dt = 0.1
    
    while True:
        control_command = get_command(arg1, arg2, arg3, arg4, dt)
    
    