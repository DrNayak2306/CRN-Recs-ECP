import  sys
import traceback
import time
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

############################## GLOBAL VARIABLES ######################################



############################ USER DEFINED FUNCTIONS ##################################



################################ MAIN FUNCTION #######################################

def simulator(sim):
    bot = sim.getObject("/crn_bot") # Get the bot handle
    r_joint = sim.getObject("/crn_bot/joint_r") # Get right joint handle
    l_joint = sim.getObject("/crn_bot/joint_l") # Get left joint handle
    sim.setObjectPosition(bot,[-1.225, 1.0, 0.00096]) # Initialize the position of the bot to a more spacious location
    v_r = 100 # Define right motor velocity
    v_l = -10 # Define left motor velocity
    sim.setJointTargetVelocity(r_joint, v_r); # Set right motor target velocity
    sim.setJointTargetVelocity(l_joint, v_l); # Set left motor target velocity
	
    return None
	
######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THE MAIN CODE BELOW #########

if __name__ == "__main__":
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    try:

    ## Start the simulation using ZeroMQ RemoteAPI
        try:
            return_code = sim.startSimulation()
            if sim.getSimulationState() != sim.simulation_stopped:
                print('\nSimulation started correctly in CoppeliaSim.')
            else:
                print('\nSimulation could not be started correctly in CoppeliaSim.')
                sys.exit()

        except Exception:
            print('\n[ERROR] Simulation could not be started !!')
            traceback.print_exc(file=sys.stdout)
            sys.exit()

        ## Runs the robot navigation logic written by participants
        try:
            simulator(sim)
            time.sleep(5)

        except Exception:
            print('\n[ERROR] Your simulator function throwed an Exception, kindly debug your code!')
            print('Stop the CoppeliaSim simulation manually if required.\n')
            traceback.print_exc(file=sys.stdout)
            print()
            sys.exit()


        ## Stop the simulation using ZeroMQ RemoteAPI
        try:
            return_code = sim.stopSimulation()
            time.sleep(0.5)
            if sim.getSimulationState() == sim.simulation_stopped:
                print('\nSimulation stopped correctly in CoppeliaSim.')
            else:
                print('\nSimulation could not be stopped correctly in CoppeliaSim.')
                sys.exit()

        except Exception:
            print('\n[ERROR] Simulation could not be stopped !!')
            traceback.print_exc(file=sys.stdout)
            sys.exit()

    except KeyboardInterrupt:
    ## Stop the simulation using ZeroMQ RemoteAPI
        return_code = sim.stopSimulation()
        time.sleep(0.5)
        if sim.getSimulationState() == sim.simulation_stopped:
            print('\nSimulation interrupted by user in CoppeliaSim.')
        else:
            print('\nSimulation could not be interrupted. Stop the simulation manually .')
        sys.exit()

