import  sys
import traceback
import time
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

#### My imports ####
import cv2 as cv; from cv2 import aruco
from PIL import Image
import numpy as np

############################## GLOBAL VARIABLES ######################################

coordinates = (0,0)
angle = 0
way_points = []

############################ USER DEFINED FUNCTIONS ##################################

def angle_deg(p1, p2):

    #### Returns angle in degrees ####

    dx = p2[0]-p1[0]
    dy = p2[1]-p1[1]

    if (dx == 0):
        dx += pow(10,-10)
    raw_angle = math.degrees(math.atan(dy/dx))

    if dx > 0 and dy > 0:
        return raw_angle
    elif dx > 0 and dy < 0:
        return 360+raw_angle
    elif dx < 0 and dy > 0:
        return 180+raw_angle
    else:
        return 180-raw_angle

def arUco_detector(frame):

    #### Marker detection ####

    global coordinates
    global angle

    marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000) # Since crn_bot uses a 4X4 of id 0
    param_markers = aruco.DetectorParameters_create() # Default parameters
    marker_corners, marker_IDs, reject = aruco.detectMarkers(frame, marker_dict, parameters=param_markers) # Marking the corners
    if marker_corners:
        for ids, corners in zip(marker_IDs, marker_corners):
            cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 0), 3, cv.LINE_AA)
            top_left = (int(corners[0][0][0]), int(corners[0][0][1]))
            top_right = (int(corners[0][1][0]), int(corners[0][1][1]))

            x = (corners[0][0][0] + corners[0][1][0] + corners[0][2][0] + corners[0][3][0]) / 4 # x coordinate
            y = (corners[0][0][1] + corners[0][1][1] + corners[0][2][1] + corners[0][3][1]) / 4 # y coordinate
            #cv.circle(frame, top_left, 1, (0, 255, 255), cv.LINE_AA) # Debug: Left yellow
            #cv.circle(frame, top_right, 1, (255, 0, 255), cv.LINE_AA) # Debug: Right magenta

            coordinates = (x, y)
            angle = angle_deg(top_right, top_left) 


def get_raw_vs_feed(vision_sensor, file_name):

    #### Getting vision sensor data and processing it to be fed to openCV methods ####

    image, resolution = sim.getVisionSensorImg(vision_sensor) # Get video feed as byte array
    imgBuffer = sim.saveImage(image, resolution, 0, file_name, 100) # Save the byte array as image file
    image = Image.open(file_name) # Open the image file using PIL
    frame = np.array(image) # Convert the image file into a numpy array (only alternative format type for image)
    return frame


def display_processed_feed(frame, text):

    #### Write localization information ####
    cv.putText(frame, str(text),(20, 40), cv.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 2) # Display the real time coordinates on the feed
    cv.putText(frame, str(angle),(200, 40), cv.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 255), 2) # Display the real time orientation on the feed

    cv.imshow("Vision Sensor Feed", frame) # Display the modified feed

    #### Exit mechanism ####
    key = cv.waitKey(1)
    if key == ord("q"):
        cv.destroyAllWindows()


################################ MAIN FUNCTION #######################################

def simulator(sim):

    #### Define handles ####
    bot = sim.getObject("/crn_bot")
    r_motor = sim.getObject("/crn_bot/joint_r")
    l_motor = sim.getObject("/crn_bot/joint_l")
    vision_sensor = sim.getObjectHandle("/vision_sensor")

    #### Set required kinematic action ####
    v_r, v_l = 30, -20 
    sim.setJointTargetVelocity(r_motor, v_r);
    sim.setJointTargetVelocity(l_motor, v_l);
    
    #### Vision sensor image processing loop ####
    while sim.getSimulationState != sim.simulation_stopped:
        frame = get_raw_vs_feed(vision_sensor, "feed.png")
        arUco_detector(frame)
        display_processed_feed(frame, coordinates)

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

