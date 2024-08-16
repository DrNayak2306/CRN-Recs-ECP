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

SPEED_LOW = 4
SPEED_HIGH = SPEED_LOW + 6.8 
GOAL_REACHED = False

top_left, bottom_left = (0,10),(0,-10)
v_r, v_l = SPEED_LOW, -SPEED_LOW
position = (0,0)
angle = 0
sense, dtheta = 0, 0
o1, o2 = (), ()
way_points = [(802,926),(102,926),(102,715),(920,720),(920, 107),(613,100),(607,400),(197,415)] # Length of an edge: 100 units
way_points_reached = []
old_point = (197,0) 
start_time = 0

############################ USER DEFINED FUNCTIONS ##################################

def angle_deg(p1, p2):

    #### Returns angle in degrees ####

    dx = p2[0]-p1[0]
    dy = p2[1]-p1[1]

    if (dx == 0):
        if (dy > 0): return 90
        else:
            return 270
    elif dy == 0:
        if (dx > 0): return 0
        else: return 180


    raw_angle = math.degrees(math.atan(dy/dx))

    
    if dx > 0 and dy > 0:
        #print("Q1") # Debug
        return raw_angle
    elif dx > 0 and dy < 0:
        #print("Q4") # Debug
        return 360+raw_angle
    else:
        #print("Q2/3") # Debug
        return 180+raw_angle

def arUco_detector(frame):

    #### Marker detection ####

    global position, angle, top_left, bottom_left

    marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000) # Since crn_bot uses a 4X4 of id 0
    param_markers = aruco.DetectorParameters_create() # Default parameters
    marker_corners, marker_IDs, reject = aruco.detectMarkers(frame, marker_dict, parameters=param_markers) # Marking the corners
    if marker_corners:
        for ids, corners in zip(marker_IDs, marker_corners):
            cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 0), 3, cv.LINE_AA)
            bottom_left = (int(corners[0][1][0]), int(corners[0][1][1]))
            top_left = (int(corners[0][2][0]), int(corners[0][2][1]))

            x = (corners[0][0][0] + corners[0][1][0] + corners[0][2][0] + corners[0][3][0]) / 4 # x coordinate
            y = (corners[0][0][1] + corners[0][1][1] + corners[0][2][1] + corners[0][3][1]) / 4 # y coordinate
            #cv.circle(frame, bottom_left, 1, (0, 255, 255), cv.LINE_AA) # Debug: yellow
            #cv.circle(frame, top_left, 1, (255, 0, 255), cv.LINE_AA) # Debug: magenta

            position = (x, y)
            angle = angle_deg(bottom_left, top_left) 


def get_raw_vs_feed(vision_sensor, file_name):

    #### Getting vision sensor data and processing it to be fed to openCV methods ####

    image, resolution = sim.getVisionSensorImg(vision_sensor) # Get video feed as byte array
    imgBuffer = sim.saveImage(image, resolution, 0, file_name, 100) # Save the byte array as image file
    image = Image.open(file_name) # Open the image file using PIL
    frame = np.array(image) # Convert the image file into a numpy array (only alternative format type for image)
    return frame


def display_processed_feed(frame, text):
    global GOAL_REACHED

    #### Write localization information ####
    position_pos = (20, 25)
    orientation_pos = (210, 25)
    wp_pos = (20, 75)
    progress_pos = (210, 75)
    cv.putText(frame, "Position",position_pos, cv.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 0), 2) 
    cv.putText(frame, str(text),(position_pos[0], position_pos[1]+20), cv.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 2) # Display the real time position on the feed
    cv.putText(frame, "Orientation",orientation_pos, cv.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 0), 2) 
    cv.putText(frame, str(round(angle,2)),(orientation_pos[0], orientation_pos[1]+20), cv.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 255), 2) # Display the real time orientation on the feed
    cv.putText(frame, "Waypoints reached",wp_pos, cv.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 0), 2) # Display the real time orientation on the feed
    cv.putText(frame, str(len(way_points_reached)),(wp_pos[0], wp_pos[1]+20), cv.FONT_HERSHEY_SIMPLEX,0.5, (155, 0, 255), 2) # Display the real time orientation on the feed
    cv.putText(frame, "Progress",progress_pos, cv.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 0), 2) # Display the real time orientation on the feed
    message = ""
    if GOAL_REACHED:
        message = "GOAL REACHED!"
    else:
        message = str(round(len(way_points_reached)*100/(len(way_points_reached)+len(way_points)),1))+"%"
    cv.putText(frame, message,(progress_pos[0],progress_pos[1]+20), cv.FONT_HERSHEY_SIMPLEX,0.5, (255, 255, 0), 2) 

    cv.imshow("Vision Sensor Feed", frame) # Display the modified feed

    #### Exit mechanism ####
    key = cv.waitKey(1)
    if key == ord("q"):
        cv.destroyAllWindows()

def scan_map():
    global position, angle, old_point

def name_components(sim):
    #### Define handles ####
    bot = sim.getObject("/crn_bot")
    r_motor = sim.getObject("/crn_bot/joint_r")
    l_motor = sim.getObject("/crn_bot/joint_l")
    vision_sensor = sim.getObjectHandle("/vision_sensor")

def navigation(way_points):
    global way_points_reached, v_r, v_l, old_point
    global top_left, bottom_left, dtheta, sense, o1, o2
    global GOAL_REACHED

    point = way_points[-1]
    if math.dist(position, point) < 68 and point not in way_points_reached:
        way_points_reached.append(point) # Mark point as reached
        old_point = way_points.pop() # Delete from target list
        #print(way_points) # Debug
        #print("="*100) # Debug
    if not way_points:
        GOAL_REACHED = True
        print("Goal reached")
        return

    o1 = (top_left[0]-bottom_left[0], top_left[1]-bottom_left[1]) # Orientation of the bot
    o2 = (way_points[-1][0]-old_point[0], way_points[-1][1]-old_point[1]) # Target orientation
    dtheta = angle_deg((0,0), o1) - angle_deg ((0,0), o2) # Deviation in degrees


    #### Determine the turning sense and calculate effective deviation ####
    if dtheta < 0:
        dtheta *= -1
        if dtheta <= 180:
            sense = 1 # Turn right
        else:
            sense = 0 # Turn left
            dtheta = 360-dtheta
    else:
        if dtheta <= 180:
            sense = 0
        else:
            sense = 1
            dtheta = 360-dtheta

    #### p-system logic ####
    if sense == 1:
        v_l = -SPEED_LOW-SPEED_HIGH*(dtheta/180)
        #print("Turning right", v_l, v_r) # Debug
    else:
        v_r = SPEED_LOW+SPEED_HIGH*(dtheta/180)
        #print("Turning left", v_l, v_r) # Debug



################################ MAIN FUNCTION #######################################

def simulator(sim):
    global v_r, v_l, old_point

    name_components(sim)
    #### Define handles ####
    bot = sim.getObject("/crn_bot")
    r_motor = sim.getObject("/crn_bot/joint_r")
    l_motor = sim.getObject("/crn_bot/joint_l")
    vision_sensor = sim.getObjectHandle("/vision_sensor")

    #### Set required kinematic action ####
    sim.setJointTargetVelocity(r_motor, v_r);
    sim.setJointTargetVelocity(l_motor, v_l);
    
    #### Vision sensor image processing loop (mainloop) ####
    while sim.getSimulationState != sim.simulation_stopped:
        if GOAL_REACHED:
            sim.setJointTargetVelocity(r_motor, -(v_r/2));
            sim.setJointTargetVelocity(l_motor, -(v_l/2));
            return

        #sim.setObjectPosition(bot,[-1.225, 1.0, 0.00096]) # Debug
        point = way_points[-1]

        #### Debug ####
        #print("Current way point: ",point)
        #print("Old way point: ",old_point)
        #print("Distance: ",math.dist(position, point))
        #print("o2: ",o2)
        #print("o1: ",o1)
        #print("angle: ",angle)
        #print("Deviation: ",dtheta)

        frame = get_raw_vs_feed(vision_sensor, "feed.png")
        arUco_detector(frame)
        navigation(way_points)
        sim.setJointTargetVelocity(r_motor, v_r);
        sim.setJointTargetVelocity(l_motor, v_l);
        display_processed_feed(frame, position)

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

