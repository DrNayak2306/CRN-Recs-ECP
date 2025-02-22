# CRN-Recs-ECP

## Introduction:

This is a set of three tasks, given for the purpose of recruitment to CSD Robocon NITK.
1. Milestone-1: Make the bot move in circles of three random radii.
2. Milestone-2: Implement localization without using API functions ```getObjectPosition()``` and ```getObjectOrientation()```.
3. Milestone-3: Implement control system and navigate through the waypoints.

This project uses openCV-python to analyze the visual sensor data, and determine the position, orientation and thus path progress of the bot.
With the locations of the waypoints fed to the bot, it uses p-system or proportionality system with angular error to orient itself towards the next waypoint.

## Milestone-1 

![Milestone-1 Demo](https://github.com/user-attachments/assets/b3de9f7a-c8dd-4124-900d-aa3617484c55)

## Milestone-2 

Axes:

<img src="https://github.com/user-attachments/assets/837b01c3-3528-4383-88e6-d1df75b80423" width=550 height=550>

Orientation is 0 deg when the bot is exactly opposite of the default starting orientation, thus starting orientation is 180 deg. Angle increases upon rotating clockwise.

![Milestone-2 Demo](https://github.com/user-attachments/assets/6d9a0c07-bbbd-42df-8d13-bfbfca539fdb)

## Milestone-3

Unfortunately, I couldn't complete the implementation within the given time frame.
Here's the logic I would go ahead with given about a day or so more:
  1. We pass as a list the waypoints to the robot
  2. The robot has its own orientation: o1, given from the AruCo tracking system; and the orientation needed to reach the next waypoint: o2, calculated from the list of waypoints.
  3. The robot utilizes PID or a similar damping error correction system to match o1 with o2 while commuting.
  4. Upon reaching the waypoint (within a given tolerance vicinity), o2 is updated and we go to step 3, except when the waypoint is the goal.
     
The error correction system has some detail to it, which I couldn't manage implementing due to shortage of time.

![Milestone-3 Demo](https://github.com/user-attachments/assets/a976de8d-8ec2-4e07-85e4-148a2a70fed6)

### Update:
1. I finally got the control system figured out. It is a p-system, and uses only proportionality for error correction.
2. Orientation system has been changed. Now starting orientation is 90 degrees, 0 degrees being bot facing way point 2 from way point 1. 

![Milestone-3 New Demo](https://github.com/user-attachments/assets/f1bf7f55-4140-468d-869f-030671d469cc)











