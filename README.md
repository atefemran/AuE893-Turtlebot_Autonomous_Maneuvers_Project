![Cover](https://raw.githubusercontent.com/atefemran/AuE893-Turtlebot_Autonomous_Maneuvers_Project/main/src/videos/git%20cover.jpg)

This repository contains the code and assets for the ROS-based project "TurtleBot Autonomous Maneuvers". As a part of class AUE893 - Autonomy Science and Systems, at Clemson University International Center for Automotive Research (CU-ICAR).
 
## Project Overview

The project is ROS based using the Turtlebot3 Burger in simulation (Gazebo) and real-world environments to go through a prespecified course in which the TurtleBot will have to deal with 4 maneouvers:

![Track](https://raw.githubusercontent.com/atefemran/AuE893-Turtlebot_Autonomous_Maneuvers_Project/main/src/videos/Track.png)

1. Wall follower and Obstacle avoidance
   Turtlebot maintains safe distance from obstacle and maneuvers through course until it finds yellow lanes. In this it manipulates Lidar values for the design of controller.
		
3. Line following
   Turtlebot follows the yellow line using image processing (open-cv package). The controller uses lane centroid to control the angular speed of the bot, with maintained linear 
   velocity. 

4. Traffic sign detection
   YOLO is used for traffic sign detection. Traffic sign callback functions stop the turtlebot for 3 seconds, then it proceeds following the line. 

5. April Tag follower
   AprilTag_ros package is used to detect apriltags and then its co-ordinates are used to design the controller for tracking the april tag, and maintain a sfae distance. 

6. Autonomous switching
   Navigating the whole course autonomously without any manual entry. This is being excuted using the below ROS archicture.
   ![ROS_switching](https://raw.githubusercontent.com/atefemran/AuE893-Turtlebot_Autonomous_Maneuvers_Project/main/src/videos/ROS_switching.png)

 
 

##  Results

### Gazebo simulation
![Gazebo gif](https://github.com/atefemran/AuE893-Turtlebot_Autonomous_Maneuvers_Project/blob/main/src/videos/Gazebo%20Simulation%20AUE893%20Turtlebot%20Autonomous%20Maneuvers%20Project_1080p.gif?raw=true)

### Real-World Demo
![realworld gif](https://github.com/atefemran/AuE893-Turtlebot_Autonomous_Maneuvers_Project/blob/main/src/videos/RealWorld%20AUE893%20Turtlebot%20Autonomous%20Maneuvers%20Project_480p.gif?raw=true)

## Running the project
The project was created on ROS1 on Ubuntu 20.04.

### Dependencies
1. [Turtlebot3 package](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)	
2. [Turlebot3 simulation](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
3. [open cv](https://github.com/ros-perception/vision_opencv)
4. [Apriltag ROS](https://github.com/AprilRobotics/apriltag_ros)
5. [Darknet Ros(YOLO)](https://github.com/leggedrobotics/darknet_ros)

### Running Commands 
Use the below lines in your terminal to start the launch files.
1. For Gazebo simulation
		1. '$ roslaunch project_turtlebot_maneouvres turtlebot3_maneouvres.launch'
		2. '$ rosrun project_turtlebot_maneouvres keyboard_teleop_apriltag.py'    -- for operating the 2nd turtlebot with apriltag in the gazebo world.
	
2. For real world
		1. Bring-up the turtleot
		2. '$ rosrun image_transport republish compressed in:=/raspicam_node/image raw out:=camera/rgb/image_raw'     -- tp republish raspicam_node to rgb/camera
		3. '$ roslaunch project_turtlebot_maneouvres turtlebot3_maneouvres_real.launch'

## Team Members
		Atef Emran 
		Anirudha Sundar  
		Kartik Loya 
		Ziyue Feng
		Abhijeet Mordekar 
