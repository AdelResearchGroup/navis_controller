
## Overview
Ros2 package for communicating with the Navis software over HTTP and websockets protocol. Allows for manual control of the mobile base as well as usage of Navis trajectory planning, mapping, obstacle avoidance, and tasks. 

## Installation/Implementation
This package functions relatively independently of the others, since as of now it only cares about Navis and mobile base movement and does not do anything with the dobot arm or brick collision objects. It can accomodate more advanced planning using the manual control or editing the map to let Navis handle brick collision avoidance. 

## Usage
 - "navis_websocket" and "navis_http" executables need to be run, either by a terminal or by a launch file as a node. 
 - Most actions are services that can be called by other nodes, such as getters that use the http client to get info from the Navis. 
 - Navis actions can be done on the navis web client, or with this node running. ie. paths can be planned on the web client and executed on the robot later using the tasks and executing a list task from code. 
 - For most navigation tasks, the robot must be in navigation mode which can be changed by calling the "nav_input" service with op type "start" and id_type "navigation"

### Map Viewer:
 - The map viewer allows for easy testing but should not be used in actual deployment. 
 - Running the "map_viewer" executable starts a gui which displays the current map image gotten by the "get_map_img" service published to the "map_img" topic.
 - When the user clicks somewhere on the map, a trajectory from the robot's current location to the clicked location is created and executed as a real time task by the Navis software, moving the robot in real life. 

### Manual Control:
 - For manual control of the robot, the "start_cmd_vel" service must be called to turn on cmd vel mode.
 - Publishing Twist messages to the "cmd_vel_always" topic commands the robot to move according to the x vel and z rotation of the twist message, controlling forward linear velocity and angular velocity. "always" means the robot does not have to be in navigation mode, but it still needs to have "start_cmd_vel" called to enable manual control mode. 

## Additional Technical Details and Comments
Odometry is published to the "odom" topic after the "start_odom" service is called. This is obtained directly from the Navis which uses the map, lidar, treads, etc to calculate the position. It is general accurate to a few cm and does not noticeably drift as long as the lidar can continuously detect known parts of the map and re-localize. The odom_ema class was present before I (Alex C) developed the rest of the package, and seems to provide aditional functions to interact with the odometry, but I never needed anything more than a general position so I did not implement it in any launch files or testing. 