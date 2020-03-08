# tb3_rescue_bot

## Overview
Disasters are horrifying and cost the lives of mankind in many ways while influencing relatives and other people. Because of that, higher priority always should be given to act as quickly as possible to perform rescue operations. Due to the higher density of the population and buildings in urban areas are more prone to disasters. As inspired by these requirements, simplified USAR robot package for Robot Operating System(ROS) was implemented to replicate a USAR task. Our Robot explores the maze and successfully detect and locate the victims and navigate back to the starting point. In the end, the robot provides the map of the maze with the marked location of the beacons. We experimented with wall following exploration and frontier exploration. Frontier-based exploration was used as it was more suited for environments with open spaces. Victim detection was achieved through opencv and Kinect RGB camera (turtlebot 3). We validated our approach through four different gazebo environments and one real-world experiment using turtlebot3 to generate a clear map of the environment with marked victims on it.

## Video demonstration
Here is the link to the [video](https://youtu.be/vjE009efW6Y) demonstartion of the project.

## Technology Usage

* ROS
* Turtlebot 3

## Instructions for installation and run

* git clone the project.

* you should have installed ros and turtlebot3.

* Change the directory to your ros package src directory **catkin_ws/src** folder.

* Copy all packages from the **tb3_rescue_bot** to **catkin_ws/src**.

* Source the setup.
  
  * ```source devel/setup.bash```

* Build the project.
  
  * ```catkin_make```

* First, Launch the tb3 with gazebo.
  
  * ```roslaunch tb3_rescue maze.launch```

* Next, Start automated mapping using following command in a seperate terminal.
  
  * ```roslaunch tb3_rescue automated_mapping.launch```

* To add mapping victims, Start the victim mapper in a another terminal window

  * ```rosrun tb3_victim_mapper victim_mapper```

* Then, Launch a command line options pannel in a seperate terminal.
  
  * ```roslaunch tb3_teleop tb3_teleop.launch```

* If you want to manually naviagate the robot use the tb3_teleop in a seperate terminal.
  
  * ```roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch```
  



