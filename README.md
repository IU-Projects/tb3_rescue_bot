# tb3_rescue_bot

## Overview
tb3_rescue_bot for USAR

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
  



