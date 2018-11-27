#### Victim Mapper

This uses the geometry_msgs/Pose messages for Victim mapping.

#### How to run:

Run the Victim Mapper
```bash
rosrun tb3_victim_mapper victim_mapper
```

Start RVIZ
```bash
rosrun rviz rviz
```

Send a message of type geometry_msgs/Pose to the /victim_info_listener topic. Ex:
```bash
rostopic pub /victim_info_listener geometry_msgs/Pose '{position: {x: 1, y: 0.0,  0.0}, orientation: {x: 0.0,y: 0.0,z: 0.0, w: 1.0}}'
```
