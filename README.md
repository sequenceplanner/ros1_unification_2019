### NOTE: Things in this README might not be true if you can see this message

# ros1_unification_2019

- Package for the ROS1 part of the Unification project for 2019
- Support for UR10 robots: TARS, KIPP and CASE
- TODO: Support for IIWA7 robot: PLEX

## Prerequisites

1.  http://wiki.ros.org/ROS/Installation
2.  https://github.com/ros-industrial/universal_robot
3.  https://github.com/ros-industrial/ur_modern_driver
4.  http://wiki.ros.org/timed_roslaunch

- TODO: List dependancies
- TODO: Add dependancies in a bash script for easier installation

### Get yourself a robot
1.  Official UR simulated robot (recommended for Ubuntu 16.04):\
Get the official UR simulator from: https://www.universal-robots.com/download/ and extract the folder somewhere convenient. Start the simulator by following the instrucions on the UR website. As MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi], it is recommended to restrict the robot's joints within the range of [-pi, pi] and that limitation is set to default for the moveit planning execution in the wake_up.launch file. The joint limits should also be set to [-pi, pi] in the official UR simulator because that will disallow the robot to move outside of those joint limits within the simulator itself, so forcing the robot outside of those bounds will result in Protective Stop. It is recommended to use the official simulator since you will have the capability to send URScript commands and do some other things.

2.  Demo simulated robot provided with the ur10_moveit_config package (Ubuntu 18.04, or figure out how to make the official UR simulator work Ubuntu 18.04).

3.  Real robot. Since the official UR Simulated robot is run on a local machine, the default robot_ip has been set to 0.0.0.0. For a real robot follow the roslaunch command with a robot_ip argument. Also, restrict all joint limits to [-pi, pi] the same way you would do it in the simulator.

Waking up robots can be done with the roslaunch command followed with the robot_name argument. Let's wake up a simulated TARS on a local machine for example:
```
roslaunch ros1_unification_2019 wake_up.launch robot_name:=TARS
```
If you are waking up a real robot, add the robot_ip argument in the end:
```
roslaunch ros1_unification_2019 wake_up.launch robot_name:=TARS robot_ip:=xxx.xxx.x.xxx
```

## Nodes

A few nodes exist in this package that aim to enable and in the same time simplify interactions with one or multiple robots (via ROS2, see https://github.com/endre90/ros2_ros1_multimaster). Short explanations and message type deffinitions follow.

### ur_pose_updater

TODO: some testing
Allows the user to save robot poses by sending messages to this node. The poses are saved in according csv files and can be later looked up by other nodes.
- Support for pose manipulation: UPDATE, DELETE and CLEAR
- UPDATE: if pose name is not existing, append to specified pose list
- UPDATE: if pose name existing in the pose list, update with new values
- DELETE: if pose name is not existing, skip with error msg
- DELETE: if pose name existing in the pose list, remove pose from the list
- CLEAR: if pose name and type specified, clear the whole pose list, leave the 'control_pose'
- CLEAR: if pose name or type not specified, skip with error mmsg (safety feature)

An example message that this node consumes:
```
action: 'UPDATE'
robot_type: 'UR10'
robot_name: 'TARS'
pose_type: 'JOINT'
pose_name: 'POSE1'
```

An example message that the node publishes, for example after receiving the message above:
```
info: 
  robot_name: "TARS"
  fresh_msg: True
  t_plus: "2.9 seconds"
  got_reset: False
  error_list: []
ricochet: 
  got_action: "UPDATE"
  got_pose_type: "JOINT"
  got_pose_name: "POSE1"
  got_robot_type: "UR10"
  got_robot_name: "TARS"
saved_poses: 
  joint_pose_list: [control_pose, POSE1]
  tcp_pose_list: [control_pose]
last_completed_action: "appended: POSE1 to JOINT list"
```
The info part of the published message contains the name of the robot which poses are manipulated, a message freshness indicator (you can set the time defining freshness in the script), elapsed time since the plast consumed message, a reset indicator showing that the node consumed a reset command and a list of errors that is generated after each message is consumed. The ricochet part publishes the received part so that the command publisher can have a comparisson if needed. All the saved poses in the csv files are published in two separate files that contain all the JOINT or the TCP poses. Lastly, The last completed action shows whait it says it shows.

### ur_pose_unidriver

TODO: some testing
Allows the user to move the robot around with planning using MoveIt, or without planning using URScript(UR robots only).
- Support for pose manipulation: MOVEJ, MOVEL and PLANNED
- MOVEJ: move the manipulator in a linear joint-space move
- MOVEL: move the manipulator in a linear tool-space move
- PLANNED: plan a collision-free path using MoveIt and execute it 

An example message that this node consumes:
```
action: 'PLANNED'
robot_type: 'UR10'
robot_name: 'TARS'
pose_type: 'JOINT'
pose_name: 'POSE1'
speed_scaling: 0.1
acc_scaling: 0.1
goal_tolerance: 0.01
```

An example message that the node publishes, for example after receiving the message above:
```
info: 
  robot_name: "TARS"
  fresh_msg: True
  t_plus: "1.5 seconds"
  got_reset: False
  error_list: []
ricochet: 
  got_action: "PLANNED"
  got_robot_type: "UR10"
  got_robot_name: "TARS"
  got_pose_type: "JOINT"
  got_pose_name: "POSE1"
  got_speed_scaling: "0.100"
  got_acc_scaling: "0.100"
  got_goal_tolerance: "0.010"
moving: False
actual_pose: "POSE3"
```

The info part of the published message contains the name of the robot which poses are manipulated, a message freshness indicator (you can set the time defining freshness in the script), elapsed time since the plast consumed message, a reset indicator showing that the node consumed a reset command and a list of errors that is generated after each message is consumed. The ricochet part publishes the received part so that the command publisher can have a comparisson if needed. Moving indicates True if any of the six joint velocities in nonequal to zero, and after that the actual pose of the robot is shown where both static and transport poses are shown.

### scene_updater

TODO: make it work
TODO: afterwards, some testing
Allows the user to manipulate the planning scene by adding, attaching and removing collision objects in the scene. This is done using the planning scene interface from moveit commander. 
- Support for object manipulation: ADD, REMOVE, MOVE, CLEAR, ATTACH and DETACH

An example message that this node consumes:
```
object_action: 'ADD'
object_name: 'ENGINE'
euler_pose: [2, 2, 2, 1.5707, 0, -1.5707]
```

An example message that the node publishes, for example after receiving the message above:
```
info: 
  robot_name: "TARS"
  fresh_msg: True
  t_plus: "1.5 seconds"
  got_reset: False
  error_list: []
ricochet: 
  got_object_action: "ADD"
  got_object_name: "ENGINE"
  euler_pose: [2, 2, 2, 1.5707, 0, -1.5707]
attached_objects: []
object_poses: [{"ENGINE":[2, 2, 2, 1.5707, 0, -1.5707]}]
```
 
### ur_transformations

Not really a node, but a class that provides transformation methods if needed

### robotiq_unidriver

Communication with the Robotiq gripper