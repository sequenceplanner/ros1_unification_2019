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

Get yourself a robot:\
1.  Official UR simulated robot (recommended for Ubuntu 16.04):\
Get the official UR simulator from: https://www.universal-robots.com/download/ and extract the folder somewhere convenient. Start the simulator by following the instrucions on the UR website. As MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi], it is recommended to restrict the robot's joints within the range of [-pi, pi] and that limitation is set to default for the moveit planning execution in the wake_up.launch file. The joint limits should also be set to [-pi, pi] in the official UR simulator because that will disallow the robot to move outside of those joint limits within the simulator itself, so forcing the robot outside of those bounds will result in Protective Stop. It is recommended to use the official simulator since you will have the capability to send URScript commands and do some other things.

2.  Demo simulated robot provided with the ur10_moveit_config package (Ubuntu 18.04, or figure out how to make the official UR simulator work Ubuntu 18.04).

3.  Real robot. Since the official UR Simulated robot is run on a local machine, the default robot_ip has been set to 0.0.0.0. For a real robot follow the roslaunch command with a robot_ip:=xxx.xxx.x.xxx argument. Also, restrict all joint limits to [-pi, pi] the same way you would do it in the simulator.

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

![Alt text](https://g.gravizo.com/svg?
  digraph G {
    size ="4,4";
    UR10 -> TARS;
    UR10 -> KIPP;
    UR10 -> CASE;
    IIWA7 -> PLEX;
    TARS -> JOINT;
    TARS -> TCP;
    KIPP -> JOINT;
    KIPP -> TCP;
    CASE -> JOINT;
    CASE -> TCP;
    PLEX -> JOINT;
    PLEX -> TCP;
    TCP -> UPDATE;
    TCP -> DELETE;
    TCP -> CLEAR;
    JOINT -> UPDATE;
    JOINT -> DELETE;
    JOINT -> CLEAR;
  }
)


## STATUS
### ur_pose_updater - should work
### ur_pose_unidriver - should work (beta, needs more testing)
### scene_updater - not really working (alpha, needs more work)

UPDATE README
ADD timed_roslaunch dependency

- Package for the ROS1 part of the Unification project for 2019.
- Using the ee_link for the end effector link, so tool0_controller is not used because it does not give consistent data. 
- A possible solution (proposed by Christian - FCC) is to update the URDF's with the actual Denavit-Hartenberg parameters downloaded from the UR10's themself. A downside to that solution is that it has to be done for every robot separately, but promisses to give reliable data. 

## ur_pose_updater

Pose updating can be done for multiple robots

This script should be run through a roslaunch 

Run this script through a roslaunch file with a robot_name argument or set the robot_name parameter elsewhere 
      before running the script
'''
INFO: Run this script through a roslaunch file with a robot_name argument or set the robot_name parameter elsewhere 
      before running the script
'''


'''
INFO: Run this script through a roslaunch file with a robot_name argument or set the robot_name parameter elsewhere 
      before running the script
'''


- Can be used to to save, update and delete saved joint and tcp poses for the UR10 robot in separate csv files.
- Message type UpdaterSPToUni holds:
    - action - to take: 
        - TYPE: String
        - 'update': Append or Update a current pose in the list
        - 'delete': Delete a certain pose from the list
        - 'clear': Clear the list of all poses
    - pose_type - that will point to a certain pose list file:
        - TYPE: String
        - 'joint': access manipulation of the Joint Pose list
        - 'tcp': access manipulation of the Tcp Pose list
    - pose_name - that defines the specific pose's name
        - TYPE: String
        - 'pose_name': define pose name
- Message type UpdaterUniToSP holds:
    - got_action - holds last received action command
        - TYPE: String
    - got_pose_type - holds last received pose type
        - TYPE: String
    - got_pose_name - holds last received pose name
        - TYPE: String
    - done_action - generates a string containing the explanation of the last completed action
        - TYPE: String
    - joint_pose_list - holds the list of all saved joint poses
        - TYPE: String[] - List of Strings
    - tcp_pose_list - holds the list of all saved tcp poses
        - TYPE: String[] - List od Strings 

### ur_moveit_unidriver

Uses the moveit pipeline to send move command to the robot. The poses are gathered from the same .csv files where the pose updater has saved them. Move type, pose, velocity and acceleration scaling and some other things are specified in the custom message type. Robot actual pose, and got commands are published.
TODO: Add support for path planning. 

### ur_transformations

Contains three transformations (quaternion to euler, euler to rotational vector and quaternion to roatational vector).

### robotiq_unidriver

Communication with the Robotiq gripper made easier.

