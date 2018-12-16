# ros1_unification_2019

## STATUS
### ur_pose_updater - should work
### ur_pose_unidriver - should work (beta, needs more testing)
### scene_updater - not really working (alpha, needs more work)

UPDATE README

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

