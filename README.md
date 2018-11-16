# ros1_unification_2019

Package for the ROS1 part of the Unification project for 2019.
TODO: Figure out the root cause of tool0_controller pose difference in teaching pendant and tf. A possible solution (Christian - FCC) is to update the URDF with the actual Denavit-Hartenberg parameters downloaded from the UR10 itself. A downside to that solution is that it has to be done for every robot, so the tool0_controller difference issue has priority in investigation.

## ur_pose_updater

Can be used to to save and update joint and tcp poses for the UR10 robot in separate.csv files. Pose type and name are specified in the custom message type. Also, the lists names of saved poses are published.
TODO: add option to delete a certain pose from a list as well as to delete all poses and maybe upgrade with an option to make new lists of poses if needed.

## ur_moveit_unidriver

Uses the moveit pipeline to send move command to the robot. The poses are gathered from the same .csv files where the pose updater has saved them. Move type, pose, velocity and acceleration scaling and some other things are specified in the custom message type. Robot actual pose, and got commands are published.
TODO: Add support for path planning. 

## ur_transformations

Contains three transformations (quaternion to euler, euler to rotational vector and quaternion to roatational vector).
TODO: Verify that the transformations are 100% correct.

## robotiq_unidriver

Communication with the Robotiq gripper made easier.

