#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # UR MoveIt Unification Driver
    # V.0.6.2.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import rospkg
import numpy
import ast
import sys
import time
import csv
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
#from moveit_python import *
from moveit_commander import MoveGroupCommander as mgc
from moveit_commander import PlanningSceneInterface as psi
from moveit_commander import roscpp_initialize, roscpp_shutdown
from ur_transformations import ur_transformations as urtrans
from ros1_unification_2019.msg import MoveItSPToUni
from ros1_unification_2019.msg import MoveItUniToSP

class ur_moveit_unidriver(urtrans):

    def __init__(self):

        roscpp_initialize(sys.argv)
        rospy.init_node('ur_moveit_unidriver', anonymous=False)
        self.robot = mgc("manipulator")
        self.scene = psi()
        #self.scene = PlanningSceneInterface("base_link")

        rospy.Subscriber("/unification_roscontrol/ur_moveit_sp_to_uni", MoveItSPToUni, self.sp_callback)
        self.to_sp_publisher = rospy.Publisher("/unification_roscontrol/ur_moveit_uni_to_sp", MoveItUniToSP, queue_size=10)

        self.rospack = rospkg.RosPack()

        self.joint_pose_file = self.rospack.get_path('ros1_unification_2019') + '/pose_lists/ur_joint_poses.csv'
        self.tcp_pose_file = self.rospack.get_path('ros1_unification_2019') + '/pose_lists/ur_tcp_poses.csv'
        
        # command
        self.robot_type = ""
        self.should_plan = False
        self.move_type = ""
        self.ref_pos = ""
        self.prev_ref_pos = ""
        self.frame = ""
        self.acc_scaling = 0
        self.velocity_scaling = 0
        self.goal_tolerance = 0.001
        self.quat_tcp_pose = []

        # state
        self.ur_moveit_unidriver_got_msg_from_sp = False
        self.got_robot_type = ""
        self.got_should_plan = False
        self.got_move_type = ""
        self.got_ref_pos = ""
        self.got_frame = ""
        self.got_acc_scaling = 0
        self.got_velocity_scaling = 0
        self.got_goal_tolerance = 0.001
        self.act_pose_rot = []
        self.act_pos = ""
        self.joint_isclose_tolerance = 0.01
        self.tcp_isclose_tolerance = 0.01
        
        self.rate = rospy.Rate(10)
       
        rospy.sleep(5)
        
        #self.robot.set_planning_frame("base")
        #self.robot.set_end_effector_link("tool0_controller")
        self.filename = "/home/crslab/kinetic_ros1_ws/src/ros1_unification_2019/src/pointcloud.stl"

        asdf_pose = [1,1,1,0,0,-0.707,-0.707]

        self.car_pose = PoseStamped()
        self.car_pose.header.frame_id = "world" #rbc.get_planning_frame() #"world"
        self.car_pose.pose.position.x = asdf_pose[0]
        self.car_pose.pose.position.y = asdf_pose[1]
        self.car_pose.pose.position.z = asdf_pose[2]
        self.car_pose.pose.orientation.x = asdf_pose[3]
        self.car_pose.pose.orientation.y = asdf_pose[4]
        self.car_pose.pose.orientation.z = asdf_pose[5]
        self.car_pose.pose.orientation.w = asdf_pose[6]

	#def addMesh(self, name, pose, filename, use_service=True):
        #def add_mesh(self, name, pose, filename, size = (1, 1, 1)):
	#self.scene.addMesh("tstool", self.car_pose, self.filename)
        #self.scene.add_box("LF", self.car_pose, (10, 10, 10)) 
	
	#self.scene.add_mesh("tstool", self.car_pose, self.filename, (1, 1, 1))
		
        #self.scene.add_box()
        print(self.robot.get_planning_frame())
        print(self.robot.get_end_effector_link())

        self.main()


    def main(self):

        self.pub_msg = MoveItUniToSP()    

        act_pose_list = [0, 0, 0, 0, 0, 0, 0]
        while not rospy.is_shutdown():
            
            act_pose_quat = self.robot.get_current_pose("ee_link")
            act_pose_list = [act_pose_quat.pose.position.x, 
                                            act_pose_quat.pose.position.y, 
                                            act_pose_quat.pose.position.z, 
                                            act_pose_quat.pose.orientation.x,
                                            act_pose_quat.pose.orientation.y,
                                            act_pose_quat.pose.orientation.z,
                                            act_pose_quat.pose.orientation.w]
            
            with open(self.tcp_pose_file, 'r') as tcp_csv:
                tcp_csv_reader = csv.reader(tcp_csv, delimiter=':')
                for row in tcp_csv_reader:
                    act_tcp_pos = ast.literal_eval(row[1])
                    if all(numpy.isclose(act_pose_list[i], act_tcp_pos[i], atol=self.tcp_isclose_tolerance) for i in range(0, 6)):
                        self.act_tcp_pos = row[0]
                        break
                    else:
                        self.act_tcp_pos = "unknown"
                        pass

            with open(self.joint_pose_file, 'r') as joint_csv:
                joint_csv_reader = csv.reader(joint_csv, delimiter=':')
                for row in joint_csv_reader:
                    act_joint_pos = ast.literal_eval(row[1])
                    self.joint_pose_list = self.robot.get_current_joint_values()
                    if all(numpy.isclose(self.joint_pose_list[i], act_joint_pos[i], atol=self.joint_isclose_tolerance) for i in range(0, 5)):
                        self.act_joint_pos = row[0]
                        break
                    else:
                        self.act_joint_pos = "unknown"
                        pass

            if self.act_joint_pos == "unknown":
                self.act_pos = self.act_tcp_pos
            else:
                self.act_pos = self.act_joint_pos

            self.pub_msg.act_pos = self.act_pos
            self.to_sp_publisher.publish(self.pub_msg)
            self.rate.sleep()
        
        rospy.spin()
    

    def move(self):

        joints = JointState()
        joints.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        tcp_pose = Pose()

        self.robot.set_max_velocity_scaling_factor(self.speed_scaling)
        self.robot.set_max_acceleration_scaling_factor(self.acc_scaling)
        self.robot.set_goal_tolerance(self.goal_tolerance)

        if self.robot_type == "ur10":
            if self.frame == "local":
                if self.should_plan == False:
                    if self.move_type == "joint":
                        with open(self.joint_pose_file, 'r') as joint_csv:
                            joint_csv_reader = csv.reader(joint_csv, delimiter=':')
                            for row in joint_csv_reader:
                                if row[0] == self.ref_pos:
                                    joints.position = ast.literal_eval(row[1])
                                    self.robot.go(joints, wait = False)
                                    rospy.sleep(1)
                                else:
                                    print("Err1: Demanded ur10 joint pose " + self.ref_pos + " not defined, make sure that the pose is defined in the 'ur_joint_poses.csv' file.")
                                    pass

                    elif self.move_type == "tcp":
                        with open(self.tcp_pose_file, 'r') as tcp_csv:
                            tcp_csv_reader = csv.reader(tcp_csv, delimiter=':')
                            for row in tcp_csv_reader:
                                if row[0] == self.ref_pos:
                                    rot_tcp_pose = ast.literal_eval(row[1])
                                    print(rot_tcp_pose)
                                    tcp_pose.position.x = rot_tcp_pose[0]
                                    tcp_pose.position.y = rot_tcp_pose[1]
                                    tcp_pose.position.z = rot_tcp_pose[2]
                                    tcp_pose.orientation.x = rot_tcp_pose[3]
                                    tcp_pose.orientation.y = rot_tcp_pose[4]
                                    tcp_pose.orientation.z = rot_tcp_pose[5]
                                    tcp_pose.orientation.w = rot_tcp_pose[6]
                                    #self.robot.set_pose_target(rot_tcp_pose, "ee_link")
                                    self.robot.go(tcp_pose, wait = False)
                                    rospy.sleep(1)
                                else:
                                    print("Err2: Demanded ur10 tcp pose " + self.ref_pos + " not defined, make sure that the pose is defined in the 'ur_tcp_poses.csv' file.")
                                    pass
                else:
                    print("Err3: Planning not defined yet.")
                    pass
            else:
                print("Err4: Invalid frame.")
                pass
        else:
            print("Err5: Unknown robot.")
            pass

    def sp_callback(self, data):

        self.ref_pos = data.ref_pos

        if self.ref_pos == "reset":
            self.prev_ref_pos = "reset"
        else:
            pass

        self.robot_cases = ['ur10', 'iiwa7']
        self.robot_case_file_identifier = [self.pose_files_ur, self.pose_files_iiwa]

        self.frame_cases = ['local', 'global']
        
        self.pose_cases = ['joint', 'tcp']
        self.pose_files_ur = [self.joint_pose_file, self.tcp_pose_file]
        self.pose_files_iiwa = [self.file1, self.file2]
        self.move_cases = ['linear_joint', 'linear_tcp', 'planned']

        
        if self.ref_pos != self.prev_ref_pos:
            self.prev_ref_pos = self.ref_pos
            self.frame = data.frame
            self.robot_type = data.robot_type
            self.move_type = data.move_type
            self.speed_scaling = data.speed_scaling
            self.acc_scaling = data.acc_scaling
            self.should_plan = data.should_plan
            self.goal_tolerance = data.goal_tolerance
            self.move()
        else:
            pass


if __name__ == '__main__':
    try:
        ur_moveit_unidriver()
    except rospy.ROSInterruptException:
        pass
