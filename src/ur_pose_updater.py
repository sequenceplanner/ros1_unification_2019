#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # UR Pose Updater (update currently only for joint poses)
    # V.0.6.2.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import rospkg
import sys
import os
import csv
from moveit_commander import MoveGroupCommander as mgc
from moveit_commander import roscpp_initialize, roscpp_shutdown
from ur_transformations import ur_transformations as urtrans
from ros1_unification_2019.msg import UpdaterSPToUni
from ros1_unification_2019.msg import UpdaterUniToSP

class ur_pose_updater(urtrans):

    def __init__(self):

        roscpp_initialize(sys.argv)
        rospy.init_node('ur_pose_updater', anonymous=False)
        self.robot = mgc("manipulator")
      
        rospy.Subscriber("/unification_roscontrol/updater_sp_to_uni", UpdaterSPToUni, self.sp_callback)
        self.pose_lists_publisher = rospy.Publisher("unification_roscontrol/updater_uni_to_sp", UpdaterUniToSP, queue_size=10)

        self.rospack = rospkg.RosPack()

        self.file_joint_input = self.rospack.get_path('ros1_unification_2019') + '/pose_lists/ur_joint_poses.csv'
        self.file_tcp_input = self.rospack.get_path('ros1_unification_2019') + '/pose_lists/ur_tcp_poses.csv'
        self.file_joint_oldpose = self.rospack.get_path('ros1_unification_2019') + '/pose_lists/_joint_oldpose.csv'
        self.file_joint_newpose = self.rospack.get_path('ros1_unification_2019') + '/pose_lists/_joint_newpose.csv'
        self.file_tcp_oldpose = self.rospack.get_path('ros1_unification_2019') + '/pose_lists/_tcp_oldpose.csv'
        self.file_tcp_newpose = self.rospack.get_path('ros1_unification_2019') + '/pose_lists/_tcp_newpose.csv'
        self.pose_name = ''
        self.prev_pose_name = ''
        self.pose_type = ''
        
        self.rate = rospy.Rate(10)
       
        rospy.sleep(5)

        self.main()


    def main(self):

        current_pose = UpdaterUniToSP()

        while not rospy.is_shutdown():
            self.read_and_publish_joint_list()
            self.read_and_publish_tcp_list()
            current_pose.joint_pose_list = self.jpl
            current_pose.tcp_pose_list = self.tpl
            self.pose_lists_publisher.publish(current_pose)
            self.rate.sleep()
            pass
        
        rospy.spin()


    def read_and_publish_joint_list(self):
        joint_pose_list = []
        with open(self.file_joint_input, 'r') as f_in:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                joint_pose_list.append(row[0])
            self.jpl = joint_pose_list
        

    def read_and_publish_tcp_list(self):
        tcp_pose_list = []
        with open(self.file_tcp_input, 'r') as f_in:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                tcp_pose_list.append(row[0])
            self.tpl = tcp_pose_list


    def update_joint_split(self, name, pose):
        with open(self.file_joint_input, "rb") as f_in, open(self.file_joint_oldpose, "wb") as f_op, open(self.file_joint_newpose, "wb") as f_np:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                if row[0] == name:
                    csv.writer(f_np, delimiter = ':').writerow([name, pose])
                else:
                    csv.writer(f_op, delimiter = ':').writerow([row[0], row[1]])

        self.update_joint_merge()
    

    def update_tcp_split(self, name, pose):
        with open(self.file_tcp_input, "rb") as f_in, open(self.file_tcp_oldpose, "wb") as f_op, open(self.file_tcp_newpose, "wb") as f_np:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                if row[0] == name:
                    csv.writer(f_np, delimiter = ':').writerow([name, pose])
                else:
                    csv.writer(f_op, delimiter = ':').writerow([row[0], row[1]])

        self.update_tcp_merge()
            

    def update_joint_merge(self):
        with open(self.file_joint_newpose, "r") as f_np:
            csv_input = csv.reader(f_np, delimiter=':')
            for row in csv_input:
                with open(self.file_joint_oldpose, "a") as f_op:
                    csv.writer(f_op, delimiter = ':').writerow([row[0], row[1]])
        
        os.remove(self.file_joint_input)
        os.remove(self.file_joint_newpose)
        os.rename(self.file_joint_oldpose, self.file_joint_input)

    
    def update_tcp_merge(self):
        with open(self.file_tcp_newpose, "r") as f_np:
            csv_input = csv.reader(f_np, delimiter=':')
            for row in csv_input:
                with open(self.file_tcp_oldpose, "a") as f_op:
                    csv.writer(f_op, delimiter = ':').writerow([row[0], row[1]])
        
        os.remove(self.file_tcp_input)
        os.remove(self.file_tcp_newpose)
        os.rename(self.file_tcp_oldpose, self.file_tcp_input)


    def append_new_joint_pose(self, name, pose):
        with open(self.file_joint_input, 'a') as joint_csv_write:
            joint_csv_writer = csv.writer(joint_csv_write, delimiter=':')
            joint_csv_writer.writerow([name, pose])
    
    
    def append_new_tcp_pose(self, name, pose):
        with open(self.file_tcp_input, 'a') as joint_csv_write:
            joint_csv_writer = csv.writer(joint_csv_write, delimiter=':')
            joint_csv_writer.writerow([name, pose])
        
    
    def sp_callback(self, data):
        self.pose_type = data.pose_type
        self.pose_name = data.pose_name

        print(self.pose_name)

        if self.pose_name == "reset":
            self.prev_pose_name = "reset"
        else:
            pass

        if self.pose_type == "joint":
            if self.pose_name != self.prev_pose_name:
                with open(self.file_joint_input, 'r') as joint_csv_read:
                    joint_csv_reader = csv.reader(joint_csv_read, delimiter=':')
                    if all((row[0] != self.pose_name) for row in joint_csv_reader):
                        self.append_new_joint_pose(self.pose_name, self.robot.get_current_joint_values())
                    else:
                        self.update_joint_split(self.pose_name, self.robot.get_current_joint_values())

            else:
                pass

        elif self.pose_type == "tcp":
            if self.pose_name != self.prev_pose_name:
                with open(self.file_tcp_input, 'r') as tcp_csv_read:
                    tcp_csv_reader = csv.reader(tcp_csv_read, delimiter=':')
                    if all((row[0] != self.pose_name) for row in tcp_csv_reader):

                        act_pose_quat = self.robot.get_current_pose("tool0_controller")
                        act_pose_rot = self.quat_to_rot(act_pose_quat.pose.position.x, 
                                                        act_pose_quat.pose.position.y, 
                                                        act_pose_quat.pose.position.z, 
                                                        act_pose_quat.pose.orientation.x,
                                                        act_pose_quat.pose.orientation.y,
                                                        act_pose_quat.pose.orientation.z,
                                                        act_pose_quat.pose.orientation.w)
                     
                        self.append_new_tcp_pose(self.pose_name, act_pose_rot)
                    else:
                        self.update_tcp_split(self.pose_name, act_pose_rot)

            else:
                pass


if __name__ == '__main__':
    try:
        ur_pose_updater()
    except rospy.ROSInterruptException:
        pass
