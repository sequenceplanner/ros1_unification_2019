#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # UR Pose Updater (update currently only for joint poses)
    # V.0.3.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import sys
import os
import csv
from moveit_commander import MoveGroupCommander as mgc
from moveit_commander import roscpp_initialize, roscpp_shutdown
from ros1_unification_2019.msg import UpdaterSPToUni
from ros1_unification_2019.msg import UpdaterUniTOSP

class ur_pose_updater():

    def __init__(self):

        roscpp_initialize(sys.argv)
        rospy.init_node('ur_pose_updater', anonymous=False)
        self.robot = mgc("manipulator")
      
        rospy.Subscriber("/unification_roscontrol/updater_sp_to_uni", UpdaterSPToUni, self.sp_callback)
        self.pose_lists_publisher = rospy.Publisher("unification_roscontrol/updater_uni_to_sp", UpdaterUniTOSP, queue_size=10)

        self.file_joint_input = 'ur_joint_poses.csv'
        self.file_tcp_input = 'ur_tcp_poses.csv'
        self.file_oldpose = '_oldpose.csv'
        self.file_newpose = '_newpose.csv'
        self.pose_name = ''
        self.prev_pose_name = ''
        self.pose_type = ''
        
        self.rate = rospy.Rate(10)
       
        rospy.sleep(5)

        self.main()


    def main(self):

        current_pose = UpdaterUniTOSP()

        while not rospy.is_shutdown():
            current_pose.joint_pose_list = self.jpl
            current_pose.tcp_pose_list = self.tpl
            self.pose_lists_publisher.publish(current_pose)
            self.rate.sleep()
            pass
        
        rospy.spin()


    def read_and_publish_joint_list(self):
        joint_pose_list = []
        with open(self.file_joint_input, 'a') as f_in:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                joint_pose_list.append(row[0])
            self.jpl = joint_pose_list
        

    def read_and_publish_tcp_list(self):
        tcp_pose_list = []
        with open(self.file_tcp_input, 'a') as f_in:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                tcp_pose_list.append(row[0])
            self.tpl = tcp_pose_list


    def update_split(self, name, pose):
        with open(self.file_input, "rb") as f_in, open(self.file_oldpose, "wb") as f_op, open(self.file_newpose, "wb") as f_np:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                if row[0] == name:
                    csv.writer(f_np, delimiter = ':').writerow([name, pose])
                else:
                    csv.writer(f_op, delimiter = ':').writerow([row[0], row[1]])

        self.update_merge()
            

    def update_merge(self):
        with open(self.file_newpose, "r") as f_np:
            csv_input = csv.reader(f_np, delimiter=':')
            for row in csv_input:
                with open(self.file_oldpose, "a") as f_op:
                    csv.writer(f_op, delimiter = ':').writerow([row[0], row[1]])
        
        os.remove(self.file_input)
        os.remove(self.file_newpose)
        os.rename(self.file_oldpose, self.file_input)


    def append_new_pose(self, name, pose):
        with open(self.file_joint_input, 'a') as joint_csv_write:
            joint_csv_writer = csv.writer(joint_csv_write, delimiter=':')
            joint_csv_writer.writerow([name, pose])
        
    
    def sp_callback(self, data):
        self.pose_type = data.pose_type
        self.pose_name = data.pose_name

        if self.pose_name == "reset":
            self.prev_pose_name = "reset"
        else:
            pass
        
        if self.pose_name != self.prev_pose_name:
            with open(self.file_joint_input, 'r') as joint_csv_read:
                joint_csv_reader = csv.reader(joint_csv_read, delimiter=':')
                if all((row[0] != self.pose_name) for row in joint_csv_reader):
                    self.append_new_pose(self.pose_name, self.robot.get_current_joint_values())
                else:
                    self.update_split(self.pose_name, self.robot.get_current_joint_values())

        else:
            pass


if __name__ == '__main__':
    try:
        ur_pose_updater()
    except rospy.ROSInterruptException:
        pass
