#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # UR Pose Updater
    # V.1.0.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import rospkg
import sys
import os
import csv
from moveit_commander import MoveGroupCommander as mgc
from moveit_commander import roscpp_initialize, roscpp_shutdown
from ros1_unification_2019.msg import UpdaterSPToUni
from ros1_unification_2019.msg import UpdaterUniToSP

class ur_pose_updater():

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

        self.action = ''
        self.pose_type = ''
        self.pose_name = ''
        self.done_action = ''
        
        self.rate = rospy.Rate(10)
       
        rospy.sleep(5)

        self.main()


    def main(self):

        msg = UpdaterUniToSP()

        while not rospy.is_shutdown():
            msg.got_action = self.action
            msg.got_pose_type = self.pose_type
            msg.got_pose_name = self.pose_name
            msg.done_action = self.done_action
            msg.joint_pose_list = self.read_and_publish_pose_list(self.file_joint_input)
            msg.tcp_pose_list = self.read_and_publish_pose_list(self.file_tcp_input)
            self.pose_lists_publisher.publish(msg)
            self.rate.sleep()
            pass
        
        rospy.spin()


    def pose_to_list(self, pose):
        pose_list = [0, 0, 0, 0, 0, 0, 0]
        pose_list[0] = pose.pose.position.x
        pose_list[1] = pose.pose.position.y 
        pose_list[2] = pose.pose.position.z 
        pose_list[3] = pose.pose.orientation.x
        pose_list[4] = pose.pose.orientation.y
        pose_list[5] = pose.pose.orientation.z
        pose_list[6] = pose.pose.orientation.w
        return pose_list


    def delete_pose(self, input_f, newpose_f, pose):
        with open(input_f, "rb") as f_in, open(newpose_f, "wb") as f_np:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                if row[0] != pose:
                    csv.writer(f_np, delimiter = ':').writerow(row)
                else:
                    pass
        
        os.remove(input_f)
        os.rename(newpose_f, input_f)


    def clear_pose_list(self, input_f, newpose_f):
        with open(input_f, "rb") as f_in, open(newpose_f, "wb") as f_np:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                if row[0] == 'control_pose':
                    csv.writer(f_np, delimiter = ':').writerow(row)
                else:
                    pass

        os.remove(input_f)
        os.rename(newpose_f, input_f)

        
    def read_and_publish_pose_list(self, input_f):
        pose_list = []
        with open(input_f, 'r') as f_in:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                pose_list.append(row[0])
            return pose_list


    def update_split(self, input_f, oldpose_f, newpose_f, name, pose):
        with open(input_f, "rb") as f_in, open(oldpose_f, "wb") as f_op, open(newpose_f, "wb") as f_np:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                if row[0] == name:
                    csv.writer(f_np, delimiter = ':').writerow([name, pose])
                else:
                    csv.writer(f_op, delimiter = ':').writerow([row[0], row[1]])

        self.update_merge(input_f, newpose_f, oldpose_f)


    def update_merge(self, input_f, newpose_f, oldpose_f):
        with open(newpose_f, "r") as f_np:
            csv_input = csv.reader(f_np, delimiter=':')
            for row in csv_input:
                with open(oldpose_f, "a") as f_op:
                    csv.writer(f_op, delimiter = ':').writerow([row[0], row[1]])
        
        os.remove(input_f)
        os.remove(newpose_f)
        os.rename(oldpose_f, input_f)


    def append_new_pose(self, file, name, pose):
        with open(file, 'a') as csv_append:
            csv_appender = csv.writer(csv_append, delimiter=':')
            csv_appender.writerow([name, pose])


    def sp_callback(self, data):
        self.action = data.action
        self.pose_type = data.pose_type
        self.pose_name = data.pose_name

        act_pose_list = [0, 0, 0, 0, 0, 0, 0]

        if self.pose_name == "reset":
            self.prev_pose_name = "reset"
        else:
            pass

        pose_types = ['joint', 'tcp']
        file_input = [self.file_joint_input, self.file_tcp_input]
        file_oldpose = [self.file_joint_oldpose, self.file_tcp_oldpose]
        file_newpose = [self.file_joint_newpose, self.file_tcp_newpose]
        poser = [self.robot.get_current_joint_values(), self.pose_to_list(self.robot.get_current_pose("ee_link"))]

        pose_case = self.switcher(self.pose_type, pose_types)

        #this wouldn't make much sense, but is still possible...
        #action_types = ['append', 'update', 'delete', 'clear']
        #actioner = [self.append_new_pose(file_input[pose_case], self.pose_name, poser[pose_case]),
        #            self.update_split(file_input[pose_case], file_oldpose[pose_case], file_newpose[pose_case], self.pose_name, poser[pose_case]),
        #            self.delete_pose(file_input[pose_case], self.pose_name),
        #            self.clear_pose_list(file_input[pose_case])]
        #action_case = self.switcher(self.action, action_types)

        if self.pose_name != self.prev_pose_name:
            with open(file_input[pose_case], 'r') as csv_read:
                csv_reader = csv.reader(csv_read, delimiter=':')
                if self.action == 'update':
                    if all((row[0] != self.pose_name) for row in csv_reader):
                        self.append_new_pose(file_input[pose_case], self.pose_name, poser[pose_case])
                        self.done_action = 'appended: ' + str(self.pose_name) + ' to ' + str(pose_types[pose_case]) + ' list.'
                    else:
                        self.update_split(file_input[pose_case], file_oldpose[pose_case], file_newpose[pose_case], self.pose_name, poser[pose_case])
                        self.done_action = 'updated: ' + str(self.pose_name) + ' in ' + str(pose_types[pose_case]) + ' list.'

                elif self.action == 'delete':
                    self.delete_pose(file_input[pose_case], file_newpose[pose_case], self.pose_name)
                    self.done_action = 'deleted: ' + str(self.pose_name) + ' from ' + str(pose_types[pose_case]) + ' list.'

                elif self.action == 'clear':
                    self.clear_pose_list(file_input[pose_case], file_newpose[pose_case])
                    self.done_action = 'cleared: ' + str(pose_types[pose_case]) + ' list.'
                else:
                    pass

        else:
            pass


    def switcher(self, what, case_list):
        for i in range(0, len(case_list), 1):
            if what == case_list[i]:
                return i
                break
            else:
                pass


if __name__ == '__main__':
    try:
        ur_pose_updater()
    except rospy.ROSInterruptException:
        pass