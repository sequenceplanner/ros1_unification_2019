#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Robot Pose Updater
    # V.1.1.0.
#----------------------------------------------------------------------------------------


import rospy
import roslib
import rospkg
import time
import sys
import os
import csv
from moveit_commander import MoveGroupCommander as mgc
from moveit_commander import roscpp_initialize, roscpp_shutdown
from ros1_unification_2019.msg import Common
from ros1_unification_2019.msg import PoseUpdaterSPToUni
from ros1_unification_2019.msg import PoseUpdaterUniToSP


class pose_updater():
    '''
    Updating the Robot poses and saving them in according csv files.
    These poses can be later fetched by other nodes.
    '''

    def __init__(self):

        # General initialisers:
        roscpp_initialize(sys.argv)
        rospy.init_node('pose_updater', anonymous=False)

        # Moveit Commander initializers:
        self.robot = mgc("manipulator")
      
        # Subscribers and Publishers:
        rospy.Subscriber("/unification_roscontrol/pose_updater_sp_to_uni", PoseUpdaterSPToUni, self.sp_callback)
        self.main_publisher = rospy.Publisher("unification_roscontrol/pose_updater_uni_to_sp", PoseUpdaterUniToSP, queue_size=10)

        # ROS package localizer:
        self.rospack = rospkg.RosPack()

        # Pose file destinations:
        self.file_joint_input = self.rospack.get_path('ros1_unification_2019') + '/poses/ur_joint_poses.csv'
        self.file_tcp_input = self.rospack.get_path('ros1_unification_2019') + '/poses/ur_tcp_poses.csv'
        self.file_joint_oldpose = self.rospack.get_path('ros1_unification_2019') + '/poses/_joint_oldpose.csv'
        self.file_joint_newpose = self.rospack.get_path('ros1_unification_2019') + '/poses/_joint_newpose.csv'
        self.file_tcp_oldpose = self.rospack.get_path('ros1_unification_2019') + '/poses/_tcp_oldpose.csv'
        self.file_tcp_newpose = self.rospack.get_path('ros1_unification_2019') + '/poses/_tcp_newpose.csv'
        
        # UR10 link idenifiers:
        self.ur10_links = ['base_link', 'shoulder_link', 'elbow_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'tool0', 'ee_link']

        # Message freshness definition in seconds:
        self.message_freshness = 3

        # Message type initializers:
        self.main_msg = PoseUpdaterUniToSP()
        self.common_msg = Common()

        # Initialize timeout and stopwatch
        self.callback_timeout = time.time()
        self.start = time.time()

        # Message value initializers:
        self.fresh_msg = False
        self.t_plus = ''
        self.got_reset = False
        self.error_list = []
        self.pose_name = ''
        self.prev_pose_name = ''
        self.pose_type = ''
        self.action = ''
        self.done_action = ''
        self.got_pose_name = ''
        self.got_pose_type = ''
        self.got_action = ''

        # Error handler value Initializers:
        self.action_method_switch_error = "none"
        self.pose_type_switch_error = "none"
        
        # Publisher rates:
        self.main_pub_rate = rospy.Rate(10)
       
        # Some time to assure initialization:
        rospy.sleep(3)

        # Main loop method call:
        self.main()


    def main(self):
        '''
        This method spins until an interrupt exception is raised.
        The state message is generated here and published to the
        main publisher topic.
        '''

        while not rospy.is_shutdown():

            # Check message freshness:
            if time.time() < self.callback_timeout:
                self.fresh_msg = True
            else:
                self.fresh_msg = False

            # Construct common message part:
            self.common_msg.fresh_msg = self.fresh_msg
            self.common_msg.t_plus = str( "%.1f" % self.timer_elapsed()) + " seconds since last msg"
            self.common_msg.got_reset = self.got_reset
            self.common_msg.error_list = self.generate_error_message()

            # Construct the whole message:
            self.main_msg.state = self.common_msg
            self.main_msg.got_action = self.action
            self.main_msg.got_pose_type = self.pose_type
            self.main_msg.got_pose_name = self.pose_name
            self.main_msg.last_done_action = self.done_action
            self.main_msg.joint_pose_list = self.read_and_generate_pose_list(self.file_joint_input)
            self.main_msg.tcp_pose_list = self.read_and_generate_pose_list(self.file_tcp_input)
            
            # Publish the message and sleep a bit:
            self.main_publisher.publish(self.main_msg)
            self.main_pub_rate.sleep()
        
        rospy.spin()


    def timer_start(self):
        '''
        Start the timer when a message arrives.
        '''

        self.start = time.time()
        return self.start


    def timer_elapsed(self):
        '''
        Measure elapsed time since the last message arrived.
        '''

        return time.time() - self.start


    def pose_to_list(self, pose):
        '''
        Transform a Pose() type to a list.
        '''

        pose_list = [0, 0, 0, 0, 0, 0, 0]
        pose_list[0] = pose.pose.position.x
        pose_list[1] = pose.pose.position.y 
        pose_list[2] = pose.pose.position.z 
        pose_list[3] = pose.pose.orientation.x
        pose_list[4] = pose.pose.orientation.y
        pose_list[5] = pose.pose.orientation.z
        pose_list[6] = pose.pose.orientation.w
        return pose_list


    def generate_error_message(self):
        '''
        Collect all the current errors and generate an error list.
        '''

        error_list = []

        if self.action_method_switch_error == self.pose_type_switch_error:
            error_list.append(self.pose_type_switch_error)
        else:
            error_list.append(self.action_method_switch_error)
            error_list.append(self.pose_type_switch_error)
        return error_list


    def delete_pose(self, input_f, newpose_f, pose_name, pose_type):
        '''
        Delete one pose from a pose list.
        '''
        
        with open(input_f, "rb") as f_in, open(newpose_f, "wb") as f_np:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                if row[0] != pose_name:
                    csv.writer(f_np, delimiter = ':').writerow(row)
                else:
                    pass
        
        os.remove(input_f)
        os.rename(newpose_f, input_f)
        self.done_action = 'deleted: ' + str(pose_name) + ' from ' + str(pose_type) + ' list.'


    def clear_pose_list(self, input_f, newpose_f, pose_type):
        '''
        Delete all poses from a pose list.
        '''
        
        with open(input_f, "rb") as f_in, open(newpose_f, "wb") as f_np:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                if row[0] == 'control_pose':
                    csv.writer(f_np, delimiter = ':').writerow(row)
                else:
                    pass

        os.remove(input_f)
        os.rename(newpose_f, input_f)
        self.done_action = 'cleared: ' + str(pose_type) + ' list.'


    def update_pose_list(self, input_f, oldpose_f, newpose_f, name, pose_type, function):
        '''
        Update a pose list by appending a new pose or updating an existing one.
        '''
        
        with open(input_f, 'r') as csv_read:
            csv_reader = csv.reader(csv_read, delimiter=':')
            if all((row[0] != name) for row in csv_reader):
                self.append_new_pose(input_f, name, function) # Function gets pose in this case
                self.done_action = 'appended: ' + str(name) + ' to ' + str(pose_type) + ' list.'
            else:
                self.update_split(input_f, oldpose_f, newpose_f, name, function)
                self.done_action = 'updated: ' + str(name) + ' in ' + str(pose_type) + ' list.'

        
    def read_and_generate_pose_list(self, input_f):
        '''
        Acquire the names of all saved poses from a file to a list.
        '''

        pose_list = []
        with open(input_f, 'r') as f_in:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                pose_list.append(row[0])
            return pose_list


    def update_split(self, input_f, oldpose_f, newpose_f, name, pose):
        '''
        Update the values of an existing pose in a pose list by
        saving it in a newpose file. The poses that are not being
        updated are saved in a oldpose list and those two lists 
        are merged in the update_merge method. 
        '''

        with open(input_f, "rb") as f_in, open(oldpose_f, "wb") as f_op, open(newpose_f, "wb") as f_np:
            csv_input = csv.reader(f_in, delimiter=':')
            for row in csv_input:
                if row[0] == name:
                    csv.writer(f_np, delimiter = ':').writerow([name, pose])
                else:
                    csv.writer(f_op, delimiter = ':').writerow([row[0], row[1]])

        self.update_merge(input_f, newpose_f, oldpose_f)


    def update_merge(self, input_f, newpose_f, oldpose_f):
        '''
        Merge an olpose and a newpose list and thus form an updated pose list. 
        '''

        with open(newpose_f, "r") as f_np:
            csv_input = csv.reader(f_np, delimiter=':')
            for row in csv_input:
                with open(oldpose_f, "a") as f_op:
                    csv.writer(f_op, delimiter = ':').writerow([row[0], row[1]])
        
        os.remove(input_f)
        os.remove(newpose_f)
        os.rename(oldpose_f, input_f)


    def append_new_pose(self, file, name, pose):
        '''
        Append a nonexisting pose to a pose list.
        '''

        with open(file, 'a') as csv_append:
            csv_appender = csv.writer(csv_append, delimiter=':')
            csv_appender.writerow([name, pose])


    def action_method_switch(self, action_case):
        '''
        Call an action method based on the case number.
        '''

        if action_case == 0:
            return self.update_pose_list(self.file_input_cases[self.pose_case],
                                         self.file_oldpose_cases[self.pose_case],
                                         self.file_newpose_cases[self.pose_case],
                                         self.pose_name,
                                         self.pose_type,
                                         self.pose_type_switch(self.pose_case))

        elif action_case == 1:
            return self.delete_pose(self.file_input_cases[self.pose_case],
                                    self.file_newpose_cases[self.pose_case],
                                    self.pose_name,
                                    self.pose_type)

        elif action_case == 2:
            return self.clear_pose_list(self.file_input_cases[self.pose_case],
                                        self.file_newpose_cases[self.pose_case],
                                        self.pose_type)
        else:
            pass

            

    def pose_type_switch(self, pose_case):
        '''
        Fetch a current pose depending on the pose type.
        '''

        if pose_case == 0:
            return self.robot.get_current_joint_values()
        elif pose_case == 1:
            return self.pose_to_list(self.robot.get_current_pose("ee_link"))
        else:
            pass


    def sp_callback(self, data):
        '''
        Evaluate and use the command message from Sequence Planner
        '''

        # Refreshing the message and restarting the stopwatch
        self.callback_timeout = time.time() + self.message_freshness
        self.timer_start()

        # Assigning for remote reuse
        self.action = data.action
        self.pose_type = data.pose_type
        self.pose_name = data.pose_name

        # Check for the 'reset' flag
        if self.pose_name == "reset":
            self.prev_pose_name = "reset"
        else:
            pass

        # Switcher lists of cases for implementing a switch-case like behavior
        self.pose_type_cases = ['joint', 'tcp']
        self.action_type_cases = ['update', 'delete', 'clear']
        self.file_input_cases = [self.file_joint_input, self.file_tcp_input]
        self.file_oldpose_cases = [self.file_joint_oldpose, self.file_tcp_oldpose]
        self.file_newpose_cases = [self.file_joint_newpose, self.file_tcp_newpose]

        # Pose type switching
        if self.pose_type in self.pose_type_cases:
            self.pose_type_switch_error = "none"
            self.pose_case = self.switcher(self.pose_type, self.pose_type_cases)
        else:
            self.pose_type_switch_error = "pose type: " + self.pose_type + " not valid."

        # Action type switcher
        if self.action in self.action_type_cases:
            self.action_method_switch_error = "none"
            self.action_case = self.switcher(self.action, self.action_type_cases)
        else:
            self.action_method_switch_error = "action: " + self.action + " not valid."

        # Evaluate messages only once and use the 'reset' flag if stuck
        if self.pose_name != self.prev_pose_name:
            if self.action_method_switch_error == "none" and self.pose_type_switch_error == "none":
                self.action_method_switch(self.action_case)
            else:
                pass
        else:
            pass
            

    def switcher(self, what, case_list):
        '''
        Implementing a switch-case like behavior with switcher lists
        '''

        for i in range(0, len(case_list), 1):
            if what == case_list[i]:
                return i
                break
            else:
                pass


if __name__ == '__main__':
    try:
        pose_updater()
    except rospy.ROSInterruptException:
        pass