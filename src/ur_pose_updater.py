#!/usr/bin/env python

#----------------------------------------------------------------------------------------------------------------------#
# authors, description, version
#----------------------------------------------------------------------------------------------------------------------#
    # Endre Eres
    # Universal Robots UR10 Pose Updater
    # V.1.3.1.
#----------------------------------------------------------------------------------------------------------------------#

import rospy
import roslib
import rospkg
import time
import sys
import os
import csv
import tf
from moveit_commander import MoveGroupCommander as mgc
from moveit_commander import roscpp_initialize, roscpp_shutdown
from ros1_unification_2019.msg import Common
from ros1_unification_2019.msg import SavedPoses
from ros1_unification_2019.msg import PoseUpdaterSPToUni
from ros1_unification_2019.msg import PoseUpdaterSPToUniRicochet as Ricochet
from ros1_unification_2019.msg import PoseUpdaterUniToSP


class ur_pose_updater():
    '''
    Updating the Robot poses and saving them in according csv files. These poses can be later fetched by other nodes. 
    On the local roshub, one robot should be defined and in the multirobot scenario unique robot names should be used.
    Since getting the current actual pose from moveit_commander is very slow, saving poses should be done in both 
    JOINT and TCP spaces so that act pose comparisson can be done only in JOINT space.
    '''

    def __init__(self):

        # Moveit Commander initializer:
        roscpp_initialize(sys.argv)

        # Getting the robot_name parameter from the parameter server:
        self.robot_name_param = rospy.get_param('/robot_name')

        # ROS node initializer:
        rospy.init_node(self.robot_name_param + '_ur_pose_updater', anonymous=False)

        # Move Group specifier:
        self.robot = mgc("manipulator")
      
        # Subscribers and Publishers:
        rospy.Subscriber("unification_roscontrol/ur_pose_updater_sp_to_uni", PoseUpdaterSPToUni, self.sp_callback)
        self.main_publisher = rospy.Publisher("unification_roscontrol/ur_pose_updater_uni_to_sp", PoseUpdaterUniToSP, queue_size=10)

        # ROS package localizer:
        self.rospack = rospkg.RosPack()

        # Pose file destinations:
        self.file_joint_input = self.rospack.get_path('ros1_unification_2019') + '/poses/ur_' \
                                                                               + self.robot_name_param \
                                                                               + '_joint_poses.csv'
        self.file_tcp_input = self.rospack.get_path('ros1_unification_2019') + '/poses/ur_' \
                                                                             + self.robot_name_param \
                                                                             + '_tcp_poses.csv'
        self.file_joint_oldpose = self.rospack.get_path('ros1_unification_2019') + '/poses/_' \
                                                                                 + self.robot_name_param \
                                                                                 + '_joint_oldpose.csv'
        self.file_joint_newpose = self.rospack.get_path('ros1_unification_2019') + '/poses/_' \
                                                                                 + self.robot_name_param \
                                                                                 + '_joint_newpose.csv'
        self.file_tcp_oldpose = self.rospack.get_path('ros1_unification_2019') + '/poses/_' \
                                                                               + self.robot_name_param \
                                                                               + '_tcp_oldpose.csv'
        self.file_tcp_newpose = self.rospack.get_path('ros1_unification_2019') + '/poses/_' \
                                                                               + self.robot_name_param \
                                                                               + '_tcp_newpose.csv'
        
        # Robot link idenifiers:
        self.ur10_links = ['base_link', 'shoulder_link', 'elbow_link', 'wrist_1_link', \
                           'wrist_2_link', 'wrist_3_link', 'tool0', 'ee_link']
        self.iiwa7_links = ['iiwa7_link_0', 'iiwa7_link_1', 'iiwa7_link_2', 'iiwa7_link_3', \
                            'iiwa7_link_4', 'iiwa7_link_5', 'iiwa7_link_6', 'iiwa7_link_ee']

        # Switcher lists of cases for implementing a switch-case like behavior:
        self.robot_type_cases = ['UR10', 'IIWA7']
        self.ur10_robot_name_cases = ['TARS', 'KIPP', 'CASE']
        self.iiwa7_robot_name_cases = ['PLEX']
        self.all_robot_names = self.ur10_robot_name_cases + self.iiwa7_robot_name_cases
        self.pose_type_cases = ['JOINT', 'TCP']
        self.action_type_cases = ['UPDATE', 'DELETE', 'CLEAR']
        self.file_input_cases = [self.file_joint_input, self.file_tcp_input]
        self.file_oldpose_cases = [self.file_joint_oldpose, self.file_tcp_oldpose]
        self.file_newpose_cases = [self.file_joint_newpose, self.file_tcp_newpose]

        # Message freshness definition in seconds:
        self.message_freshness = 3

        # Message type initializers:
        self.main_msg = PoseUpdaterUniToSP()
        self.common_msg = Common()
        self.poses_msg = SavedPoses()
        self.ricochet_msg = Ricochet()

        # Timeout and Stopwatch initializers:
        self.callback_timeout = time.time()
        self.start = time.time()

        # Common message value initializers:
        self.robot_name = ''
        self.fresh_msg = False
        self.t_plus = ''
        self.got_reset = False
        self.error_list = []

        # Saved poses message value initializers:
        self.joint_pose_list = []
        self.tcp_pose_list = []

        # Pose updater command mesage value initializers:
        self.action = ''
        self.robot_type = ''
        self.robot_name = ''
        self.pose_name = ''

        # Command ricochet message value initializers:
        self.got_action = ''
        self.got_robot_type = ''
        self.got_robot_name = ''
        self.got_pose_name = ''

        # Pose updater state message value initializers:
        self.last_completed_action = ''

        # Error handler value Initializers:
        self.action_method_error = ''
        self.robot_type_error = ''
        self.robot_name_error = ''
        self.pose_name_error = ''
        
        # Publisher rates:
        self.main_pub_rate = rospy.Rate(10)

        # Tick inhibitor:
        self.tick_inhibited = False
        self.prev_action = ''
        self.prev_robot_name = ''
        self.prev_robot_type = ''
        self.prev_pose_name = ''
       
        # Some time to assure initialization:
        rospy.sleep(3)

        # Check if Robot Name argument is correct:
        if self.robot_name_param in self.ur10_robot_name_cases:

            # Main loop method call:
            self.main()

        else:
            self.common_msg.error_list.append("robot name: " + self.robot_name_param + " not valid.")
            self.main_msg.state = self.common_msg
            self.main_publisher.publish(self.main_msg)


    def main(self):
        '''
        This method spins until an interrupt exception is raised. The state message is generated here and published to 
        the main publisher topic.
        '''

        while not rospy.is_shutdown():

            # Check message freshness:
            if time.time() < self.callback_timeout:
                self.fresh_msg = True
            else:
                self.fresh_msg = False

            # Construct common message part:
            self.common_msg.robot_name = self.robot_name_param
            self.common_msg.fresh_msg = self.fresh_msg
            self.common_msg.t_plus = self.timer_elapsed()
            self.common_msg.got_reset = self.got_reset
            self.common_msg.error_list = self.generate_error_message()

            # Contruct the ricochet message part:
            self.ricochet_msg.got_action = self.action
            self.ricochet_msg.got_robot_type = self.robot_type
            self.ricochet_msg.got_robot_name = self.robot_name
            self.ricochet_msg.got_pose_name = self.pose_name

            # Construct saved poses message part:
            self.poses_msg.joint_pose_list = self.read_and_generate_pose_list(self.file_joint_input)
            self.poses_msg.tcp_pose_list = self.read_and_generate_pose_list(self.file_tcp_input)

            # Construct the whole message:
            self.main_msg.info = self.common_msg
            self.main_msg.ricochet = self.ricochet_msg
            self.main_msg.saved_poses = self.poses_msg
            self.main_msg.last_completed_action = self.last_completed_action
            
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

        return str('%.1f' % (time.time() - self.start)) + " seconds"


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

    
    def generate_error_message(self):
        '''
        Collect all the current errors and generate an error list.
        '''

        error_list = []
        potential_error_list = [self.action_method_error,
                                self.pose_name_error,
                                self.robot_type_error,
                                self.robot_name_error]

        if all((err == '') for err in potential_error_list):
            error_list = []
        else:
            pass
        
        for err in potential_error_list:
            if err != '':
                error_list.append(err)
            else:
                pass
            
        return error_list


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


    def delete_pose(self, input_f, newpose_f, pose_name):
        '''
        Delete one pose from pose lists.
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
        self.last_completed_action = 'deleted: ' + str(pose_name)


    def clear_pose_list(self, input_f, newpose_f):
        '''
        Delete all poses from pose lists.
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
        self.last_completed_action = 'pose lists cleared'


    def update_pose_list(self, input_f, oldpose_f, newpose_f, name, function):
        '''
        Update a pose list by appending a new pose or updating an existing one.
        '''
        
        with open(input_f, 'r') as csv_read:
            csv_reader = csv.reader(csv_read, delimiter=':')
            if all((row[0] != name) for row in csv_reader):
                self.append_new_pose(input_f, name, function) # Function gets pose in this case
                self.last_completed_action = 'appended: ' + str(name)
            else:
                self.update_split(input_f, oldpose_f, newpose_f, name, function)
                self.last_completed_action = 'updated: ' + str(name)

        
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
        Update the values of an existing pose in a pose list by saving it in a newpose file. The poses that are not 
        being updated are saved in a oldpose list and those two lists are merged in the update_merge method. 
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
        Call an action method based on the case number.Doesn't make much sense since
        both tcp and joint poses are saved each time.
        '''

        if action_case == 0:
            for i in [0, 1]:
                self.update_pose_list(self.file_input_cases[i],
                                      self.file_oldpose_cases[i],
                                      self.file_newpose_cases[i],
                                      self.pose_name,
                                      self.pose_type_switch(i))

        elif action_case == 1:
            for i in [0, 1]:
                self.delete_pose(self.file_input_cases[i],
                                 self.file_newpose_cases[i],
                                 self.pose_name)

        elif action_case == 2:
            for i in [0, 1]:
                self.clear_pose_list(self.file_input_cases[i],
                                     self.file_newpose_cases[i])
        else:
            pass


    def pose_type_switch(self, pose_case):
        '''
        Fetch a current pose depending on the pose type. Doesn't make much sense since
        both tcp and joint poses are saved each time.
        '''

        if pose_case == 0:
            return self.robot.get_current_joint_values()
        elif pose_case == 1:
            return self.pose_to_list(self.robot.get_current_pose("tool0"))
        else:
            pass

    
    def inhibit_tick(self):
        '''
        Check if two successive messages are the same and disallow consumption if True.
        This method assigns values to the previous message variables so that they can be compared later.
        '''

        self.prev_action = self.action
        self.prev_robot_name = self.robot_name
        self.prev_robot_type = self.robot_type
        self.prev_pose_name = self.pose_name



    def sp_callback(self, data):
        '''
        Evaluate and consume the command message from Sequence Planner
        '''

        # Assigning for remote reuse
        self.action = data.action
        self.robot_name = data.robot_name
        self.robot_type = data.robot_type
        self.pose_name = data.pose_name

        # Tick inhibitor msg check:
        if self.action == self.prev_action and \
           self.robot_name == self.prev_robot_name and \
           self.robot_type == self.prev_robot_type and \
           self.pose_name == self.prev_pose_name:
            self.tick_inhibited = True
        else:
            self.tick_inhibited = False

        # Check for the 'reset' flag
        if self.pose_name == "RESET":
            self.prev_pose_name = "RESET"
            self.got_reset = True
        else:
            self.got_reset = False

        # Check if according robot type:
        if self.robot_type == "UR10" and self.pose_name != "RESET":

            # Clearing robot type error:
            self.robot_type_error = ""

            # Check if valid robot name
            if self.robot_name in self.ur10_robot_name_cases:

                # Clearing robot name error:
                self.robot_name_error = ""
            
                # Addressing only if the script is true for the name
                if self.robot_name == self.robot_name_param:

                    # Refreshing the message and restarting the stopwatch
                    self.callback_timeout = time.time() + self.message_freshness
                    self.timer_start()

                    # Action type switching
                    if self.action in self.action_type_cases:
                        self.action_method_switch_error = ""
                        self.action_case = self.switcher(self.action, self.action_type_cases)
                    else:
                        self.action_method_switch_error = "action: " + self.action + " not valid"

                    # Evaluate messages only once and use the 'reset' flag if stuck:
                    if self.tick_inhibited == False:
                        if self.action_method_switch_error == "":
                            self.inhibit_tick()
                            self.action_method_switch(self.action_case)
                        else:
                            pass
                    else:
                        pass
                else:
                    self.last_completed_action = "SKIPPED"
            else:
                self.robot_name_error = "UR10 robot name: " + data.robot_name + " not valid"
                self.last_completed_action = "SKIPPED"

        elif self.robot_type == "UR10" and self.robot_name not in self.all_robot_names:
            self.robot_name_error = "robot name: " + self.robot_name + " not valid"
            self.last_completed_action = "SKIPPED"

        elif self.robot_type == "IIWA7" and self.robot_name not in self.all_robot_names:
            self.robot_name_error = "robot name: " + self.robot_name + " not valid"
            self.last_completed_action = "SKIPPED"

        elif self.robot_type == "UR10" and self.robot_name in self.iiwa7_robot_name_cases:
            self.robot_name_error = "robot name: " + self.robot_name + " does not match type " + self.robot_type
            self.last_completed_action = "SKIPPED"
            
        elif self.robot_type == "IIWA7" and self.robot_name in self.ur10_robot_name_cases:
            self.robot_name_error = "robot name: " + self.robot_name + " does not match type " + self.robot_type
            self.last_completed_action = "SKIPPED"
        
        elif self.robot_type not in self.robot_type_cases:
            self.robot_type_error = "robot type: " + self.robot_type + " not valid"
            self.last_completed_action = "SKIPPED"


if __name__ == '__main__':
    try:
        ur_pose_updater()
    except rospy.ROSInterruptException:
        pass