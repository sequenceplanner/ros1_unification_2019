#!/usr/bin/env python

#----------------------------------------------------------------------------------------------------------------------#
# authors, description, version
#----------------------------------------------------------------------------------------------------------------------#
    # Endre Eres
    # UR Pose Unification Driver
    # V.1.2.0.
#----------------------------------------------------------------------------------------------------------------------#

import rospy
import roslib
import rospkg
import numpy
import ast
import sys
import time
import csv
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from transformations import transformations
from moveit_commander import MoveGroupCommander as mgc
from moveit_commander import roscpp_initialize, roscpp_shutdown
from ros1_unification_2019.msg import Common
from ros1_unification_2019.msg import URPoseSPToUni
from ros1_unification_2019.msg import URPoseSPToUniRicochet as Ricochet
from ros1_unification_2019.msg import URPoseUniToSP

class ur_pose_unidriver(transformations):
    '''
    Moving the UR10 robot to the demanded poses that are saved in according csv files.On the local roshub, one robot 
    should be defined and in the multirobot scenario unique robot names should be used. NEW IN 1.2.0.: Since getting
    the current actual pose from moveit_commander is very slow, saving poses should be done in both JOINT and TCP
    spaces so that act pose comparisson can be done only in JOINT space. TODO: add support for urscript:movel to 
    joint pose and movej to tcp pose. 
    '''

    def __init__(self):

        # Moveit Commander initializer:
        roscpp_initialize(sys.argv)

        # Getting the robot_name parameter from the parameter server (maybe shouldn't be set to global):
        self.robot_name_param = rospy.get_param('robot_name')

        # ROS node initializer:
        rospy.init_node('ur_' + self.robot_name_param + '_pose_unidriver', anonymous=False)

        # Move Group specifier:
        self.robot = mgc("manipulator")
      
        # Subscribers and Publishers:
        rospy.Subscriber("/unification_roscontrol/ur_pose_unidriver_sp_to_uni", URPoseSPToUni, self.sp_callback)
        rospy.Subscriber("/joint_states", JointState, self.jointCallback)
        self.main_publisher = rospy.Publisher("unification_roscontrol/ur_" \
            + self.robot_name_param + "_pose_unidriver_uni_to_sp", URPoseUniToSP, queue_size=10)
        self.urScriptPublisher = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)

        # ROS package localizer:
        self.rospack = rospkg.RosPack()

        # Pose file destinations:
        self.file_joint_input = self.rospack.get_path('ros1_unification_2019') + '/poses/ur_' \
                                                                               + self.robot_name_param \
                                                                               + '_joint_poses.csv'
        self.file_tcp_input = self.rospack.get_path('ros1_unification_2019') + '/poses/ur_' \
                                                                             + self.robot_name_param \
                                                                             + '_tcp_poses.csv'

        # Init tf listener
        # self.tf_listener = tf.TransformListener()
        
        # Switcher lists of cases for implementing a switch-case like behavior:
        self.robot_type_cases = ['UR10', 'IIWA7']
        self.ur10_robot_name_cases = ['TARS', 'KIPP', 'CASE']
        self.iiwa7_robot_name_cases = ['PLEX']
        self.all_robot_names = self.ur10_robot_name_cases + self.iiwa7_robot_name_cases
        self.pose_type_cases = ['JOINT', 'TCP']
        self.action_type_cases = ['MOVEJ', 'MOVEL', 'PLANNED']
        self.file_input_cases = [self.file_joint_input, self.file_tcp_input]

        # Message freshness definition in seconds:
        self.message_freshness = 3

        # Message type initializers:
        self.pose = Pose()
        self.joints = JointState()
        self.main_msg = URPoseUniToSP()
        self.common_msg = Common()
        self.ricochet_msg = Ricochet()

        # Timeout and Stopwatch initializers:
        self.callback_timeout = time.time()
        self.start = time.time()

        # Pose check tolerance initializers:
        self.joint_tol = 0.01
        self.tcp_tol = 0.01

        # Common message value initializers:
        self.robot_name = ''
        self.fresh_msg = False
        self.t_plus = ''
        self.got_reset = False
        self.error_list = []

        # URPoseSpToUni message value initializers:
        self.action = ''
        self.robot_type = ''
        self.robot_name = ''
        self.pose_type = ''
        self.pose_name = ''
        self.speed_scaling = 0.0
        self.acc_scaling = 0.0
        self.goal_tolerance = 0.0

        # Ricochet Message value initializers:
        self.got_action = ''
        self.got_robot_type = ''
        self.got_robot_name = ''
        self.got_pose_type = ''
        self.got_pose_name = ''
        self.got_speed_scaling = ''
        self.got_acc_scaling = ''
        self.got_goal_tolerance = ''

        # Main message value initializers:
        self.moving = False
        self.actual_pose = ''

        # Other:
        self.prev_pose_name = ''
        self.prev_stat_pose = ''
        self.joint_pose = []

        # Error handler value initializers:
        self.action_method_error = ''
        self.robot_type_error = ''
        self.robot_name_error = ''
        self.pose_type_error = ''
        self.pose_name_error = ''
        self.pose_length_error = ''

         # Tick inhibitor:
        self.tick_inhibited = False
        self.prev_action = ''
        self.prev_robot_name = ''
        self.prev_robot_type = ''
        self.prev_pose_type = ''
        self.prev_pose_name = ''
        self.prev_speed_scaling = 0.0
        self.prev_acc_scaling = 0.0
        self.prev_goal_tolerance = 0.0

        # Robot joint identifiers:
        self.joints.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', \
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        # Robot link idenifiers:
        self.ur10_links = ['base_link', 'shoulder_link', 'elbow_link', 'wrist_1_link', \
                           'wrist_2_link', 'wrist_3_link', 'tool0', 'ee_link']
        self.iiwa7_links = ['iiwa7_link_0', 'iiwa7_link_1', 'iiwa7_link_2', 'iiwa7_link_3', \
                            'iiwa7_link_4', 'iiwa7_link_5', 'iiwa7_link_6', 'iiwa7_link_ee']
        
         # Publisher rates:
        self.main_pub_rate = rospy.Rate(10)
       
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
            self.ricochet_msg.got_pose_type = self.pose_type
            self.ricochet_msg.got_pose_name = self.pose_name
            self.ricochet_msg.got_speed_scaling = str('%.3f' % self.speed_scaling)
            self.ricochet_msg.got_acc_scaling = str('%.3f' % self.acc_scaling)
            self.ricochet_msg.got_goal_tolerance = str('%.3f' % self.goal_tolerance)

            # Construct the whole message:
            self.main_msg.info = self.common_msg
            self.main_msg.ricochet = self.ricochet_msg
            self.main_msg.moving = self.moving
            self.main_msg.actual_pose = self.get_static_joint_pose()
            
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
                                self.pose_type_error,
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


    def list_to_pose(self, list):
        '''
        Transform a list into a Pose() type
        '''

        self.pose.position.x = list[0]
        self.pose.position.y = list[1]
        self.pose.position.z = list[2]
        self.pose.orientation.x = list[3]
        self.pose.orientation.y = list[4]
        self.pose.orientation.z = list[5]
        self.pose.orientation.w = list[6]
        return self.pose


    def find_pose(self, name, input_f):
        '''
        Returns the saved pose that matches the pose name
        '''

        pose = []
        with open(input_f, 'r') as f_in:
            csv_reader = csv.reader(f_in, delimiter=':')
            for row in csv_reader:
                if name == row[0]:
                    pose = ast.literal_eval(row[1])
                    break
                else:
                    pass

        if pose != []:
            self.pose_name_error = ''
            return pose
        else:
            self.pose_name_error = 'pose with the name ' + name + ' not saved'
            return []


    def movej(self, input_f, name, a, v):
        '''
        Currently only for joint poses.
        Using the URScript API, move to a position with a linear in joint-space move.
        '''
        joint_pose = []
        joint_pose = self.find_pose(name, input_f)

        if len(joint_pose) == 6:
            self.pose_length_error = ''
            script_str = "movej(" + str(joint_pose) + ", a=" + str(a) + ", v=" + str(v) + ", t=" + str(0) \
            + ", r=" + str(0) + ")"
            self.urScriptPublisher.publish(script_str)
        else:
            self.pose_length_error = 'invalid joint pose length' 


    def movel(self, input_f, name, a, v):
        '''
        Currently only for tcp poses.
        Using the URScript API, move to a position with a linear in tool-space move.
        '''

        quat_pose = []
        tcp_pose = []
        quat_pose = self.find_pose(name, input_f)
                
        if quat_pose != []:
            tcp_pose = self.quat_to_rot(quat_pose[0], quat_pose[1], quat_pose[2],quat_pose[3], quat_pose[4], quat_pose[5], quat_pose[6])
            if len(tcp_pose) == 6:
                self.pose_length_error = ''
                script_str = "movel(p" + str(tcp_pose) + ", a=" + str(a) + ", v=" + str(v) + ", t=" + str(0) + ")"
                self.urScriptPublisher.publish(script_str)
            else:
                self.pose_length_error = 'invalid tcp pose length'
        else:
            self.pose_name_error = 'pose with the name ' + name + ' not saved'

    
    def planned_move(self, input_f, name, pose_type, a, v):
        '''
        Using the movegroup_commander class from MoveIt, plan and execute a trajectory to a joint or a tcp pose
        '''
        pose = self.find_pose(name, input_f)

        self.robot.set_max_velocity_scaling_factor(self.speed_scaling)
        self.robot.set_max_acceleration_scaling_factor(self.acc_scaling)
        self.robot.set_goal_tolerance(self.goal_tolerance)


        if pose_type == "JOINT":
            self.joints.position = pose
            self.robot.go(self.joints, wait = False)
            rospy.sleep(1)
        elif pose_type == "TCP":
            quat_pose = self.list_to_pose(pose)

            #(self.trans, self.rot) = self.tf_listener.lookupTransform('/world', '/ENGINE', rospy.Time(0))
            #engineInWorld=fromTranslationRotation(self.trans, self.rot)
            #goalInInEngine=fromTranslationRotation(quat_pose.position, quat_pose.orientation)

            #print(engineInWorld*goalInInEngine)

            #self.rot_vec = self.trans.append
            self.robot.go(quat_pose, wait = False)
            rospy.sleep(1)
        else:
            pass

    
    def get_static_joint_pose(self):
        '''
        While all joint velocities are 0, compare current robot joint pose with saved poses in the joint_csv file
        and return the name of the saved pose if they match with a tolerance, else return unknown as the 
        current joint pose. Using joint_callback because it is very slow to get_current_joint_values via 
        moveit_commander according to KCacheGrind.
        '''

        # Very slow: current_pose = self.robot.get_current_joint_values()
        current_pose = self.joint_pose

        with open(self.file_joint_input, 'r') as joint_csv:
            joint_csv_reader = csv.reader(joint_csv, delimiter=':')
            for row in joint_csv_reader:
                saved_pose = ast.literal_eval(row[1])
                if all(numpy.isclose(current_pose[i], saved_pose[i], atol=self.joint_tol) for i in range(0, 5)):
                    actual_joint_pose = row[0]
                    break
                else:
                    actual_joint_pose = "UNKNOWN"
                    pass
        
        return actual_joint_pose


    def get_static_tcp_pose(self):
        '''
        While all joint velocities are 0, compare current robot tcp pose with saved poses in the tcp_csv file
        and return the name of the saved pose if they match with a tolerance, else return unknown as the 
        current tcp pose. NOT USED CURRENTLY.
        '''

        current_pose = self.pose_to_list(self.robot.get_current_pose("ee_link"))
            
        with open(self.file_tcp_input, 'r') as tcp_csv:
            tcp_csv_reader = csv.reader(tcp_csv, delimiter=':')
            for row in tcp_csv_reader:
                saved_pose = ast.literal_eval(row[1])
                if all(numpy.isclose(current_pose[i], saved_pose[i], atol=self.tcp_tol) for i in range(0, 6)):
                    actual_tcp_pose = row[0]
                    break
                else:
                    actual_tcp_pose = "UNKNOWN"
                    pass
        
        return actual_tcp_pose


    def generate_current_pose(self):
        '''
        NOT USED CURRENTLY. 
        '''

        actual_pose = ''
        actual_joint_pose = self.get_static_joint_pose()
        actual_tcp_pose = self.get_static_tcp_pose()

        if self.moving == False:
            if actual_joint_pose == "UNKNOWN":
                actual_pose = actual_tcp_pose
                self.prev_stat_pose = actual_pose
            else:
                actual_pose = actual_joint_pose
                self.prev_stat_pose = actual_pose

        else:
            # maybe prev_prev because prev and act is equalized after cmd
            actual_pose = 'moving from ' + self.prev_stat_pose + ' to ' + self.pose_name
        
        return actual_pose


    def action_method_switch(self, action_case):
        '''
        Call an action method based on the case number.
        '''

        if action_case == 0:
            return self.movej(self.file_input_cases[self.pose_case],
                              self.pose_name,
                              self.acc_scaling,
                              self.speed_scaling)

        elif action_case == 1:
            return self.movel(self.file_input_cases[self.pose_case],
                              self.pose_name,
                              self.acc_scaling,
                              self.speed_scaling)

        elif action_case == 2:
            return self.planned_move(self.file_input_cases[self.pose_case],
                                     self.pose_name,
                                     self.pose_type,
                                     self.acc_scaling,
                                     self.speed_scaling)
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

    
    def inhibit_tick(self):
        '''
        Check if two successive messages are the same and disallow consumption if True.
        This method assigns values to the previous message variables so that they can be compared later.
        '''

        self.prev_action = self.action
        self.prev_robot_name = self.robot_name
        self.prev_robot_type = self.robot_type
        self.prev_pose_type = self.pose_type
        self.prev_pose_name = self.pose_name
        self.prev_speed_scaling = self.speed_scaling
        self.prev_acc_scaling = self.acc_scaling
        self.prev_goal_tolerance = self.goal_tolerance


    def sp_callback(self, data):
        '''
        Evaluate and consume the command message from Sequence Planner
        '''

        # Assigning for remote reuse
        self.action = data.action
        self.robot_name = data.robot_name
        self.robot_type = data.robot_type
        self.pose_type = data.pose_type
        self.pose_name = data.pose_name
        self.speed_scaling = data.speed_scaling
        self.acc_scaling = data.acc_scaling
        self.goal_tolerance = data.goal_tolerance

        # Tick inhibitor msg check:
        if self.action == self.prev_action and \
           self.robot_name == self.prev_robot_name and \
           self.robot_type == self.prev_robot_type and \
           self.pose_type == self.prev_pose_type and \
           self.pose_name == self.prev_pose_name and \
           self.speed_scaling == self.prev_speed_scaling and \
           self.acc_scaling == self.prev_acc_scaling and \
           self.goal_tolerance == self.prev_goal_tolerance:
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

                    # Pose type switching
                    if self.pose_type in self.pose_type_cases:
                        self.pose_type_switch_error = ""
                        self.pose_case = self.switcher(self.pose_type, self.pose_type_cases)
                    else:
                        self.pose_type_switch_error = "pose type: " + self.pose_type + " not valid"

                    # Action type switching
                    if self.action in self.action_type_cases:
                        self.action_method_switch_error = ""
                        self.action_case = self.switcher(self.action, self.action_type_cases)
                    else:
                        self.action_method_switch_error = "action: " + self.action + " not valid"

                    # Evaluate messages only once and use the 'reset' flag if stuck
                    if self.tick_inhibited == False:
                        if self.action_method_switch_error == "" and self.pose_type_switch_error == "":
                            self.inhibit_tick()
                            self.action_method_switch(self.action_case)
                        else:
                            pass
                    else:
                        pass
                else:
                    self.done_action = "SKIPPED"
            else:
                self.robot_name_error = "UR10 robot name: " + data.robot_name + " not valid"
                self.done_action = "SKIPPED"

        elif self.robot_type == "UR10" and self.robot_name not in self.all_robot_names:
            self.robot_name_error = "robot name: " + self.robot_name + " not valid"
            self.done_action = "SKIPPED"

        elif self.robot_type == "IIWA7" and self.robot_name not in self.all_robot_names:
            self.robot_name_error = "robot name: " + self.robot_name + " not valid"
            self.done_action = "SKIPPED"

        elif self.robot_type == "UR10" and self.robot_name in self.iiwa7_robot_name_cases:
            self.robot_name_error = "robot name: " + self.robot_name + " does not match type " + self.robot_type
            self.done_action = "SKIPPED"
            
        elif self.robot_type == "IIWA7" and self.robot_name in self.ur10_robot_name_cases:
            self.robot_name_error = "robot name: " + self.robot_name + " does not match type " + self.robot_type
            self.done_action = "SKIPPED"
        
        elif self.robot_type not in self.robot_type_cases:
            self.robot_type_error = "robot type: " + self.robot_type + " not valid"
            self.done_action = "SKIPPED"
    

    def jointCallback(self, joint):
        '''
        Check if the arm is moving and aid in transport pose generation
        '''

        self.joint_pose = joint.position


        if len(joint.velocity) != 0:
            if all(joint.velocity[i] == 0 for i in range(0, 5, 1)):
                self.moving = False
            else:
                self.moving = True
        else:
            self.moving = False


if __name__ == '__main__':
    try:
        ur_pose_unidriver()
    except rospy.ROSInterruptException:
        pass