#!/usr/bin/env python

#----------------------------------------------------------------------------------------------------------------------#
# authors, description, version
#----------------------------------------------------------------------------------------------------------------------#
    # Endre Eres
    # UR Pose Unification Driver
    # V.1.3.0.
#----------------------------------------------------------------------------------------------------------------------#

import rospy
import roslib
import rospkg
import numpy
import ast
import sys
import time
import csv
import os
import tf
import struct
import socket
import threading
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from transformations import transformations
from moveit_commander import MoveGroupCommander as mgc
from moveit_commander import roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import MoveGroupActionFeedback as mgaf
from ros1_unification_2019.msg import Common
from ros1_unification_2019.msg import URPoseSPToUni
from ros1_unification_2019.msg import URPoseSPToUniRicochet as Ricochet
from ros1_unification_2019.msg import URPoseUniToSP
from ros1_unification_2019.msg import PoseUpdaterSPToUni

#HOST = "192.168.10.16"
#HOST = "0.0.0.0"
HOST = rospy.get_param('robot_ip')
PORT = 30003

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
        rospy.init_node(self.robot_name_param + '_ur_pose_unidriver', anonymous=False)

        # Move Group specifier:
        self.robot = mgc("manipulator")
      
        # Subscribers and Publishers:
        rospy.Subscriber("/unification_roscontrol/ur_pose_unidriver_sp_to_uni", URPoseSPToUni, self.sp_callback)
        rospy.Subscriber("joint_states", JointState, self.jointCallback)
        rospy.Subscriber("move_group/feedback", mgaf, self.moveitFdbckCallback)
        self.main_publisher = rospy.Publisher("/unification_roscontrol/ur_TARS_pose_unidriver_uni_to_sp", URPoseUniToSP, queue_size=10)
        self.urScriptPublisher = rospy.Publisher("ur_driver/URScript", String, queue_size=10)
        rospy.Subscriber("unification_roscontrol/gestures_uni_to_sp", String, self.gesture_callback)
        self.gesture_ack = rospy.Publisher("/unification_roscontrol/gestures_sp_to_uni", String, queue_size=10)
	    #self.photoneoPublisher = rospy.Publisher("sp_to_phoxi_loc", String, queue_size=10)


        # ROS package localizer:
        self.rospack = rospkg.RosPack()

        # Pose file destinations:
        self.file_joint_input = self.rospack.get_path('ros1_unification_2019') + '/poses/ur_' \
                                                                               + self.robot_name_param \
                                                                               + '_joint_poses.csv'
        self.file_tcp_input = self.rospack.get_path('ros1_unification_2019') + '/poses/ur_' \
                                                                             + self.robot_name_param \
                                                                             + '_tcp_poses.csv'

        # Switcher lists of cases for implementing a switch-case like behavior:
        self.robot_type_cases = ['UR10', 'IIWA7']
        self.ur10_robot_name_cases = ['TARS', 'KIPP', 'CASE']
        self.iiwa7_robot_name_cases = ['PLEX']
        self.all_robot_names = self.ur10_robot_name_cases + self.iiwa7_robot_name_cases
        self.pose_type_cases = ['JOINT', 'TCP']
        self.action_type_cases = ['MOVEJ', 'MOVEL', 'PLANNED']
        self.file_input_cases = [self.file_joint_input, self.file_tcp_input]

        self.actual_urscript_tcp_pose = ''

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
        self.socket_error = ''

        # Moveit state feedback initializers:
        self.moveit_state = ''
        self.moveit_message = ''

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

        self.once3 = True
        self.once4 = True

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

        self.get_urscript_static_tcp_pose()

        # Check if Robot Name argument is correct:
        if self.robot_name_param in self.ur10_robot_name_cases:

            # Main loop method call:
            self.main()

        else:
            self.common_msg.error_list.append("robot name: " + self.robot_name_param + " not valid.")
            self.main_msg.state = self.common_msg
            self.main_publisher.publish(self.main_msg)


    def run_picknplace(self):
        def callback_run_picknplace():
            os.system('rosrun ros1_unification_2019 picknplace.py')
        t1 = threading.Thread(target=callback_run_picknplace)
        t1.daemon = True
        t1.start()

    def gesture_callback(self, data):
        if data.data == "COMMAND3" and self.once3 == True:
            time.sleep(4)
            self.movej(self.file_joint_input, "AfterLFOperationJOINTPose", 0.1, 0.1)
            self.once3 = False
        elif data.data == "COMMAND4" and self.once4 == True:
            time.sleep(5)
            self.gesture_ack.publish("CLEAR")
            self.planned_move(self.file_joint_input, "handover", "JOINT", 0.1, 0.1)
            self.once4 = False
        elif data.data == "COMMAND9":
            self.once3 = True
            self.once4 = True
            self.gesture_ack.publish("CLEAR")
        else:
            pass

    def main(self):
        '''
        This method spins until an interrupt exception is raised. The state message is generated here and published to 
        the main publisher topic.
        '''

        #self.threaded_tf_lookup_movel()

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
            self.main_msg.actual_pose = self.generate_current_pose()
            self.main_msg.moveit_state = self.moveit_state 
            self.main_msg.moveit_message = self.moveit_message

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
                                self.robot_name_error,
                                self.socket_error]

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
            if len(quat_pose) == 7:
                self.pose_length_error = 'pose is quat => planning'
            elif len(quat_pose) == 6:
                self.pose_length_error = ''
                script_str = "movel(p" + str(quat_pose) + ", a=" + str(a) + ", v=" + str(v) + ", t=" + str(0) + ")"
                self.urScriptPublisher.publish(script_str)
            else:
                self.pose_length_error = 'invalid tcp pose length'
        else:
            self.pose_name_error = 'pose with the name ' + name + ' not saved'


    def threaded_tf_lookup_movel(self):
        def threaded_tf_lookup_movel_callback():

            listener = tf.TransformListener()
            unset = False
            trans = []
            rot = []
            while not rospy.is_shutdown():
                try:
                
                    (trans, rot) = listener.lookupTransform('/world', '/ee_link', rospy.Time(0))
                    #print(trans, rot)
                    
                    # if type(trans) == list and type(rot) == list and unset == False:
# 
                    #    ? self.robot.set_max_velocity_scaling_factor(self.speed_scaling)
                    #  self.robot.set_max_acceleration_scaling_factor(self.acc_scaling)
                        # self.robot.set_goal_tolerance(self.goal_tolerance)
# 
                        # quaternion = (rot[0], rot[1], rot[2], rot[3])
                        # euler_pose = tf.transformations.euler_from_quaternion(quaternion)
                        # euler_list = [trans[0], trans[1], trans[2], euler_pose[0], euler_pose[1], euler_pose[2]]
# 
                        # print(euler_pose)
# 
                        # script_str = "movel(p" + str(euler_list) + ", a=" + str(self.acc_scaling) + ", v=" + str(self.speed_scaling) + ", t=" + str(0) + ")"
                        # self.urScriptPublisher.publish(script_str)
                        # rospy.sleep(1)
                        # unset = True
                    #    
                    # else:
                        # pass

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): 
                    continue

        t = threading.Thread(target=threaded_tf_lookup_movel_callback)
        t.daemon = True
        t.start()



    def threaded_tf_lookup(self):
        def threaded_tf_lookup_callback():

            listener = tf.TransformListener()
            unset = False
            trans = []
            rot = []
            while not rospy.is_shutdown() and not unset:
                for i in [0, 1, 2, 3, 4, 5]:
                    try:
                    
                        (trans, rot) = listener.lookupTransform('/world', '/HandRight' + str(i), rospy.Time(0))

                        if type(trans) == list and type(rot) == list and unset == False:
                            pose_app = [trans[0], trans[1] - 0.3, trans[2], 0.31393227071831764, 0.31763498116667394, -0.6335004244729144, -0.6318478933521307]
                            print(pose_app)
                            self.robot.set_max_velocity_scaling_factor(0.1) # terrible hacks
                            self.robot.set_max_acceleration_scaling_factor(0.1)
                            self.robot.set_goal_tolerance(0.01)

                            quat_pose = self.list_to_pose(pose_app)
                            #print(quat_pose)
                            print("MOVING TO POSE")
                            ret = self.robot.go(quat_pose, wait = True)
                            print("MOVE COMPLETED: ")
                            print(ret)
                            if(ret):
                                handover_pose_saver = rospy.Publisher("/unification_roscontrol/ur_pose_updater_sp_to_uni", PoseUpdaterSPToUni, queue_size=10)
                                save_handover_psoe = PoseUpdaterSPToUni()
                                save_handover_psoe.action = "UPDATE"
                                save_handover_psoe.robot_type = "UR10"
                                save_handover_psoe.robot_name = "TARS"
                                save_handover_psoe.pose_name = "handover"
                                handover_pose_saver.publish(save_handover_psoe)
                            
                            unset = True

                            break

                        else:
                            pass
#   
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): 
                        continue

        t = threading.Thread(target=threaded_tf_lookup_callback)
        t.daemon = True
        t.start()

    
    def planned_move(self, input_f, name, pose_type, a, v):
        '''
        Using the movegroup_commander class from MoveIt, plan and execute a trajectory to a joint or a tcp pose
        '''

        if name == 'handover':
            self.threaded_tf_lookup()

        elif name == "LFOperation":
            self.run_picknplace()           

        else:
            pose = self.find_pose(name, input_f)

            self.robot.set_max_velocity_scaling_factor(self.speed_scaling)
            self.robot.set_max_acceleration_scaling_factor(self.acc_scaling)
            self.robot.set_goal_tolerance(self.goal_tolerance)

            self.pose_length_error = ''
            self.pose_type_error = ''

            if pose_type == "JOINT":
                if len(pose) == 6:
                    self.joints.position = pose
                    self.robot.go(self.joints, wait = False)
                    rospy.sleep(1)
                else:
                    self.pose_length_error = 'Invalid pose length'
                    pass
            elif pose_type == "TCP":
                if len(pose) == 7:
                    quat_pose = self.list_to_pose(pose)
                    self.robot.go(quat_pose, wait = False)
                    rospy.sleep(1)
                else:
                    self.pose_length_error = 'pose is rotvec => urscript movel'
                    pass
            else:
                self.pose_type_error = 'invalid pose_type'
                pass
    


    def get_static_joint_pose(self):
        '''
        While all joint velocities are 0, compare current robot joint pose with saved poses in the joint_csv file
        and return the name of the saved pose if they match with a tolerance, else return unknown as the 
        current joint pose. Using joint_callback because it is very slow to get_current_joint_values via 
        moveit_commander according to KCacheGrind.
        '''

        actual_joint_pose = ""
        current_pose = self.joint_pose

        with open(self.file_joint_input, 'r') as joint_csv:
            joint_csv_reader = csv.reader(joint_csv, delimiter=':')
            for row in joint_csv_reader:
                if len(ast.literal_eval(row[1])) == 6 and current_pose != []:
                    saved_pose = ast.literal_eval(row[1])
                    if all(numpy.isclose(current_pose[i], saved_pose[i], atol=self.joint_tol) for i in range(0, 6)):
                        actual_joint_pose = row[0]
                        break
                    else:
                        actual_joint_pose = "UNKNOWN"
                        pass
                else:
                    pass
        
        return actual_joint_pose


    def get_moveit_static_tcp_pose(self):
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
                if all(numpy.isclose(current_pose[i], saved_pose[i], atol=self.tcp_tol) for i in range(0, 7)):
                    actual_tcp_pose = row[0]
                    break
                else:
                    actual_tcp_pose = "UNKNOWN"
                    pass
        
        return actual_tcp_pose


    def get_urscript_static_tcp_pose(self):
        '''
        Using the client interface port 30003 to read cartesian TCP pose from the UR directly.
        It is very slow, so it is put in a separate thread.
        '''

        

        def socket_tcp_callback():

            self.socket_error = ""
            tcp_pose = []
            actual_tcp_pose = ''
            print("asdf")

            while(1):

                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(10)
                    s.connect((HOST, PORT))
                    time.sleep(0.1)

                    # Drop the first 11 packets...
                    packet_1 = s.recv(4)
                    packet_2 = s.recv(8)
                    packet_3 = s.recv(48)
                    packet_4 = s.recv(48)
                    packet_5 = s.recv(48)
                    packet_6 = s.recv(48)
                    packet_7 = s.recv(48) 
                    packet_8 = s.recv(48)
                    packet_9 = s.recv(48)
                    packet_10 = s.recv(48)
                    packet_11 = s.recv(48)

                    packet_12 = s.recv(8)
                    packet_12 = packet_12.encode("hex") #convert the data from \x hex notation to plain hex
                    x = str(packet_12)
                    x = struct.unpack('!d', packet_12.decode('hex'))[0]

                    packet_13 = s.recv(8)
                    packet_13 = packet_13.encode("hex") #convert the data from \x hex notation to plain hex
                    y = str(packet_13)
                    y = struct.unpack('!d', packet_13.decode('hex'))[0]

                    packet_14 = s.recv(8)
                    packet_14 = packet_14.encode("hex") #convert the data from \x hex notation to plain hex
                    z = str(packet_14)
                    z = struct.unpack('!d', packet_14.decode('hex'))[0]

                    packet_15 = s.recv(8)
                    packet_15 = packet_15.encode("hex") #convert the data from \x hex notation to plain hex
                    Rx = str(packet_15)
                    Rx = struct.unpack('!d', packet_15.decode('hex'))[0]

                    packet_16 = s.recv(8)
                    packet_16 = packet_16.encode("hex") #convert the data from \x hex notation to plain hex
                    Ry = str(packet_16)
                    Ry = struct.unpack('!d', packet_16.decode('hex'))[0]

                    packet_17 = s.recv(8)
                    packet_17 = packet_17.encode("hex") #convert the data from \x hex notation to plain hex
                    Rz = str(packet_17)
                    Rz = struct.unpack('!d', packet_17.decode('hex'))[0]

                    tcp_pose = [x, y, z, Rx, Ry, Rz]
                    #print(tcp_pose)

                    s.close()

                except (socket.error):
                    self.socket_error = "Socket conn to UR wrong"
                    pass

                
                with open(self.file_tcp_input, 'r') as tcp_csv:
                    tcp_csv_reader = csv.reader(tcp_csv, delimiter=':')
                    for row in tcp_csv_reader:
                        if len(ast.literal_eval(row[1])) == 6 and tcp_pose != []:
                            saved_pose = ast.literal_eval(row[1])
                            #print(saved_pose)
                            #print(tcp_pose)
                            if all(numpy.isclose(tcp_pose[i], saved_pose[i], atol=self.tcp_tol) for i in range(0, 6)):
                                actual_tcp_pose = row[0]
                                break
                            else:
                                actual_tcp_pose = "UNKNOWN"
                                pass
                        else:
                            pass

                self.actual_urscript_tcp_pose = actual_tcp_pose
                
            
        t1 = threading.Thread(target=socket_tcp_callback)
        t1.daemon = True
        t1.start()


    def generate_current_pose(self):
        '''
        Genetates current pose that is sent as act_pos to SP.
        '''

        actual_pose = ''
        actual_joint_pose = self.get_static_joint_pose()
        actual_moveit_tcp_pose = self.get_moveit_static_tcp_pose()
        actual_urscript_tcp_pose = self.actual_urscript_tcp_pose

        if self.moving == False:
            if actual_joint_pose == "UNKNOWN":
                if actual_moveit_tcp_pose == "UNKNOWN":
                    actual_pose = self.actual_urscript_tcp_pose
                    self.prev_stat_pose = actual_pose
                else:
                    actual_pose = actual_moveit_tcp_pose
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
        if self.pose_name == "UNKNOWN":
            self.prev_pose_name = "UNKNOWN"
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
    

    def moveitFdbckCallback(self, data):
        '''
        Check state and last message of moveit
        '''
        self.moveit_state = data.feedback.state
        self.moveit_message = data.status.text


    def jointCallback(self, joint):
        '''
        Check if the arm is moving and aid in transport pose generation
        '''

        self.joint_pose = joint.position


        if len(joint.velocity) != 0:
            if all(abs(joint.velocity[i]) < 0.01 for i in range(0, 5, 1)):
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
