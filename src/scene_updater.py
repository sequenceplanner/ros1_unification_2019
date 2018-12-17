#!/usr/bin/env python

#----------------------------------------------------------------------------------------------------------------------#
# authors, description, version
#----------------------------------------------------------------------------------------------------------------------#
    # Endre Eres
    # Unification Scene Updater
    # V.0.9.8. alpha
#----------------------------------------------------------------------------------------------------------------------#

import rospy
import roslib
import rospkg
import time
import sys
import os
import csv
from moveit_commander import MoveGroupCommander as mgc
from moveit_commander import PlanningSceneInterface as psi
from moveit_commander import roscpp_initialize, roscpp_shutdown
from transformations import transformations 
from ros1_unification_2019.msg import Common
from ros1_unification_2019.msg import SceneUpdaterSPToUni
from ros1_unification_2019.msg import SceneUpdaterSPToUniRicochet as Ricochet
from ros1_unification_2019.msg import SceneUpdaterUniToSP
from ros1_unification_2019.msg import PsiObjectPose


class scene_updater(transformations):
    '''
    Updating the Scene by manipulating collision objects
    using the moveit_commanders PlanningSceneInterface class.
    The inherited transformations class provides methods for
    coordinate tranformations.
    '''
    def __init__(self):

        # General initialisers:
        roscpp_initialize(sys.argv)
        rospy.init_node('scene_updater', anonymous=False)

        # Moveit Commander initializers:
        self.robot = mgc("manipulator")
        self.scene = psi()
      
        # Subscribers and Publishers:
        rospy.Subscriber("/unification_roscontrol/scene_updater_sp_to_uni", SceneUpdaterSPToUni, self.sp_callback)
        self.main_publisher = rospy.Publisher("unification_roscontrol/scene_updater_uni_to_sp", \
                              SceneUpdaterUniToSP, queue_size=10)

        # ROS package localizer:
        self.rospack = rospkg.RosPack()

        # Stl mesh destinations:
        self.lf_mesh = self.rospack.get_path('ros1_unification_2019') + '/cad_meshes/LF.stl'
        self.engine_mesh = self.rospack.get_path('ros1_unification_2019') + '/cad_meshes/engine_reduced.stl'
        self.agv_mesh = self.rospack.get_path('ros1_unification_2019') + '/cad_meshes/AGV.stl'
        self.ts_tool_mesh = self.rospack.get_path('ros1_unification_2019') + '/unstruct_ptcl_meshes/tstool.stl'

        # UR10 link idenifiers:
        self.ur10_links = ['base_link', 'shoulder_link', 'elbow_link', 'wrist_1_link', '
                            'wrist_2_link', 'wrist_3_link', 'tool0', 'ee_link']

        # Switcher lists of cases for implementing a switch-case like behavior:
        self.object_action_cases = ['ADD', 'REMOVE', 'MOVE', 'CLEAR', 'ATTACH', 'DETACH']
        self.object_name_cases = ['LF', 'TSTOOL', 'AGV', 'ENGINE'] # Gradually add more...
        self.object_file_cases = [self.lf_mesh, self.ts_tool_mesh, self.agv_mesh, self.engine_mesh]

        # Message freshness definition in seconds:
        self.message_freshness = 3

        # Message type initializers:
        self.main_msg = SceneUpdaterUniToSP()
        self.obj_msg = PsiObjectPose()
        self.common_msg = Common()
        self.ricochet_msg = Ricochet()
        self.psi_msg = PsiObjectPose()

        # Initialize timeout and stopwatch
        self.callback_timeout = time.time()
        self.start = time.time()

        # Common message value initializers:
        self.robot_name = ''
        self.fresh_msg = False
        self.t_plus = ''
        self.got_reset = False
        self.error_list = []

        # PsiObjectPose message value initializers:
        self.scene_object_name = ''
        self.scene_object_pose = []

        #SceneUpdaterSPToUni message value initializers:
        self.object_action = ''
        self.object_name = ''
        self.euler_pose = []

        # Ricochet message value initializers:
        self.got_object_action = ''
        self.got_object_name = ''
        self.got_euler_pose = []

        # Main message value initializers:
        self.attached_objects = []
        self.object_poses = []

        # Other:
        self.prev_object_name = ''

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

        print(self.scene.get_attached_objects())

        while not rospy.is_shutdown():

            # Check message freshness:
            if time.time() < self.callback_timeout:
                self.fresh_msg = True
            else:
                self.fresh_msg = False

            # Construct common message part:
            self.common_msg.robot_name = '' # add read from param later
            self.common_msg.fresh_msg = self.fresh_msg
            self.common_msg.t_plus = self.timer_elapsed()
            self.common_msg.got_reset = self.got_reset
            self.common_msg.error_list = [] #self.generate_error_message()
            
            # Construct the ricochet message part:
            self.ricochet_msg.got_object_action = self.object_action
            self.ricochet_msg.got_object_name = self.object_name
            self.ricochet_msg.got_euler_pose = self.euler_pose

            #Construct the PsiOvjectPose message part:
            self.psi_msg.scene_object_name = self.scene_object_name 
            self.psi_msg.scene_object_pose = self.scene_object_pose 

            # Construct the whole message:
            self.main_msg.state = self.common_msg
            self.main_msg.ricochet = self.ricochet_msg
            self.main_msg.attached_objects = [] # self.attached_objects()
            self.main_msg.object_poses = [] #self.object_poses()
            
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


    def attached_objects(self):
        '''
        Check if there are any attached collision objects 
        and return the list of those object.
        '''

        object_list = []
        object_list.append(str(self.scene.get_attached_objects()))

        return object_list

    
    def object_poses(self):
        '''
        Check if there are any collision objects in the
        current planning scene and return the list 
        of those objects.
        '''

        object_list = []
        objects_in_scene = list(self.scene.get_objects())
        for obj in objects_in_scene:
            self.obj_msg.object_name = obj
            self.obj_msg.object_pose = self.quat_to_rot(self.get_object_poses(obj))
            object_list.append(self.obj_msg)
        return object_list

    
    def add_object(self, file, name, pose):
        '''
        Add a collision object to the planning scene from
        the list of object name cases
        '''

        if name in self.object_name_cases:

            quat = self.rpy_to_quat(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5])

            scene_pose.header.frame_id = "world"
            scene_pose.pose.position.x = quat[0]
            scene_pose.pose.position.y = quat[1]
            scene_pose.pose.position.z = quat[2]
            scene_pose.pose.orientation.x = quat[3]
            scene_pose.pose.orientation.y = quat[4]
            scene_pose.pose.orientation.z = quat[5]
            scene_pose.pose.orientation.w = quat[6]

            self.scene.add_mesh(name, quat, file, (1, 1, 1))
            self.error = "none"

        elif name != '' and name not in self.object_name_cases:
            self.error = "Object named: " + name + " not valid."
        else:
            pass


    def remove_object(self, name):
        '''
        Remove an existing collision object from the planning scene
        '''

        if name in list(self.get_objects()):
            self.scene.remove_world_object(name)
            self.error = "none"
        elif name != '' and name not in list(self.get_objects()) and name in self.object_name_cases:
            self.error = "Object named: " + name + " not existing in the scene."
        elif name != '' and name not in self.object_name_cases:
            self.error = "Object named: " + name + " not valid."
        else:
            pass

    
    def move_object(self, file, name, pose):
        '''
        Move and existing object in the scene to a noew pose
        '''

        self.scene.remove_object(file, name)
        self.add_object(file, name, pose)


    def clear_scene(self):
        '''
        Remove all collision objects from the scene
        '''

        self.scene.remove_world_object()

    
    def attach_object(self, name, link):
        '''
        Attach a collision object to a specified link
        '''

        if name in list(self.get_objects()) and link in self.links:
            self.robot.attach_object(name, link)
            self.error = "none"
        elif name in list(self.get_objects()) and not link in self.ur10_links:
            self.error = "Link named: " + link + " not valid."
        elif name not in list(self.get_objects()):
            self.error = "Object named: " + name + "not valid."
        else:
            pass

    
    def detach_object(self, name):
        '''
        Detach a specific collision object from the whole move group
        '''

        if name in list(self.get_attached_objects()):
            self.robot.detach_object(name)
            self.error = "none"
        elif name not in list(self.get_attached_objects()) and name in self.get_objects() and name in self.object_name_cases:
            self.error = "Object named: " + name + "not attached."
        elif name not in self.get_objects() and name in self.object_name_cases:
            self.error = "Object named: " + name + "not in the scene."
        elif name not in self.object_name_cases:
            self.error = "Object named: " + name + "not valid."
        else:
            pass

    def action_method_switch(self, action_case):
        '''
        Call an action method based on the case number.
        '''

        if action_case == 0:
            return self.add_object(self.object_name, 
                                   self.object_file_cases[self.object_name_case], 
                                   self.euler_pose)

        elif action_case == 1:
            return self.remove_object(self.object_name)

        elif action_case == 2:
            return self.move_object(self.object_name, 
                                    self.euler_pose)

        elif action_case == 3:
            return self.clear_scene()

        elif action_case == 4:
            return self.attach_object(self.object_name, 
                                      self.robot.get_end_effector_link())

        elif action_case == 5:
            return self.detach_object(self.object_name)

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
        Evaluate and consume the command message from Sequence Planner
        '''

        # Assigning for remote reuse
        self.object_action = data.object_action 
        self.object_name = data.object_name
        self.euler_pose = data.euler_pose

        # Check for the 'reset' flag (doesn't make much sense but needed against the SP ping)
        if self.object_name == "RESET":
            self.prev_object_name = "RESET"
            self.got_reset = True
        else:
            self.got_reset = False        

        # Object name switching
        if self.object_name in self.object_name_cases:
            self.object_name_switch_error = ""
            self.object_name_case = self.switcher(self.object_name, self.object_name_cases)
        else:
            self.object_name_switch_error = "object name: " + self.object_name + " not valid"

         # Action type switching
        if self.object_action in self.object_action_cases:
            self.action_method_switch_error = ""
            self.object_action_case = self.switcher(self.object_action, self.object_action_cases)
        else:
            self.action_method_switch_error = "action: " + self.action + " not valid"

        # Evaluate messages only once and use the 'reset' flag if stuck
        if self.object_name != self.prev_object_name:
            if self.action_method_switch_error == "" and self.object_name_switch_error == "":
                self.prev_object_name = self.object_name
                self.action_method_switch(self.object_action_case)
            else:
                pass

        
    def switcher(self, what, case_list):
        '''
        Implementing a switch-case like behavior with switcher lists
        '''

        for i in range(0, len(case_list), 1):
            if what == case_list[i]:
                print(i)
                return i
                break
            else:
                pass


if __name__ == '__main__':
    try:
        scene_updater()
    except rospy.ROSInterruptException:
        pass