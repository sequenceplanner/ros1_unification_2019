#!/usr/bin/env python

#----------------------------------------------------------------------------------------------------------------------#
# authors, description, version
#----------------------------------------------------------------------------------------------------------------------#
    # Endre Eres
    # Unification Scene Updater
    # V.1.0.0.
#----------------------------------------------------------------------------------------------------------------------#

import rospy
import roslib
import rospkg
import time
import sys
import os
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander as mgc
from moveit_commander import PlanningSceneInterface as psi
from moveit_commander import roscpp_initialize, roscpp_shutdown
from transformations import transformations 
from ros1_unification_2019.msg import Common
from ros1_unification_2019.msg import SceneUpdaterSPToUni
from ros1_unification_2019.msg import SceneUpdaterSPToUniRicochet as Ricochet
from ros1_unification_2019.msg import SceneUpdaterUniToSP


class ur_scene_updater(transformations):
    '''
    Updating the Scene by manipulating collision objects
    using the moveit_commanders PlanningSceneInterface class.
    The inherited transformations class provides methods for
    coordinate tranformations.
    '''
    def __init__(self):

        # General initialisers:
        roscpp_initialize(sys.argv)
        rospy.init_node('ur_scene_updater', anonymous=False)

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
        self.lf_mesh = self.rospack.get_path('ros1_unification_2019') + '/description/cad_meshes/LF.stl'
        self.engine_mesh = self.rospack.get_path('ros1_unification_2019') + '/description/cad_meshes/engine_reduced.stl'
        self.agv_mesh = self.rospack.get_path('ros1_unification_2019') + '/description/cad_meshes/AGV.stl'
        self.ts_tool_mesh = self.rospack.get_path('ros1_unification_2019') + '/description/unstruct_ptcl_meshes/tstool.stl'
        self.of_tool_mesh = self.rospack.get_path('ros1_unification_2019') + '/description/cad_meshes/OFTool.stl'

        # UR10 link idenifiers:
        self.ur10_links = ['base_link', 'shoulder_link', 'elbow_link', 'wrist_1_link',
                            'wrist_2_link', 'wrist_3_link', 'tool0', 'ee_link']

        # Switcher lists of cases for implementing a switch-case like behavior:
        self.robot_type_cases = ['UR10', 'IIWA7']
        self.ur10_robot_name_cases = ['TARS', 'KIPP', 'CASE']
        self.iiwa7_robot_name_cases = ['PLEX']
        self.all_robot_names = self.ur10_robot_name_cases + self.iiwa7_robot_name_cases
        self.object_action_cases = ['ADD', 'REMOVE', 'CLEAR', 'ATTACH', 'DETACH']
        self.object_name_cases = ['LF', 'TSTOOL', 'AGV', 'ENGINE', 'OFTOOL'] # Gradually add more...
        self.object_file_cases = [self.lf_mesh, self.ts_tool_mesh, self.agv_mesh, self.engine_mesh, self.of_tool_mesh]

        # Message freshness definition in seconds:
        self.message_freshness = 3

        # Message type initializers:
        self.pose = PoseStamped()
        self.main_msg = SceneUpdaterUniToSP()
        self.common_msg = Common()
        self.ricochet_msg = Ricochet()

        # Initialize timeout and stopwatch
        self.callback_timeout = time.time()
        self.start = time.time()

        # Common message value initializers:
        self.robot_name = ''
        self.fresh_msg = False
        self.t_plus = ''
        self.got_reset = False
        self.error_list = []

        # SceneUpdaterSPToUni message value initializers:
        self.object_action = ''
        self.object_name = ''

        # TODO: Add error list and error handler

        # Ricochet message value initializers:
        self.got_object_action = ''
        self.got_object_name = ''

        # Tick inhibitor:
        self.tick_inhibited = False
        self.prev_object_action = ''
        self.prev_object_name = ''

        # Publisher rates:
        self.main_pub_rate = rospy.Rate(10)

        # Some time to assure initialization:
        rospy.sleep(3)

        #self.box_pose = ["world", 0.15115, -0.661048, 2.4475, 0.484524, 0.515012, -0.484524, 0.515012]
        self.engine_pose = [0, 0.5, 0.8, 1.5707, 3.1415, 0]
        self.box_of_pose = ["world", 0.15115, -0.661048, 2.4475, 0.484524, 0.515012, -0.484524, 0.515012]
        self.lf_pose = [0.15115, -0.661048, 2.4475, 0.484524, 0.515012, -0.484524, 0.515012]

        # Adding collision objects (will be done in a method after getting the pose)
        self.scene.add_box("OFTOOL", self.list_to_pose_stamped(self.box_of_pose), size = (0.1, 0.1, 0.25))
        self.add_object(self.engine_mesh, 'ENGINE', self.engine_pose)
        time.sleep(5)
        #self.add_object(self.of_tool_mesh, 'OFTOOL', self.of_pose)
        #self.add_object(self.lf_mesh, 'LF', self.lf_pose)

        time.sleep(2)
        # self.attach_object("box1", "tool0")
        
        # Main loop method call:
        self.main()


    def list_to_pose_stamped(self, list):
        '''
        Transform a list into a PoseStamped() type
        '''

        self.pose.header.frame_id = list[0]
        self.pose.pose.position.x = list[1]
        self.pose.pose.position.y = list[2]
        self.pose.pose.position.z = list[3]
        self.pose.pose.orientation.x = list[4]
        self.pose.pose.orientation.y = list[5]
        self.pose.pose.orientation.z = list[6]
        self.pose.pose.orientation.w = list[7]
        return self.pose


    def inhibit_tick(self):
        '''
        Check if two successive messages are the same and disallow consumption if True.
        This method assigns values to the previous message variables so that they can be compared later.
        '''

        self.prev_object_action = self.object_action
        self.prev_object_name = self.object_name



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
            self.common_msg.robot_name = '' # add read from param later
            self.common_msg.fresh_msg = self.fresh_msg
            self.common_msg.t_plus = self.timer_elapsed()
            self.common_msg.got_reset = self.got_reset
            self.common_msg.error_list = [] #self.generate_error_message()
            
            # Construct the ricochet message part:
            self.ricochet_msg.got_object_action = self.object_action
            self.ricochet_msg.got_object_name = self.object_name

            # Construct the whole message:
            self.main_msg.state = self.common_msg
            self.main_msg.ricochet = self.ricochet_msg
            
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

    
    def add_object(self, obj_file, name, pose):
        '''
        Add a collision object to the planning scene from
        the list of object name cases
        '''

        if name in self.object_name_cases:

            quat = self.rpy_to_quat(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5])

            self.pose.header.frame_id = "world"
            self.pose.pose.position.x = quat[0]
            self.pose.pose.position.y = quat[1]
            self.pose.pose.position.z = quat[2]
            self.pose.pose.orientation.x = quat[3]
            self.pose.pose.orientation.y = quat[4]
            self.pose.pose.orientation.z = quat[5]
            self.pose.pose.orientation.w = quat[6]

            self.scene.add_mesh(name, self.pose, obj_file, (0.01, 0.01, 0.01))
            self.error = "none"

        elif name != '' and name not in self.object_name_cases:
            self.error = "Object named: " + name + " not valid."
        else:
            pass


    def remove_object(self, name):
        '''
        Remove an existing collision object from the planning scene
        '''
        self.scene.remove_world_object(name)

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
        '''

    
    def move_object(self, obj_file, name, pose):
        '''
        Move and existing object in the scene to a noew pose
        '''

        self.scene.remove_object(obj_file, name)
        self.add_object(obj_file, name, pose)


    def clear_scene(self):
        '''
        Remove all collision objects from the scene
        '''

        self.scene.remove_world_object()

    
    def attach_object(self, name, link):
        '''
        Attach a collision object to a specified link
        '''

        #if name in list(self.get_objects()) and link in self.links:
        self.robot.attach_object(name, link)
            # self.error = "none"
        # elif name in list(self.get_objects()) and not link in self.ur10_links:
            # self.error = "Link named: " + link + " not valid."
        # elif name not in list(self.get_objects()):
            # self.error = "Object named: " + name + "not valid."
        # else:
            # pass

    
    def detach_object(self, name):
        '''
        Detach a specific collision object from the whole move group
        '''
        self.robot.detach_object(name)
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
        '''

    def action_method_switch(self, action_case):
        '''
        Call an action method based on the case number.
        '''

        if action_case == 0:
            return self.add_object(self.object_name, 
                                   self.object_file_cases[self.object_name_case])

        elif action_case == 1:
            return self.remove_object(self.object_name)

        elif action_case == 2:
            return self.clear_scene()

        elif action_case == 3:
            return self.attach_object(self.object_name, 
                                      "tool0")

        elif action_case == 4:
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

        # Tick inhibitor msg check:
        if self.object_action == self.prev_object_action and \
           self.prev_object_name == self.object_name:
            self.tick_inhibited = True
        else:
            self.tick_inhibited = False

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
            self.action_method_switch_error = "action: " + self.object_action + " not valid"

        # Evaluate messages only once and use the 'reset' flag if stuck
        if self.tick_inhibited == False:
            if self.action_method_switch_error == "":
                self.inhibit_tick()
                self.action_method_switch(self.object_action_case)
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
                print(i)
                return i
                break
            else:
                pass


if __name__ == '__main__':
    try:
        ur_scene_updater()
    except rospy.ROSInterruptException:
        pass
