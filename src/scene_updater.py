#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Unification Scene Updater
    # V.0.3.0.
#----------------------------------------------------------------------------------------

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
        rospy.init_node('ur_pose_updater', anonymous=False)

        # Moveit Commander initializers:
        self.robot = mgc("manipulator")
        self.scene = psi()
      
        # Subscribers and Publishers:
        rospy.Subscriber("/unification_roscontrol/scene_updater_sp_to_uni", SceneUpdaterSPToUni, self.sp_callback)
        self.main_publisher = rospy.Publisher("unification_roscontrol/scene_updater_uni_to_sp", SceneUpdaterUniToSP, queue_size=10)

        # ROS package localizer:
        self.rospack = rospkg.RosPack()

        # Stl mesh destinations:
        self.lf_mesh = self.rospack.get_path('ros1_unification_2019') + '/cad_meshes/LF.stl'
        self.engine_mesh = self.rospack.get_path('ros1_unification_2019') + '/cad_meshes/engine_reduced.stl'
        self.agv_mesh = self.rospack.get_path('ros1_unification_2019') + '/cad_meshes/AGV.stl'
        self.ts_tool_mesh = self.rospack.get_path('ros1_unification_2019') + '/unstruct_ptcl_meshes/tstool.stl'

        # UR10 link idenifiers:
        self.ur10_links = ['base_link', 'shoulder_link', 'elbow_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'tool0', 'ee_link']

        # Message freshness definition in seconds:
        self.message_freshness = 3

        # Message type initializers:
        self.main_msg = SceneUpdaterUniToSP()
        self.obj_msg = PsiObjectPose()
        self.common_msg = Common()

        # Initialize timeout and stopwatch
        self.callback_timeout = time.time()
        self.start = time.time()

        # Message value initializers:
        self.fresh_msg = False
        self.t_plus = ''
        self.got_reset = False
        self.error = "none"
        self.object_action = ''
        self.object_name = ''
        self.euler_pose = []
        self.got_object_action = ''
        self.got_object_name = ''
        self.got_euler_pose = [] 

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
            self.common_msg.error = self.error
            
            # Construct the whole message:
            self.main_msg.state = self.common_msg
            self.main_msg.got_object_action = self.object_action
            self.main_msg.got_object_name = self.object_name
            self.main_msg.got_euler_pose = self.euler_pose
            #self.main_msg.attached_objects = self.attached_objects()
            self.main_msg.object_poses = self.object_poses()

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
        elif name not in self.get_objects() name in self.object_name_cases:
            self.error = "Object named: " + name + "not in the scene."
        elif name not in self.object_name_cases:
            self.error = "Object named: " + name + "not valid."
        else:
            pass
    
    
    def sp_callback(self, data):
        '''
        Evaluate and use the command message from Sequence Planner
        '''

        # Refreshing the message
        self.callback_timeout = time.time() + self.message_freshness

        # Assigning for remote reuse
        self.object_action = data.object_action 
        self.object_name = data.object_name
        self.euler_pose = data.euler_pose

        # Check for the 'reset' flag
        if self.object_name == "reset":
            self.prev_object_name = "reset"
        else:
            pass

        # Switcher lists of cases for implementing a switch-case like behavior:
        self.object_action_cases = ['add', 'remove', 'move', 'clear', 'attach', 'detach']
        self.object_name_cases = ['lf', 'ts_tool', 'agv', 'engine'] # Gradually add more...

        # Checking action command corectness:
        if self.object_action not in self.object_action_cases:
            self.error = "Action case: " + name + "not valid."
        else:
            pass

        # Checking object name corectness:
        if self.object_action not in self.object_action_cases:
            self.error = "Action case: " + name + "not valid."
        else:
            pass

        # Switcher lists of functions for implementing a switch-case like behavior:
        self.object_name_functions = [self.lf_mesh, self.ts_tool_mesh, self.agv_mesh, self.engine_mesh]
        self.object_action_functions = [self.add_object(self.object_name, self.object_name_functions[self.object_name_switcher], self.euler_pose),
                                        self.remove_object(self.object_name),
                                        self.move_object(self.object_name, self.euler_pose),
                                        self.clear_scene(),
                                        self.attach_object(self.object_name, self.robot.get_end_effector_link()),
                                        self.detach_object(self.object_name)]

        # Instantiate switchers
        self.object_name_switcher = self.switcher(self.object_name, self.object_name_cases)                               
        self.object_action_switcher = self.switcher(self.object_action, self.object_action_cases)

        # Evaluate messages only once and use the 'reset' flag if stuck
        if self.object_name != self.prev_object_name:
            self.object_action_functions[self.object_action_switcher]
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