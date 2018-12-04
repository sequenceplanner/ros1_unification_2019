#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Unification Scene Updater
    # V.0.1.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import rospkg
import sys
import os
import csv
from moveit_commander import PlanningSceneInterface as psi
from moveit_commander import roscpp_initialize, roscpp_shutdown
from transformations import transformations 
from ros1_unification_2019.msg import SceneUpdaterSPToUni
from ros1_unification_2019.msg import SceneUpdaterUniToSP
from ros1_unification_2019.msg import PsiObjectPose

class ur_pose_updater(transformations):

    def __init__(self):

        roscpp_initialize(sys.argv)
        rospy.init_node('ur_pose_updater', anonymous=False)
        self.robot = mgc("manipulator")
        self.scene = psi()
      
        rospy.Subscriber("/unification_roscontrol/scene_updater_sp_to_uni", SceneUpdaterSPToUni, self.sp_callback)
        self.object_poses_publisher = rospy.Publisher("unification_roscontrol/scene_updater_uni_to_sp", SceneUpdaterUniToSP, queue_size=10)

        self.rospack = rospkg.RosPack()

        self.main_msg = SceneUpdaterUniToSP()
        self.obj_msg = PsiObjectPose()

        self.lf_mesh = self.rospack.get_path('ros1_unification_2019') + '/cad_meshes/LF.stl'
        self.engine_mesh = self.rospack.get_path('ros1_unification_2019') + '/cad_meshes/engine_reduced.stl'
        self.agv_mesh = self.rospack.get_path('ros1_unification_2019') + '/cad_meshes/AGV.stl'
        self.ts_tool_mesh = self.rospack.get_path('ros1_unification_2019') + '/unstruct_ptcl_meshes/tstool.stl'

        self.rate = rospy.Rate(10)

        self.object_action = ''
        self.object_name = ''
        self.euler_pose = []
        self.error = ''

        rospy.sleep(5)

        self.main()


    def main(self):

        while not rospy.is_shutdown():
            
            self.main_msg.got_object_action = self.object_action
            self.main_msg.got_object_name = self.object_name
            self.main_msg.got_euler_pose = self.euler_pose
            self.main_msg.attached_object = self.attached_object()
            self.main_msg.object_poses = self.object_poses()
            self.main_msg.error = self.error
            self.object_poses_publisher.publish(main_msg)
            self.rate.sleep()
        
        rospy.spin()


    def add_object(self, rpy, object_name, object_file, object_size):

            quat = self.rpy_to_quat(rpy[0], rpy[1], rpy[2], rpy[3], rpy[4], rpy[5])

            scene_pose.header.frame_id = "world"
            scene_pose.pose.position.x = quat[0]
            scene_pose.pose.position.y = quat[1]
            scene_pose.pose.position.z = quat[2]
            scene_pose.pose.orientation.x = quat[3]
            scene_pose.pose.orientation.y = quat[4]
            scene_pose.pose.orientation.z = quat[5]
            scene_pose.pose.orientation.w = quat[6]

            self.scene.add_mesh(object_name, scene_pose, object_file, object_size)


    def attached_objects(self):
        object_list = []
        object_list.append(self.psi.get_attached_objects())
        return pose_list

    
    def object_poses(self):
        object_list = []
        objects_in_scene = self.get_objects()
        for obj in objects_in_scene:
            self.obj_msg.object_name = obj
            self.obj_msg.object_pose = self.quat_to_rot(self.get_object_poses(obj))
            object_list.append(self.obj_msg)
        return object_list

    
    def add_object(self, file, name, pose):
        if name not in self.object_name_cases:
            self.psi.add_mesh(name, pose, file, (1, 1, 1))
            self.error == "none"
        else:
            self.error == "Object: " + name + " not in valid. Add to object cases.")
    
    
    
    def sp_callback(self, data):
        self.object_action= data.object_action 
        self.object_name = data.object_name
        self.euler_pose = data.euler_pose

        if self.object_name == "reset":
            self.prev_object_name = "reset"
        else:
            pass

        self.object_name_cases = ['lf', 'ts_tool', 'agv', 'engine']
        self.object_name_functions = [self.lf_mesh, self.ts_tool_mesh, self.agv_mesh, self.engine_mesh]
        # advanced object_name_cases = ['lf', 'lf_tool', 'of', 'of_tool', 'ts_tool', 'agv', 'engine', 'mir', ... ]

        self.object_name_switcher = self.switcher(self.object_name, object_name_cases)

        self.object_action_cases = ['add', 'move', 'remove', 'attach', 'detach']
        self.object_action_functions = [self.add_object(self.object_name, object_name_functions[object_name_switcher], self.euler_pose),
                                   self.move_object(self.object_name, self.euler_pose),
                                   self.remove_object(self.object_name),
                                   self.attach_object(self.object_name, self.robot.get_end_effector_link(), self.object_name),
                                   self.detach_object(self.object_name)]

        self.object_action_switch = self.switcher(self.object_action, object_action_cases)

        if self.object_name != self.prev_object_name:
            object_action_functions[object_action_switch]
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
        scene_updater()
    except rospy.ROSInterruptException:
        pass