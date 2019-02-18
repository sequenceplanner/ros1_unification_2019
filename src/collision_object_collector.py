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
from moveit_commander import PlanningSceneInterface as psi
from moveit_commander import roscpp_initialize, roscpp_shutdown


class collision_object_collector():
   
    def __init__(self):

        # General initialisers:
        roscpp_initialize(sys.argv)
        rospy.init_node('collision_object_collector', anonymous=False)

        # Moveit Commander initializers:
        self.scene = psi()
      
        # Subscribers and Publishers:
        #self.main_publisher = rospy.Publisher("unification_roscontrol/collision_objects", \
        #                      SceneUpdaterUniToSP, queue_size=10)

       
        # Message type initializers:
        # self.main_msg = SceneObjects()
    
        # Publisher rates:
        self.main_pub_rate = rospy.Rate(10)

        time.sleep(2)
        # self.attach_object("box1", "tool0")
        
        # Main loop method call:
        self.main()



    def main(self):
        '''
        This method spins until an interrupt exception is raised.
        The state message is generated here and published to the
        main publisher topic.

        '''

        while not rospy.is_shutdown():

            #print(self.scene_obje)
            print(self.scene.get_known_object_names())
            #rospy.sleep(1)
        
        rospy.spin()



    def attached_objects(self):
   
        object_list = []
        object_list.append(str(self.scene.get_attached_objects()))

        return object_list

    def scene_objects(self):

        object_list = []
        object_list.append(list(self.scene.get_objects()))

        return object_list

   
if __name__ == '__main__':
    try:
        collision_object_collector()
    except rospy.ROSInterruptException:
        pass
