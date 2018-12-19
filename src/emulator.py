#!/usr/bin/env python

import time
import rospy
import roslib
from ros1_unification_2019.msg import URPoseSPToUni

class emulator():
    
    def __init__(self):

        rospy.init_node('emulator', anonymous=False)
        self.main_publisher = rospy.Publisher("unification_roscontrol/ur_pose_unidriver_sp_to_uni", URPoseSPToUni, queue_size=10)
       
        self.main_msg = URPoseSPToUni()

        self.action = ''
        self.robot_type = ''
        self.robot_name = ''
        self.pose_type = ''
        self.pose_name = ''
        self.speed_scaling = 0.0
        self.acc_scaling = 0.0
        self.goal_tolerance = 0.0

    def main(self):

        while not rospy.is_shutdown():

            self.main_msg.action = 'PLANNED'
            self.main_msg.robot_type = 'UR10'
            self.main_msg.robot_name = 'TARS'
            self.main_msg.pose_type = 'TCP'
            self.main_msg.pose_name = 'PRE_ATTACH_OF'
            self.main_msg.speed_scaling = 0.1
            self.main_msg.acc_scaling = 0.1
            self.main_msg.goal_tolerance = 0.01
            
            
            self.main_publisher.publish(self.main_msg)
            time.sleep(5)

            self.main_msg.action = 'PLANNED'
            self.main_msg.pose_type = 'TCP'
            self.main_msg.pose_name = 'ATTACH_OF'
            self.main_pub_rate.sleep()
        
        rospy.spin()


    

if __name__ == '__main__':
    try:
        emulator()
    except rospy.ROSInterruptException:
        pass