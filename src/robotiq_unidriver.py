#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres, Patrik Bergagard
    # Unification Driver for the Robotiq Gripper
    # V.0.3.0.
#----------------------------------------------------------------------------------------

import time
import json
import rospy
import roslib
import threading
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
from ros1_unification_2019.msg import RobotiqSPToUni
from ros1_unification_2019.msg import RobotiqUniToSP
from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg
from robotiq_c_model_control.msg import _CModel_robot_input as inputMsg

class robotiq_unidriver():

    def __init__(self):

        rospy.init_node('robotiq_unidriver', anonymous=False)

        rospy.Subscriber("/CModelRobotInput", inputMsg.CModel_robot_input, self.gripperCallback)
        rospy.Subscriber("/unification_roscontrol/robotiq_sp_to_uni", RobotiqSPToUni, self.spCallback)
        self.gripperPub = rospy.Publisher('/CModelRobotOutput', outputMsg.CModel_robot_output, queue_size=10)
        self.statePub = rospy.Publisher('/unification_roscontrol/robotiq_uni_to_sp', RobotiqUniToSP, queue_size=10)

        self.cmd = ''
        self.got_cmd = ''
        self.ref_pos = 0
        self.got_ref_pos = 0
        self.prev_ref_pos = 0
        self.act_pos = 0

        rospy.sleep(2)
        self.resetGripper()
        rospy.sleep(1)
        self.activateGripper()
        rospy.sleep(2)
        self.state_rate = rospy.Rate(10)       

        self.main()
    
    
    def publish(self):
        self.gripperPub.publish(self.command)
        rospy.sleep(0.2)


    def resetGripper(self):
        self.command = outputMsg.CModel_robot_output()
        self.command.rACT = 0
        self.publish()


    def activateGripper(self):
        self.command = outputMsg.CModel_robot_output()
        self.command.rACT = 1
        self.command.rGTO = 1
        self.command.rSP  = 255
        self.command.rFR  = 150
        self.publish()


    def positionGripper(self, pos):
        self.command.rPR = int(pos)
        if self.command.rPR > 255:
            self.command.rPR = 255
        if self.command.rPR < 0:
            self.command.rPR = 0
        self.publish()


    def openGripper(self):
        self.positionGripper(0)


    def closeGripper(self):
        self.positionGripper(255)


    def setGripperSpeed(self, speed):
        self.command.rSP = int(speed)
        if self.command.rSP > 255:
            self.command.rSP = 255
        if self.command.rSP < 0:
            self.command.rSP = 0
        self.publish()


    def setGripperForce(self, force):
        self.command.rFR = int(force)
        if self.command.rFR > 255:
            self.command.rFR = 255
        if self.command.rFR < 0:
            self.command.rFR = 0
        self.publish()
    

    
    def main(self):

        self.msg = RobotiqUniToSP()

        while not rospy.is_shutdown():
	    self.msg.got_cmd = self.cmd
            self.msg.got_ref_pos = self.ref_pos
            self.msg.act_pos = self.act_pos
            self.statePub.publish(self.msg)
            self.state_rate.sleep()
    
        rospy.spin()

   
    def spCallback(self, data):
        self.cmd = data.cmd
        self.ref_pos = data.ref_pos

        if self.ref_pos == 0:
            self.prev_ref_pos = 0
        else:
            pass

        if self.ref_pos != self.prev_ref_pos:
            self.prev_ref_pos = self.ref_pos
            self.positionGripper(self.ref_pos)
        else:
            pass

    
    def gripperCallback(self, obj):
        self.act_pos = obj.gPO


if __name__ == '__main__':
    try:
        robotiq_unidriver()
    except rospy.ROSInterruptException:
        pass
