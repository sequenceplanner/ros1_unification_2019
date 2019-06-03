#!/usr/bin/env python

import rospy
import time
import tf
from numpy import matrix
from numpy import linalg
import numpy
import math
import os
import socket
import atexit
import signal

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_msgs.msg import TFMessage
from ros1_unification_2019.msg import RecuSPToUni
from ros1_unification_2019.msg import URPoseSPToUni
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO

HOST = "192.168.10.16"
PORT = 29999

#--------------------------------------------------------------------  
# Test 1 class:
#--------------------------------------------------------------------  
class Test1():

    #----------------------------------------------------------------
    # init function
    #----------------------------------------------------------------
    def __init__(self):

        
        #self.forcepublisher = rospy.Publisher("/FORCEFORCE", String, queue_size=1)

        self.init = rospy.Rate(30)
        self.io_service = rospy.ServiceProxy('/ur_driver/set_io', SetIO, self.cb5)


        self.lock_rsp = False
        self.unlock_rsp = False
        self.open_gripper = False
        self.close_gripper = False
        self.executing = False
        self.grab = ""
        self.ref_pos = ""


        # Wait for topics to start publishing
        rospy.Subscriber("/joint_states", JointState, self.cb1)
        while(True):
            try:
                self.joints
                rospy.loginfo("/joint_states found")
                break
            except (AttributeError):
                self.init.sleep()
                continue

        rospy.Subscriber("/wrench",WrenchStamped, self.cb2)
        while(True):
            try:
                self.force
                self.wrench
                rospy.loginfo("/wrench found")
                break
            except (AttributeError):
                self.init.sleep()
                continue

        #rospy.Subscriber("/ethdaq_data",WrenchStamped, self.cb3)
        #while(True):
        #    try:
        #        self.T
        #        self.F
        #        rospy.loginfo("/ethdaq_data found")
        #        break
        #    except (AttributeError):
        #        self.init.sleep()
        #        continue
            
        rospy.Subscriber("/ur_driver/io_states", IOStates, self.cb4)
        while(True):
            try:
                self.IO
                rospy.loginfo("/io_states found")
                break
            except (AttributeError):
                self.init.sleep()
                continue

        #rospy.Subscriber("/FORCEFORCE", String, self.forceforceclbk)
        rospy.Subscriber("/unification_roscontrol/gestures_uni_to_sp", String, self.gesture_callback)
        self.gesture_ack = rospy.Publisher("/unification_roscontrol/gestures_sp_to_uni", String, queue_size=10)

        # Publish to robot
        # self.urScriptPub = rospy.Publisher("/ur_driver/URScript", String, queue_size=1)
        self.RecuSPToUniPublisher = rospy.Publisher("/unification_roscontrol/recu_sp_to_uni", RecuSPToUni, queue_size=1)
        self.URPoseSPToUniPublisher = rospy.Publisher("/unification_roscontrol/ur_pose_unidriver_sp_to_uni", URPoseSPToUni, queue_size=1)

        rospy.sleep(1.0)
        
        # Go into spin, with rateOption!
        self.rate = rospy.Rate(10)          # 10hz

        self.set_IO_states(1, 0, 0)

        self.lifting = False
        self.freedrive = True
        
        self.loadNPlay('/programs/CollabForce_Freedrive.urp')

        self.set_IO_states(1, 7, 1)

        atexit.register(self.exit_handler)
	
    #----------------------------------------------------------------
    # subscribers' callback functions:
    #----------------------------------------------------------------
    def cb1(self, data):
        # Get update from the manipulator:
        self.joints = data.position

    def cb2(self,data):
	    # Get update from force sensors on manipulator
        self.force = data.wrench.force
        self.wrench = data.wrench.torque
    
    def cb3(self,data):
        #Get update from optoforce force sensor
        self.T = data.wrench.torque
        self.F = data.wrench.force
        #print self.F.z

    def cb4(self,data):
        self.IO = data      

    def cb5(self,data):
        self.IO = data
        print 'wrote IO cb5'

    # def forceforceclbk(self, data):
        # if data.data == "grab":
            # self.grab = "grab"
        # elif data.data == "release":
            # self.grab = "release"
        # else:
            # pass

    def gesture_callback(self, data):
        if data.data == "COMMAND1":
            self.grab = "grab"
            self.gesture_ack.publish("CLEAR")
        elif data.data == "COMMAND3":
            self.grab = "release"
            self.gesture_ack.publish("CLEAR")
        else:
            pass


 
    #----------------------------------------------------------------
    # spin function (To keep on the node running)
    #----------------------------------------------------------------
    def spin(self):

        self.recu_sp_to_uni = RecuSPToUni()
        self.ur_pose_sp_to_uni = URPoseSPToUni()

        while (not rospy.is_shutdown()):

            

            #--------------------------------------------------------
            # Change the mode from handling the object to releasing the object:
            #--------------------------------------------------------
            if self.lifting and not self.freedrive and self.grab == "release":  # and self.F.z < -100:
                #Release frame
                self.tcp('stop')
                self.loadNPlay('/programs/CollabForce_Release.urp')

                self.set_IO_states(1, 7, 1)

                while not self.IO.digital_out_states[7].state:
                    if self.IO.digital_out_states[0].state == 1:
                        RecuSPToUni.lock_rsp = False
                        RecuSPToUni.unlock_rsp = False
                        RecuSPToUni.open_gripper = True
                        RecuSPToUni.close_gripper = False
                        self.RecuSPToUniPublisher.publish(self.recu_sp_to_uni)
                    else:
                        RecuSPToUni.lock_rsp = False
                        RecuSPToUni.unlock_rsp = False
                        RecuSPToUni.open_gripper = False
                        RecuSPToUni.close_gripper = True
                        self.RecuSPToUniPublisher.publish(self.recu_sp_to_uni)
                    self.init.sleep()
                while self.IO.digital_out_states[7].state:
                    if self.IO.digital_out_states[0].state == 1:
                        RecuSPToUni.lock_rsp = False
                        RecuSPToUni.unlock_rsp = False
                        RecuSPToUni.open_gripper = True
                        RecuSPToUni.close_gripper = False
                        self.RecuSPToUniPublisher.publish(self.recu_sp_to_uni)
                    else:
                        RecuSPToUni.lock_rsp = False
                        RecuSPToUni.unlock_rsp = False
                        RecuSPToUni.open_gripper = False
                        RecuSPToUni.close_gripper = True
                        self.RecuSPToUniPublisher.publish(self.recu_sp_to_uni)
                    self.init.sleep()
                
                #Go into freedrive
                self.tcp('stop')
                URPoseSPToUni.should_plan = False
                URPoseSPToUni.ref_pos = "AfterLFOperationJOINT"
                self.URPoseSPToUniPublisher.publish(self.ur_pose_sp_to_uni)
                rospy.sleep(10)
                os.system('rosnode kill /pickNPlace')
                time.sleep(1)
                os.kill(os.getppid(), signal.SIGHUP)
                self.loadNPlay('/programs/CollabForce_Freedrive.urp')
                self.freedrive = True
                self.lifting = False
                print 'Going into freedrive mode!\n------------------------------------------------'
                time.sleep(3)
            
            #--------------------------------------------------------
            # Change the mode from freedrive to the handling the object:
            #--------------------------------------------------------
            elif not self.lifting and self.freedrive and self.grab == "grab": #and self.F.z < -80:
                #Grip frame		
                self.tcp('stop')
                self.loadNPlay('/programs/CollabForce_Grip.urp')

                self.set_IO_states(1, 7, 1)

                while not self.IO.digital_out_states[7].state:
                    if self.IO.digital_out_states[0].state == 1:
                        RecuSPToUni.lock_rsp = False
                        RecuSPToUni.unlock_rsp = False
                        RecuSPToUni.open_gripper = True
                        RecuSPToUni.close_gripper = False
                        self.RecuSPToUniPublisher.publish(self.recu_sp_to_uni)
                    else:
                        RecuSPToUni.lock_rsp = False
                        RecuSPToUni.unlock_rsp = False
                        RecuSPToUni.open_gripper = False
                        RecuSPToUni.close_gripper = True
                        self.RecuSPToUniPublisher.publish(self.recu_sp_to_uni)
                    self.init.sleep()
                while self.IO.digital_out_states[7].state:
                    if self.IO.digital_out_states[0].state == 1:
                        RecuSPToUni.lock_rsp = False
                        RecuSPToUni.unlock_rsp = False
                        RecuSPToUni.open_gripper = True
                        RecuSPToUni.close_gripper = False
                        self.RecuSPToUniPublisher.publish(self.recu_sp_to_uni)
                    else:
                        RecuSPToUni.lock_rsp = False
                        RecuSPToUni.unlock_rsp = False
                        RecuSPToUni.open_gripper = False
                        RecuSPToUni.close_gripper = True
                        self.RecuSPToUniPublisher.publish(self.recu_sp_to_uni)
                    self.init.sleep()
                    
                #Go into lifting mode
                self.tcp('stop')
                self.loadNPlay('/programs/CollabForce_Follow_ZUnlockHalfForce.urp')
                self.freedrive = False
                self.lifting = True
                print 'Going into lifting mode!\n------------------------------------------------'
                time.sleep(3)

            
            # Go into spin, with rateOption!
            self.rate.sleep()        

    #----------------------------------------------------------------
    # using the socket programming to check the status, load the program, ...:
    #----------------------------------------------------------------
    def tcp(self, cmd):
        # https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/dashboard-server-port-29999-15690/
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        self.init.sleep()
        s.recv(1024)
        s.send (cmd + "\n")
        self.init.sleep()
        self.msg = s.recv(1024)
        print self.msg
        s.close()

    #----------------------------------------------------------------
    # load the program on the urPanel:
    #----------------------------------------------------------------
    def loadNPlay(self,cmd):
        print '======= Begin Loading:'
        self.tcp('load ' + cmd)
        self.init.sleep()
        self.loaded = 0
        self.played = 0

        # check if the program is loaded and played!
        while self.loaded != 1 and self.played != 1:
            self.tcp('get loaded program')
            if cmd in self.msg:
                self.loaded = 1
                self.tcp('programState')
                if 'PLAYING' in self.msg:
                    self.played = 1
                else:   
                    # print 'message:' + self.msg
                    self.tcp('play')
                    print '! ->  Trying to play again!\n'
            else:
                self.tcp('load ' + cmd)
                self.init.sleep()
                print '! ->  Trying to load again!\n'

        print '======= End Loading'
        return True

    def exit_handler(self):
        print '\n'
        self.tcp('stop')
        print 'Program terminated by user!'

    #----------------------------------------------------------------
    # set the IO on the robot:
    #----------------------------------------------------------------
    def set_IO_states(self,fun, pin, state):
        # resp = self.io_service.call(fun,pin,state)
        resp = []
        if fun != 1:
            resp = self.io_service.call(fun,pin,state)
        else:
            # To be sure that the DO is turning on or off (now it is just for digital outputs (pin 0 to 7)):
            if state == 1:
                while not self.IO.digital_out_states[pin].state:
                    resp = self.io_service.call(fun,pin,state)
                    self.init.sleep()
            else:
                while self.IO.digital_out_states[pin].state:
                    resp = self.io_service.call(fun,pin,state)
                    self.init.sleep()
        return resp
                
        
 

#--------------------------------------------------------------------  
# the main function to run the program
#--------------------------------------------------------------------  
if __name__ == '__main__':
    try:
        # Init the node:
        rospy.init_node('pickNPlace')
        
        # Initializing and continue running the class Test1:
        a = Test1();
        a.spin()

    except rospy.ROSInterruptException:
        pass

