#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Kinect Human tf Broadcaster
    # V.0.0.1.
#----------------------------------------------------------------------------------------

import tf
import tf2_ros as tf2
import time
import rospy
import roslib
from ros1_unification_2019.msg import KinectHumans
from ros1_unification_2019.msg import JointPosition
from ros1_unification_2019.msg import HumanJoints


class kinect_human_broadcaster():

    def __init__(self):

        time.sleep(1) # Wait for the bridge to start
        rospy.init_node('kinect_human_broadcaster', anonymous=False)

        rospy.Subscriber("kinect_humans", KinectHumans, self.kinect_human_callback)

        self.br = tf.TransformBroadcaster()

        self.x = 0

        self.human_id_list = [1, 2, 3, 4, 5, 6]

        self.joint_name_list = ["SpineBase",
                          "SpineMid",
			              "Neck",
			              "Head",
			              "ShoulderLeft",
			              "ElbowLeft",
			              "WristLeft",
			              "HandLeft",
			              "ShoulderRight",
			              "ElbowRight",
			              "WristRight",
			              "HandRight",
			              "HipLeft",
			              "KneeLeft",
			              "AnkleLeft",
			              "FootLeft",
			              "HipRight",
			              "KneeRight",
			              "AnkleRight",
			              "FootRight",
			              "SpineShoulder",
			              "HandTipLeft",
			              "ThumbLeft",
			              "HandTipRight",
                          "ThumbRight" ]

        self.rate = rospy.Rate(10)
        self.main()


    def kinect_human_callback(self, humans):

        if humans.human_list != []:

            for j in range(0, len(humans.human_list)):

                self.human_id = humans.human_list[j].human_id
                self.joint_list = humans.human_list[j].joint_list

                for i in range(0, len(self.joint_list)):

                    self.br.sendTransform((self.joint_list[i].x, self.joint_list[i].y, self.joint_list[i].z),
                                           (0, 0, 0, 1),
                                           rospy.Time.now(),
                                           str(self.joint_list[i].joint_id) + str(self.human_id),
                                           "kinect")

        else:
            pass
           

    def main(self):
        while not rospy.is_shutdown():
            pass
                
        self.rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    try:
        kinect_human_broadcaster()
    except rospy.ROSInterruptException:
        pass