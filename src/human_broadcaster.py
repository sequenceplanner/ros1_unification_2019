#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Kinect Humans tf Broadcaster
    # V.0.0.1.
#----------------------------------------------------------------------------------------

import tf
import tf2_ros as tf2
import time
import rospy
import roslib
from ros1_unification_2019.msg import KinectHumans

class kinect_human_broadcaster():

    def __init__(self):

        time.sleep(1) # Wait for the bridge to start
        rospy.init_node('kinect_human_broadcaster', anonymous=False)

        rospy.Subscriber("kinect_humans", KinectHumans, self.kinect_human_callback)

        self.br = tf.TransformBroadcaster()
        self.human_list = []
        self.joint_list = []
        self.distances = []
        self.x = 0

        self.human_id_list = [1, 2, 3, 4, 5, 6]

        # Filtered out the non-essential joints
        self.joint_name_list = ["Neck", "Head", "ShoulderLeft", "ShoulderRight", "SpineShoulder"]

        self.rate = rospy.Rate(25)
        self.main()


    def kinect_human_callback(self, humans):

        self.human_list = humans.human_list


    def main(self):

        while not rospy.is_shutdown():

            self.distances = []

            if self.human_list != []:
                self.human_list_sample = self.human_list

                for j in range(0, len(self.human_list_sample)):

                    self.human_id = self.human_list_sample[j].human_id
                    self.joint_list = self.human_list_sample[j].joint_list

                    for i in range(0, len(self.joint_list)):

                        if self.joint_list[i].joint_id in self.joint_name_list:

                            self.br.sendTransform((self.joint_list[i].x, self.joint_list[i].y, self.joint_list[i].z),
                                              (0, 0, 0, 1),
                                              rospy.Time.now(),
                                              str(self.joint_list[i].joint_id) + str(self.human_id),
                                              "kinect")

                            if self.joint_list[i].z > 1 and self.joint_list[i].z < 2:
                                self.distances.append("close")
                            elif self.joint_list[i].z > 2 and self.joint_list[i].z < 3:
                                self.distances.append("far")
                            else:
                                self.distances.append("unknown")

                        else:
                            pass

                    if "close" in self.distances:
                        print("human " + str(self.human_id) + " is close")
                    elif "close" not in self.distances and "far" in self.distances:
                        print("human " + str(self.human_id) + " is far")
                    else:
                        print("human " + str(self.human_id) + " is unknown")                   
                            
                else:
                    pass
                
            self.rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    try:
        kinect_human_broadcaster()
    except rospy.ROSInterruptException:
        pass