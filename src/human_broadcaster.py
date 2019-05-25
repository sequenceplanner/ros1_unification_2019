#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Kinect Humans tf Broadcaster and distance discretization
    # V.1.0.0.
#----------------------------------------------------------------------------------------

import tf
import tf2_ros as tf2
import time
import rospy
import roslib
from ros1_unification_2019.msg import KinectHumans
from ros1_unification_2019.msg import OccupationZone
from ros1_unification_2019.msg import HumansPoseUniToSP

class kinect_human_broadcaster():

    def __init__(self):

        time.sleep(1) # Wait for the bridge to start
        rospy.init_node('kinect_human_broadcaster', anonymous=False)

        rospy.Subscriber("kinect_humans", KinectHumans, self.kinect_human_callback)
        self.main_pub = rospy.Publisher('unification_roscontrol/humans_pose_uni_to_sp', HumansPoseUniToSP, queue_size=10)

        self.br = tf.TransformBroadcaster()
        self.human_list = []
        self.joint_list = []
        self.distances = []
        self.x = 0

        self.occupation_zone = OccupationZone()
        self.humans_pose_uni_to_sp = HumansPoseUniToSP()

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
            self.humans_pose_uni_to_sp = []

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
                            elif self.joint_list[i].z > 3 and self.joint_list[i].z < 4:
                                self.distances.append("far")
                            else:
                                self.distances.append("unknown")

                        else:
                            pass

                    if "close" in self.distances:
                        self.occupation_zone.zone = "close"
                        self.occupation_zone.human_id = self.human_id
                        #print("human " + str(self.human_id) + " is close")
                    elif "close" not in self.distances and "far" in self.distances:
                        self.occupation_zone.zone = "far"
                        self.occupation_zone.human_id = self.human_id
                        #print("human " + str(self.human_id) + " is far")
                    else:
                        self.occupation_zone.zone = "outside"
                        self.occupation_zone.human_id = self.human_id
                        #print("human " + str(self.human_id) + " is unknown")

                    self.humans_pose_uni_to_sp.append(self.occupation_zone)
                
                self.main_pub.publish(self.humans_pose_uni_to_sp)
                            
            else:
                self.occupation_zone.zone = "no people in the frame"
                self.occupation_zone.human_id = 0
                self.humans_pose_uni_to_sp.append(self.occupation_zone)
                self.main_pub.publish(self.humans_pose_uni_to_sp)
                pass
            
            self.rate.sleep()

        rospy.spin()

if __name__ == '__main__':
    try:
        kinect_human_broadcaster()
    except rospy.ROSInterruptException:
        pass