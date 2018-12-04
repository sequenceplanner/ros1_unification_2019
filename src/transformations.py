#!/usr/bin/env python

#----------------------------------------------------------------------------------------
# authors, description, version
#----------------------------------------------------------------------------------------
    # Endre Eres
    # Helper Class for Transformations
    # V.0.3.0.
#----------------------------------------------------------------------------------------

import rospy
import roslib
import numpy
import math
import tf

class transformations:

    def quat_to_rpy(self, x, y, z, qx, qy, qz, qw):
        "returns euler rpy vector from quaternions"
	
	quaternions = (qx, qy, qz, qw)
        euler = tf.transformations.euler_from_quaternion(quaternions)

        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        return [x, y, z, roll, pitch, yaw]

    def rpy_to_quat(self, x, y, z, roll, pitch, yaw):
	"returns quaternions from euler rpy vector"

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

	qx = quaternion[0]
        qy = quaternion[1]
        qz = quaternion[2]
        qw = quaternion[3]

        return [x, y, z, qx, qy, qz, qw]

    def rpy_to_rot(self, x, y, z, roll, pitch, yaw):
        "returns rotational vector from euler rpy"

        yawMatrix = numpy.matrix([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
        ])

        pitchMatrix = numpy.matrix([
        [math.cos(pitch), 0, -math.sin(pitch)],
        [0, 1, 0],
        [math.sin(pitch), 0, math.cos(pitch)]
        ])

        rollMatrix = numpy.matrix([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
        ])

        R = yawMatrix * pitchMatrix * rollMatrix

        theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1) / 2)
        multi = 1 / (2 * math.sin(theta))

        rx = multi * (R[2, 1] - R[1, 2]) * theta
        ry = multi * (R[0, 2] - R[2, 0]) * theta
        rz = multi * (R[1, 0] - R[0, 1]) * theta

        return [x, y, z, rx, ry, rz]

    
    def quat_to_rot(self, x, y, z, qx, qy, qz, qw):
        "returns rotational vector from quaternions"

        rpy = [0, 0, 0, 0, 0, 0]
        rpy = self.quat_to_rpy(x, y, z, qx, qy, qz, qw)

        roll = rpy[3]
        pitch = rpy[4]
        yaw = rpy[5]
        
        rot = self.rpy_to_rot(x, y, z, roll, pitch, yaw)

        return rot
