#!/usr/bin/env python3

# http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20%28Python%29
# usage:   $ rosrun tf tf_echo world object_8

import rospy
import tf
import numpy as np  # Scientific computing library for Python


def eul2quat(roll, pitch, yaw):
    """
        https://automaticaddison.com/how-to-convert-euler-angles-to-quaternions-using-python/
        Convert an Euler angle to a quaternion.

        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
            :return qx, qy, qz, qw: The orientation in quaternion
            [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)
    - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)

    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)

    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)
    + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
    return [qx, qy, qz, qw]


if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # Hier Abstand camera_link zum world_frame eintragen
        # vorher messen in Meter
        # x, y, z    67cm hohes Stativ
        # quaternion?
        br.sendTransform((0.0, 0.0, 0.67),
                         (eul2quat(0.0, 0.78, 0.0)),
                         rospy.Time.now(),
                         "camera_link",
                         "world")
       
        rate.sleep()