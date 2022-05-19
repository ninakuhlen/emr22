#!/usr/bin/env python3

# http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20%28Python%29


import roslib

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 2.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "world",
                         "camera_link")
        rate.sleep()