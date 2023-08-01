#!/usr/bin/env python  
import roslib
# roslib.load_manifest('robot_control')
import rospy
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('robot_position')
    listener = tf.TransformListener()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print('(x, y) = (',round(trans[0],6), ' , ', round(trans[1],6), ')')
        rate.sleep()
