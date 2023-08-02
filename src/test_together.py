#!/usr/bin/env python
import roslib
# roslib.load_manifest('robot_control')
import rospy
import tf
from geometry_msgs.msg import Twist
import time
import math
from robot_move_class import RobotMove
from robot_sensor_class import RobotSensor

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


robotmove = RobotMove()
robotsensor = RobotSensor()
cmd=Twist() 
bridge = CvBridge()

if __name__ == '__main__':
    rospy.init_node('robot_position')
    listener = tf.TransformListener()
    rate = rospy.Rate(20)
    start_time = time.time()
    vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print('(x, y) = (',round(trans[0],6), ' , ', round(trans[1],6), ')')

        scan = robotsensor.get_velodyne()
        [roll, pitch, yaw] = robotsensor.get_odom()
        # [angle, dis] = robotsensor.get_min_distance_angle()

        # print(angle, dis)
        # print(pitch)
        # print(type(scan))  <class 'tuple'>
        # print(type(scan.data))  <class 'bytes'>
        # print(scan.data.hex())
        # scan.data.tolist()

        img = robotsensor.get_image()
        cv_image = bridge.imgmsg_to_cv2(img, "passthrough")
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(3)

        #########---------------------- plan OK ----------------------#########
        # cmd.linear.x = 0.5
        # cmd.linear.y = 0
        # cmd.linear.z = 0
        # cmd.angular.x = 0
        # cmd.angular.y = 0
        # cmd.angular.z = 0
        # # Publish the velocity
        # vel_publisher.publish(cmd)
        # rate.sleep()

        #########---------------------- plan OK in class----------------------#########  
        cmd = robotmove.move_robot(0.5,0.5)
        vel_publisher.publish(cmd)
        rate.sleep()

        end_time = time.time()
        runing_time = end_time - start_time
        print('time = ', runing_time)



