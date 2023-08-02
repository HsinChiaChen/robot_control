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

def design_path(t):
    freq = 2*math.pi/30
    xRef = 1.1 + 0.7*math.sin(freq*t); yRef = 0.9 + 0.7*math.sin(2*freq*t)
    dxRef = freq*0.7*math.cos(freq*t); dyRef = 2*freq*0.7*math.cos(2*freq*t)
    ddxRef =-freq^2*0.7*math.sin(freq*t); ddyRef =-4*freq^2*0.7*math.sin(2*freq*t)

    qRef = (xRef, yRef, math.atan(dyRef, dxRef)) # Reference trajectory
    vRef = math.sqrt(dxRef^2+dyRef^2)
    wRef = (dxRef*ddyRef-dyRef*ddxRef)/(dxRef^2+dyRef^2)
    uRef = (vRef,wRef) # Reference inputs

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
        print('(x, y, z) = (',round(trans[0],6), ' , ', round(trans[1],6), ' , ', round(trans[2],6) ,')')

        end_time = time.time()
        runing_time = end_time - start_time

        # (x,y) = design_path(runing_time)
        # x_error = x - round(trans[0],6)
        # y_error = y - round(trans[1],6)

        print('time = ', runing_time)


        scan = robotsensor.get_velodyne()
        [roll, pitch, yaw] = robotsensor.get_odom()
        # [angle, dis] = robotsensor.get_min_distance_angle()

        # print(angle, dis)
        # print(pitch)
        # print(type(scan))  <class 'tuple'>
        # print(type(scan.data))  <class 'bytes'>
        # int_scan_data = int(float(scan.data))
        # print(int_scan_data)
        # scan.data.tolist()

        img = robotsensor.get_image()
        cv_image = bridge.imgmsg_to_cv2(img, "passthrough")
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(3)

        (roll, pitch, yaw) = robotsensor.get_odom()

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

        