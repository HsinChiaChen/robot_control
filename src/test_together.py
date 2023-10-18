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
import matplotlib.pyplot as plt

import numpy as np

robotmove = RobotMove()
robotsensor = RobotSensor()
cmd = Twist() 
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

def max_filtering(N, I_temp):
    wall = np.full((I_temp.shape[0]+(N//2)*2, I_temp.shape[1]+(N//2)*2), -1)
    wall[(N//2):wall.shape[0]-(N//2), (N//2):wall.shape[1]-(N//2)] = I_temp.copy()
    temp = np.full((I_temp.shape[0]+(N//2)*2, I_temp.shape[1]+(N//2)*2), -1)
    for y in range(0,wall.shape[0]):
        for x in range(0,wall.shape[1]):
            if wall[y,x]!=-1:
                window = wall[y-(N//2):y+(N//2)+1,x-(N//2):x+(N//2)+1]
                num = np.amax(window)
                temp[y,x] = num
    A = temp[(N//2):wall.shape[0]-(N//2), (N//2):wall.shape[1]-(N//2)].copy()
    return A


def min_filtering(N, A):
    wall_min = np.full((A.shape[0]+(N//2)*2, A.shape[1]+(N//2)*2), 300)
    wall_min[(N//2):wall_min.shape[0]-(N//2), (N//2):wall_min.shape[1]-(N//2)] = A.copy()
    temp_min = np.full((A.shape[0]+(N//2)*2, A.shape[1]+(N//2)*2), 300)
    for y in range(0,wall_min.shape[0]):
        for x in range(0,wall_min.shape[1]):
            if wall_min[y,x]!=300:
                window_min = wall_min[y-(N//2):y+(N//2)+1,x-(N//2):x+(N//2)+1]
                num_min = np.amin(window_min)
                temp_min[y,x] = num_min
    B = temp_min[(N//2):wall_min.shape[0]-(N//2), (N//2):wall_min.shape[1]-(N//2)].copy()
    return B


#B is the filtered image and I is the original image
def background_subtraction(I, B):
    O = I - B
    norm_img = cv2.normalize(O, None, 0,255, norm_type=cv2.NORM_MINMAX)
    return norm_img

def min_max_filtering(M, N, I):
    if M == 0:
        #max_filtering
        A = max_filtering(N, I)
        #min_filtering
        B = min_filtering(N, A)
        #subtraction
        normalised_img = background_subtraction(I, B)
    elif M == 1:
        #min_filtering
        A = min_filtering(N, I)
        #max_filtering
        B = max_filtering(N, A)
        #subtraction
        normalised_img = background_subtraction(I, B)
    return normalised_img

def goal_point(img):
    x = 0
    y = 0
    z = 0.35

    cv_image = bridge.imgmsg_to_cv2(img, "passthrough")
    kernel = np.ones((5, 5), np.uint8)
    cv_image = cv2.resize(cv_image, (0, 0), fx=1, fy=1)
    # print(cv_image)
    
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # O_P = min_max_filtering(M=1, N=70, I=gray)
    # O_P = O_P.astype(np.uint8)
    # cv2.imshow('O_P',O_P)
    # masked_edge_img=cv2.bitwise_and(erode,mask)   #与运算

    canny = cv2.Canny(cv_image,150,200)
    dilate = cv2.dilate(canny, kernel, iterations=10)
    erode = cv2.erode(dilate, kernel, iterations=5)
    cv2.imshow('canny',canny)

    # print(img.shape) 480*640
    # data =  canny[300, :]
    # print(type(O_P))
    return [x,y,z]


if __name__ == '__main__':
    rospy.init_node('robot_move')
    listener = tf.TransformListener()
    rate = rospy.Rate(100)
    start_time = time.time()
    vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    key = 0
    while not rospy.is_shutdown():
    # while True:
        try:
            (trans,rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print('(x, y, z) = (',round(trans[0],6)+2, ' , ', round(trans[1],6)+2, ' , ', round(trans[2],6) ,')')

        
        # (x,y) = design_path(runing_time)
        # x_error = x - round(trans[0],6)
        # y_error = y - round(trans[1],6)

        

        scan = robotsensor.get_velodyne()
        [roll, pitch, yaw] = robotsensor.get_odom()
        # [angle, dis] = robotsensor.get_min_distance_angle()

        img = robotsensor.get_image()
        [goal_x,goal_y,goal_z] = goal_point(img)


        # cv2.imshow("Image Window", cv_image)
        cv2.waitKey(1)

        (roll, pitch, yaw) = robotsensor.get_odom()

        #########---------------------- plan OK in class----------------------#########  
        cmd = robotmove.move_robot(0.8,0)
        vel_publisher.publish(cmd)

        end_time = time.time()
        runing_time = end_time - start_time
        print('time = ', runing_time)
        rate.sleep()


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
    
    # cmd = robotmove.move_robot(0,0)
    # vel_publisher.publish(cmd)

        