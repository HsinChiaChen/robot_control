#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
import time
import math


class RobotMove():

    def __init__(self):
        self.cmd = Twist()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.ctrl_c = False

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    # 讓正在運行中的 waffle 停下
    def stop_robot(self):
        #rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0

    # self.cmd.linear.x 代表了 waffle 輪軸的轉速，大於0表示前進；
    # self.cmd.angular.z 則代表 waffle 本身旋轉的角速度，大於0為逆時針轉。

    def move_robot(self,linear_velocity,angular_velocity):
         # 設定線速度和角速度
        self.cmd.linear.x = linear_velocity
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0

        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z  = angular_velocity # rad/s

        # print(self.cmd.linear.x)
        # print(self.cmd.angular.z)

        return self.cmd


if __name__ == '__main__':
    #rospy.init_node('robot_control_node', anonymous=True)
    robotcontrol_object = RobotMove()
    try:
        robotcontrol_object.move_robot(1,0.5)

    except rospy.ROSInterruptException:
        pass