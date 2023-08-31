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

    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            summit_connections = self.summit_vel_publisher.get_num_connections()
            if connections > 0 or summit_connections > 0:
                self.vel_publisher.publish(self.cmd)
                self.summit_vel_publisher.publish(self.cmd)
                #rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()


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
    # rospy.init_node('robot_move_node', anonymous=True)
    robotcontrol_object = RobotMove()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            robotcontrol_object.move_robot(1,0.5)
        except rospy.ROSInterruptException:
            pass

