#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import math


class RobotControl():

    def __init__(self):
        rospy.init_node('robot_control_node', anonymous=True)
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.summit_vel_publisher = rospy.Publisher('/summit_xl_control/cmd_vel', Twist, queue_size=1)
        # self.laser_subscriber = rospy.Subscriber(
        #     '/kobuki/laser/scan', LaserScan, self.laser_callback)
        # self.summit_laser_subscriber = rospy.Subscriber(
        #     '/hokuyo_base/scan', LaserScan, self.summit_laser_callback)
        self.laser_subscriber = rospy.Subscriber(
            '/scan', LaserScan, self.laser_callback)
        self.summit_laser_subscriber = rospy.Subscriber(
            '/scan', LaserScan, self.summit_laser_callback)
        # 須修改成 '/scan'，這是由於教學課程當中的機器人與 waffle 不同，
        # 所以在 .xacro 當中對 Topic 的命名有所差異
        
        self.odom_sub = rospy.Subscriber ('/odom', Odometry, self.odom_callback)
        self.cmd = Twist()
        self.laser_msg = LaserScan()
        self.summit_laser_msg = LaserScan()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.ctrl_c = False
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdownhook)

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

    def laser_callback(self, msg):
        self.laser_msg = msg

    def summit_laser_callback(self, msg):
        self.summit_laser_msg = msg

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)
        
    # 使用雷射獲取距離資訊
    def get_laser(self, pos):
        time.sleep(1)
        return self.laser_msg.ranges[pos]
    # 若同時需要獲得多點之距離資訊時，建議使用函式 scan = get_laser_full(self)，其資訊會儲存成一個名為 scan 的 list，
    # 若想得到某角度之資訊可使用 scan[index] 來取得。
    
    def get_laser_summit(self, pos):
        time.sleep(1)
        return self.summit_laser_msg.ranges[pos]

    def get_front_laser(self):
        time.sleep(1)
        return self.laser_msg.ranges[360]

    def get_laser_full(self):
        time.sleep(1)
        return self.laser_msg.ranges

    # 讓正在運行中的 waffle 停下
    def stop_robot(self):
        #rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()
    # self.cmd.linear.x 代表了 waffle 輪軸的轉速，大於0表示前進；
    # self.cmd.angular.z 則代表 waffle 本身旋轉的角速度，大於0為逆時針轉。

    # 使 waffle 直行數秒
    def move_straight(self):

        # Initilize velocities
        self.cmd.linear.x = 0.5
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        # Publish the velocity
        self.publish_once_in_cmd_vel()

    def move_straight_time(self, motion, speed, time):

        # Initilize velocities
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        if motion == "forward":
            self.cmd.linear.x = speed
        elif motion == "backward":
            self.cmd.linear.x = - speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            self.summit_vel_publisher.publish(self.cmd)
            i += 0.1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Moved robot " + motion + " for " + str(time) + " seconds"
        return s


    def turn(self, clockwise, speed, time):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0

        if clockwise == "clockwise":
            self.cmd.angular.z = -speed
        else:
            self.cmd.angular.z = speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)

        while (i <= time):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            self.summit_vel_publisher.publish(self.cmd)
            i += 0.1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Turned robot " + clockwise + " for " + str(time) + " seconds"
        return s

    def rotate(self, degrees):
        time.sleep(1)
        target_rad = (degrees * math.pi/180) + self.yaw

        if target_rad < (- math.pi):
            target_rad = target_rad + (2 * math.pi)

        if target_rad > (math.pi):
            target_rad = target_rad - (2 * math.pi)

        while abs(target_rad - self.yaw) > 0.01:
            self.cmd.angular.z = 0.5 * (target_rad - self.yaw)
            self.vel_publisher.publish(self.cmd)
            self.rate.sleep()

        self.stop_robot()

    
    def move_robot(self,linear_velocity,angular_velocity):
         # 設定線速度和角速度
        self.cmd.linear.x = linear_velocity
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0

        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z  = angular_velocity # rad/s

        # Publish the velocity
        self.publish_once_in_cmd_vel()


if __name__ == '__main__':
    rospy.init_node('robot_control_node', anonymous=True)
    robotcontrol_object = RobotControl()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            robotcontrol_object.move_robot(1,1.5)
        except rospy.ROSInterruptException:
            pass
