#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
import time
import math


class RobotSensor():

    def __init__(self):
        self.laser_subscriber = rospy.Subscriber(
            '/scan', LaserScan, self.laser_callback)
        self.summit_laser_subscriber = rospy.Subscriber(
            '/scan', LaserScan, self.summit_laser_callback)
        # 須修改成 '/scan'，這是由於教學課程當中的機器人與 waffle 不同，
        # 所以在 .xacro 當中對 Topic 的命名有所差異

        self.d435_subscriber = rospy.Subscriber(
            '/camera/color/image_raw', Image, self.image_callback)
        
        self.velodyne_subscriber = rospy.Subscriber(
            '/velodyne_points', PointCloud2, self.PointCloud2_callback)
        
        self.odom_sub = rospy.Subscriber ('/odom', Odometry, self.odom_callback)
        self.cmd = Twist()
        self.laser_msg = LaserScan()
        self.summit_laser_msg = LaserScan()
        self.image_msg =  Image()
        self.pointcloud2_msg =  PointCloud2()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True
    
    ############################## laser #############################
    def laser_callback(self, msg):
        self.laser_msg = msg

    def summit_laser_callback(self, msg):
        self.summit_laser_msg = msg

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
        # time.sleep(1)
        return self.laser_msg.ranges
    
    def get_min_distance_angle(self):
        ranges = self.laser_msg.ranges
        min_distance = min(ranges)
        total_number = len(ranges)
        min_distance_angle = ranges.index(min_distance)
        return [min_distance_angle, min_distance]
    
    def PointCloud2_callback(self, msg):
        # print("Cloud: width = {} height = {}".format(msg.width, msg.height))
        # for pt in msg.points:
        #     print("\t({}, {}, {})".format(pt.x, pt.y, pt.z))
        self.pointcloud2_msg = msg

    def get_velodyne(self):
        # time.sleep(1)
        return self.pointcloud2_msg
    

    ############################# vision #############################
    def image_callback(self, msg):
        self.image_msg = msg

    def get_image(self):
        # time.sleep(1)
        return self.image_msg


    ############################# odom #############################
    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)

    def get_odom(self):
        time.sleep(1)
        return (self.roll, self.pitch, self.yaw)
        
    
    
    
    
    
    
    
    
