#!/usr/bin/env python
import rospy
from robot_control_class import RobotControl

robotcontrol = RobotControl()

while not rospy.is_shutdown():

    ####################   method 1   #######################
    # robotcontrol.move_straight_time("forward", 1, 2)
    # robotcontrol.stop_robot()
    # d = robotcontrol.get_laser(0)
    # print(d)
    # robotcontrol.turn("counter_clockwise", 1, 2)
    # robotcontrol.stop_robot()

    ####################   method 2   #######################
    robotcontrol.move_robot(0.5, 0.5)
    d = robotcontrol.get_laser_full()
    print(d)
    robotcontrol.stop_robot()

    # robotcontrol.rotate(90)
    # robotcontrol.move_straight_time("forward", 1, 3)
    # robotcontrol.stop_robot()
    # print('straight_end')

    # s = robotcontrol.turn_and_move("clockwise",1, 1.7, 3)
    # robotcontrol.stop_robot()
    # print(s)

    # scan = robotcontrol.get_laser_full()
    # print(len(scan))
    # print('start')
    # s = robotcontrol.turn_and_move2angle(1,30)
    # robotcontrol.stop_robot()
    # print(s)

    # robotcontrol.move_straight_time("backward", 1, 3)
    # robotcontrol.stop_robot()
    # print('backward_end')



