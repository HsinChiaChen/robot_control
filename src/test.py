#!/usr/bin/env python

from robot_control_class import RobotControl

robotcontrol = RobotControl()

t = 1

while t > 0:

    # robotcontrol.move_straight_time("forward", 1, 3)
    # robotcontrol.stop_robot()
    # d = robotcontrol.get_laser(0)
    # print(d)
    scan = robotcontrol.get_laser_full()
    print(len(scan))
    print('start')
    # robotcontrol.turn("counter_clockwise", 1, 2)
    # robotcontrol.rotate(90)
    # robotcontrol.move_straight_time("forward", 1, 3)
    # robotcontrol.stop_robot()
    # print('straight_end')

    # s = robotcontrol.turn_and_move("clockwise",1, 1.7, 3)
    # robotcontrol.stop_robot()
    # print(s)

    s = robotcontrol.turn_and_move2angle(1,30)
    robotcontrol.stop_robot()
    print(s)

    # robotcontrol.move_straight_time("backward", 1, 3)
    # robotcontrol.stop_robot()
    # print('backward_end')



