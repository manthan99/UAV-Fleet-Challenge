#!/usr/bin/env python
import AHP
import GOW
import math
import rospy
from geometry_msgs.msg import Point
from waypoint_generator.msg import point_list


pos_list = []


def posCallback(data):
    global pos_list
    pos_list = data.points
    path()
#	print (pos_list)


def listener():
    rospy.init_node('path', anonymous=True)
    pos_sub = rospy.Subscriber('chatter', point_list, posCallback)
    rospy.spin()


def path():
    global pos_list
    current_pos = pos_list[0]
    pos_list = pos_list[1:]
    edge = []

    for i in range(len(pos_list)):
        for j in range(i + 1, len(pos_list)):
            edge.append(AHP.Edge(i, j, math.sqrt((pos_list[i].x - pos_list[j].x)**2 + (pos_list[i].y - pos_list[j].y)**2)))

    for e in edge:
        print(e.u, "<->", e.v, ", W:", e.w)

    path, w = AHP.AHP(edge, len(pos_list))
    print("Final result", path, "TW:", w)

    order_waypt = GOW.generateOrderedWaypoints(pos_list, current_pos, path)
    print(order_waypt)


if __name__ == '__main__':
    listener()
