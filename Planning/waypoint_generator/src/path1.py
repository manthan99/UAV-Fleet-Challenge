#!/usr/bin/env python
import AHP
import GOW
import math
import rospy
from geometry_msgs.msg import Point
from waypoint_generator.msg import point_list
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from geopy.distance import geodesic


pos_list = []

global pub1
global current_position
current_position = Point()
flag = Twist()
flag.linear.y = 0
GPS_flag = 0


def frameCallback(data):
    global pos_list
    pos_list = data.points
    if flag.linear.y > 0.5 and GPS_flag == 1:
        path()
    # if flag.linear.y < 0.5:
    #     print("ROI not published yet")
    # if GPS_flag != 1:
    #     print("GPS not received")
#	print (pos_list)


def current_callback(data):
    global GPS_flag
    global current_position
    current_position.x = data.latitude
    current_position.y = data.longitude
    GPS_flag = 1


def flag_callback(data):
    global flag
    flag = data


def listener():
    global pub1
    rospy.init_node('path_drone1', anonymous=True)
    frame_sub = rospy.Subscriber('/drone1/ROI_flow_initial', point_list, frameCallback)
    current_pos = rospy.Subscriber("/drone1/mavros/global_position/global", NavSatFix, current_callback)
    flags = rospy.Subscriber("/drone1/flags", Twist, flag_callback)
    pub1 = rospy.Publisher('/drone1/ROI_flow', point_list, queue_size=5)
    rospy.spin()


def path():
    global pub1
    global pos_list
    current_pos = current_position
    # pos_list = pos_list
    edge = []

    print(len(pos_list))
    for i in range(len(pos_list)):
        for j in range(i + 1, len(pos_list)):
            edge.append(AHP.Edge(i, j, geodesic((pos_list[i].x, pos_list[i].y), (pos_list[j].x, pos_list[j].y)).m))

    for e in edge:
        pass
        # print(e.u, "<->", e.v, ", W:", e.w)

    print(len(edge))
    path, w = AHP.AHP(edge, len(pos_list))
    # print("Final result", path, "TW:", w)

    order_waypt = GOW.generateOrderedWaypoints(pos_list, current_pos, path)
    pub1.publish(order_waypt)
    # print(order_waypt)


if __name__ == '__main__':
    listener()
