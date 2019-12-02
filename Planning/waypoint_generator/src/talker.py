#!/usr/bin/env python
import rospy
from waypoint_generator.msg import point_list
from geometry_msgs.msg import Point


def talker():
    pub = rospy.Publisher('chatter', point_list)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10)  # 10hz
    msg = point_list()
    point1 = Point()
    point1.x = 0
    point1.y = 0
    point2 = Point()
    point2.x = 0
    point2.y = 1
    point3 = Point()
    point3.x = 1
    point3.y = 1
    point4 = Point()
    point4.x = 2
    point4.y = 2
    msg.points = [point1, point2, point3, point4]

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
