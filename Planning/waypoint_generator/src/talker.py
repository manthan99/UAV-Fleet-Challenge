#!/usr/bin/env python
import rospy
from waypoint_generator.msg import point_list
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import tf


def talker():
    pub = rospy.Publisher('/some_topic', point_list, queue_size=10)
    gps = rospy.Publisher('/drone1/mavros/global_position/global', NavSatFix, queue_size=10)
    height = rospy.Publisher('/drone1/mavros/local_position/pose',  PoseStamped, queue_size=10)
    compass_hdg = rospy.Publisher('/drone1/mavros/global_position/compass_hdg', Float64, queue_size=10)

    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10)  # 10hz

    # frame points
    msg = point_list()
    point1 = Point()  # bottom right
    point1.x = 360 + 100
    point1.y = 640 + 100
    point2 = Point()  # top left
    point2.x = 360 - 100
    point2.y = 640 - 100
    point3 = Point()  # top right
    point3.x = 360 - 100
    point3.y = 640 + 100
    point4 = Point()  # bottom left
    point4.x = 360 + 100
    point4.y = 640 - 100
    point5 = Point()  # 0
    point5.x = 360 + 110
    point5.y = 640 + 110
    point6 = Point()  # pi
    point6.x = 360
    point6.y = 640
    msg.points = [point1, point2, point3, point4, point5, point6]

    # GPS
    navsat = NavSatFix()
    navsat.latitude = 0.00
    navsat.longitude = 0.00
    navsat.altitude = 30.00

    # heading_angle
    heading_angle = Float64(0.0)

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = 10
    pose.pose.position.y = 10
    pose.pose.position.z = 10.0                 # only this used in subscriber

    quaternion = tf.transformations.quaternion_from_euler(0, 0, 10)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    while not rospy.is_shutdown():
        # rospy.loginfo(msg)
        gps.publish(navsat)
        compass_hdg.publish(heading_angle)
        height.publish(pose)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
