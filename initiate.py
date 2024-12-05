#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('initiate')
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub = rospy.Publisher('initiate', String, queue_size=1)
        pub.publish('start')
        print('initiate')
        rate.sleep()