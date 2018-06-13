#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def pinger():
    pub = rospy.Publisher('ping', String, queue_size=10)
    rospy.init_node('pinger', anonymous=True)
    rate = rospy.Rate(0.1) # 10 seconds
    while not rospy.is_shutdown():
        ping_str = "ping"
        rospy.loginfo(ping_str)
        pub.publish(ping_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        pinger()
    except rospy.ROSInterruptException:
        pass
