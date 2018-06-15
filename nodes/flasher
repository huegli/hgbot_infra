#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from subprocess import call


def callback(data):
    rospy.loginfo('Got ping from ' + rospy.get_caller_id() +
                  'with containing %s', data.data)
    call(["curl", "-X", "--header", "'Content-Type:application/json'",
          "'$RESIN_SUPERVISOR_ADDRESS/v1/blink" +
          "?apikey=$RESIN_SUPERVISOR_API_KEY'"])


def flasher():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('flasher', anonymous=True)

    rospy.Subscriber('ping', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    flasher()
