#!/usr/bin/env python

import os
import pexpect
import re
import requests
import time

import rospy

import hgbot_infra.leds as leds
from hgbot_infra.srv import HGBCmd


class LEDService:
    """A service to blink LED's on a HGBot platform"""
    def __init__(self, num_leds):

        self.activity = leds.ActivityLED()
        if (num_leds == 1):
            self.red = leds.GpioLED(4)
        if (num_leds == 3):
            self.red = leds.GpioLED(6)
            self.green = leds.GpioLED(8)
            self.yellow = leds.GpioLED(10)


def bt_joy_server():
    rospy.init_node('bt_joy', log_level=rospy.DEBUG)

    rospy.logdebug("Creating BT Controller object")
    btc = BlueToothController()

    rospy.logdebug("Launch bt_joy service")
    rospy.Service('bt_joy_service', HGBCmd, btc.bt_joy_callback)

    if not btc.is_paired():
        rospy.logdebug("FAST-BLINK")
        btc.pair()

    d = rospy.Duration(20)  # wake up every 20 seconds to report status
    while not rospy.is_shutdown():
        if btc.is_scanning():
            rospy.logdebug("FAST-BLINK")
            blink_resin_LED()
        elif btc.is_connected():
            rospy.logdebug("SOLID")
        elif btc.is_paired():
            rospy.logdebug("BLINK")
        else:
            rospy.logdebug("OFF")
        rospy.sleep(d)


if __name__ == "__main__":
    bt_joy_server()
