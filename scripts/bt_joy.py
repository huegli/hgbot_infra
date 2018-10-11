#!/usr/bin/env python

import pexpect
import re

import roslaunch
import rospy


def pair_controller():
    "Set up BT to scan for and pair with a wireless controller"

    mac_re = re.compile('[A-F0-9:]+')

    print("Looking for existing wireless controller")
    bt = pexpect.spawn('bluetoothctl')
    bt.expect('#')
    bt.sendline('devices')
    bt.expect('#')

    try:
        bt.expect('Device [A-F0-9:]+ Wireless', timeout=10)
    except pexpect.exceptions.TIMEOUT:
        print("No wireless controller paired")
    else:
        print("Wireless controller found")
        return

    print("Attempting to pair ...")
    bt.sendline('agent on')
    bt.expect('#')
    bt.sendline('default-agent')
    bt.expect('#')
    bt.sendline('scan on')

    try:
        bt.expect('[A-F0-9:]+ Wireless', timeout=300)
    except pexpect.exceptions.TIMEOUT:
        print("No pairing attempt detected, quiting")
        exit(1)

    m = mac_re.match(bt.after)
    if not m:
        print("No valid MAC address detected")
        exit(1)
    mac = m.group(0)

    print("Found {}".format(mac))
    bt.send('pair ')
    bt.sendline(mac)
    bt.expect('#')
    bt.send('trust ')
    bt.sendline(mac)
    bt.expect('#')
    print("Paired & trusted wireless controller")
    bt.sendline('exit')


def launch_nodes():
    "Call a ROS launch file on successful controller pairing"

    rospy.init_node('bt_joy', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(
            uuid,
            ["/ros/hgbot_ws/src/hgbot_infra/launch/bt_joy.launch"])
    launch.start()

if __name__ == "__main__":
    pair_controller()
    launch_nodes()
