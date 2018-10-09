#!/usr/bin/env python

import pexpect
import re
import roslaunch

mac_re = re.compile('[A-F0-9:]+')

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
    exit(0)
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

package = 'joy'
executable = 'joy_node'
joy_node = roslaunch.core.Node(package, executable)

package = 'teleop_twist_joy'
executable = 'teleop_node'
tele_node = roslaunch.core.Node(package, executable)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

joy_process = launch.launch(joy_node)
tele_process = launch.launch(tele_node)
