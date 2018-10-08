#!/usr/bin/env python

import pexpect

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
#print bt.before
#print bt.after

