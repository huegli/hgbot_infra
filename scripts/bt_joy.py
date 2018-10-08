#!/usr/bin/env python

import pexpect

bt = pexpect.spawn('bluetoothctl')
bt.expect('#')
bt.sendline('devices')
bt.expect('#')
try:
    bt.expect('Dvice [A-F0-9:]+ Wireless')
except pexpect.exceptions.TIMEOUT:
    print("No wireless controller paired") 
print("Wireless controller found")
#print bt.before
#print bt.after

