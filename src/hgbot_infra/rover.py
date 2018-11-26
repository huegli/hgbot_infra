"""Python API to controlling a rover-type mobile robot"""

from __future__ import print_function

import sys

try:
    import smbus
except (ImportError, RuntimeError):
    from .fake_MSC import MotorSpeedControl
else:
  	from PicoBorgRev import PicoBorgRev as MotorSpeedControl


class Rover(object):

    def __init__(self, num_msc = 2):
      	self.motor_speeds = []
        for m in range(num_msc):
            self.motor_speeds[m] = 0.0
        self.MAXPOWER = 11.5
      	self.MSC = MotorSpeedControl()
        
        if not self.MSC.foundChip:
          print("Can't commumincate with Motor Speed Control")
          sys.exit()
          
        self.MSC.Init()
        self.MSC.MotorsOff()
        self.MSC.SetCommsFailsafe(True)
        
    def _limit_speed(speed):
      	if (speed > 1.0):
            return 1.0
        if (speed < -1.0):
            return -1.0
        else:
            return speed

    def set_motor(self, motor, speed):
        limited = _limit_speed(speed)
        if motor == 0:
        		MSC.SetMotor1(limited * self.MAXPOWER)
        if motor == 1:
        		MSC.SetMotor2(-limited * self.MAXPOWER)
        

    def get_motor(self, motor):
        if (motor >= 0) and (motor < len(self.motor_speeds)):
					  return self.motor_speeds[motor]
        else:
            return 0.0        

    def set_all(self, speed):
        limited = _limit_speed(speed)
        for m in range(len(self.motor_speeds)):
          self.motor_speeds[m] = limited
        self.MSC.SetMotor1(limited * self.MAXPOWER)
        self.MSC.SetMotor2(-limited * self.MAXPOWER)

    def all_off(self):
        self.MSC.MotorsOff()
        