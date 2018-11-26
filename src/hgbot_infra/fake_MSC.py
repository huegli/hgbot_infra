"""Fake Motor Speed Control for use in CI environments w/o real hardware"""

class MotorSpeedControl(object):

    def __init__(self):
        self._fail_safe = False
        self.foundChip = True
        pass

    def Init(self):
        pass

    def SetCommsFailsafe(self, state):
        self._fail_safe = state

    def MotorsOff(self):
        pass

    def SetMotor1(self, speed):
        pass

    def SetMotor2(self, speed):
        pass



