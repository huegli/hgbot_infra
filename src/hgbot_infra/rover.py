"""Python API to controlling a rover-type mobile robot"""

class Rover(object):

    def __init__(self, wheels = 4):
        pass

    def _limit_speed(self, speed):
        return 0

    def set_motor(self, motor, speed):
        pass

    def get_motor(self, motor):
        return 0

    def set_all(self, speed):
        pass

    def all_off(self):
        pass
        