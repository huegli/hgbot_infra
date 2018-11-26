import unittest

from hgbot_infra.rover import Rover

class MscTest(unittest.TestCase):

    def setUp(self):
        self.rover = Rover()
    
    def test_forward(self):
        self.rover.set_all(0.75)
        self.assertEqual(0.75, self.rover.motor_speeds[0])
        self.assertEqual(0.75, self.rover.motor_speeds[1])

    def test_reverse(self):
        self.rover.set_all(-0.25)
        self.assertEqual(-0.25, self.rover.motor_speeds[0])
        self.assertEqual(-0.25, self.rover.motor_speeds[1])

    def test_turn(self):
        self.rover.set_motor(0, 0.5)
        self.rover.set_motor(1, -0.5)
        self.assertEqual(0.5, self.rover.get_motor(0))
        self.assertEqual(-0.5, self.rover.get_motor(1))
                