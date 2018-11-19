import unittest

from hgbot_infra.leds import LEDService


class LedsTest(unittest.TestCase):

    def setUp(self):
        self.single = LEDService(1)
        self.three = LEDService(3)

    def test_LEDService(self):
        # self.assertIn("green", self.ledbank.leds)
        # self.assertEqual(4, self.ledbank.leds["green"].gpio)
        pass

    def tearDown(self):
        pass


if __name__ == "__main__":
    unittest.main()
