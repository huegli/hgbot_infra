import unittest

from hgbot_infra.leds import GpioLEDBank


class LedsTest(unittest.TestCase):

    def setUp(self):
        my_leds = {
                "green": 4,
                "red": 12,
                "yellow": 15
                }
        self.ledbank = GpioLEDBank(my_leds)

    def test_gpioLEDBank_create(self):
        self.assertIn("green", self.ledbank.leds)
        self.assertEqual(4, self.ledbank.leds["green"].gpio)
        self.assertIn("red", self.ledbank.leds)
        self.assertEqual(12, self.ledbank.leds["red"].gpio)
        self.assertIn("yellow", self.ledbank.leds)
        self.assertEqual(15, self.ledbank.leds["yellow"].gpio)

    def test_gpioLEDBank_all_on(self):
        self.ledbank.all_on()
        for color, led in self.ledbank.leds.items():
            self.assertTrue(led.is_on)

    def test_gpioLEDBank_all_off(self):
        self.ledbank.all_on()
        self.ledbank.all_off()
        for color, led in self.ledbank.leds.items():
            self.assertFalse(led.is_on)

    def tearDown(self):
        pass


if __name__ == "__main__":
    unittest.main()
