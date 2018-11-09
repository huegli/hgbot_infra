import unittest

from hgbot_infra.leds import GpioLED

class LedsTest(unittest.TestCase):

    def setUp(self):
        self.led = GpioLED(4)

    def test_gpioLED_create(self):
        self.assertEqual(4, self.led.gpio)

    def test_gpioLED_on(self):
        self.led.led_on()
        self.assertTrue(self.led.is_on)

    def test_gpioLED_on_off(self):
        self.led.led_on()
        self.assertTrue(self.led.is_on)
        self.led.led_off()
        self.assertFalse(self.led.is_on)

    def test_gpioLED_on_toggle_off(self):
        self.led.led_on()
        self.assertTrue(self.led.is_on)
        self.led.led_blink()
        self.assertFalse(self.led.is_on)
        self.led.led_blink()
        self.assertTrue(self.led.is_on)
        self.led.led_off()
        self.assertFalse(self.led.is_on)

if __name__ == "__main__":
    unittest.main()
