from unittest import TestCase
from unittest.mock import patch

import hgbot_infra.leds as leds


class TestLEDs(TestCase):

    def test_bare():
        assert True


    @patch('hgbot_infa.leds.RPI.GPIO.setmode', return_value = True)
    def test_gpioLED_create():
        led = leds.GpioLED(4)
        assert led.gpio == 4
