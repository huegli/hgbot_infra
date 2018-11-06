"""Provide Classes to access LED's on HGBot Platforms"""

from __future__ import print_function

import os
import sys

import requests

try:
    import RPI.GPIO
except ImportError:
    from .fake_gpio import *
else:
    from .real_gpio import *
finally:
    init_gpio()


class GpioLED(object):
    """A base class for GPIO driven LED's"""
    def __init__(self, gpio):
        self.gpio = gpio
        self.is_on = False
        set_output(gpio)

    def led_on(self):
        """Turn on LED by driving GPIO high"""
        set_on(self.gpio)
        self.is_on = True

    def led_off(self):
        """Turn off LED by driving GPIO low"""
        set_off(self.gpio)
        self.is_on = False

    def led_blink(self):
        """Blin LED by toggling the GPIO"""
        if self.is_on:
            self.led_off()
        else:
            self.led_on()


class ActivityLED(object):
    """A base class for activity LED driven through resio.io's
       Supervisor API"""
    # pylint: disable=too-few-public-methods

    def __init__(self):
        try:
            supervisor_url = os.environ["RESIN_SUPERVISOR_ADDRESS"]
            api_key = os.environ["RESIN_SUPERVISOR_API_KEY"]
        except KeyError:
            print("Can't find RESIN environment variables,"
                  " run from resin.io shell")
            sys.exit(1)

        self.headers = {
            'Content-Type': 'application/json',
        }

        self.params = (
            ('apikey', api_key),
        )

        self.blink_url = supervisor_url + '/v1/blink'

    def blink(self):
        """Blink the Activity LED by sending a JSON request"""
        requests.post(self.blink_url,
                      headers=self.headers,
                      params=self.params)
