"""Provide Classes to access LED's on HGBot Platforms"""

from __future__ import print_function

import os
import sys

import requests

try:
    import RPi.GPIO
except (ImportError, RuntimeError):
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


class GpioLEDBank(object):
    def __init__(self, my_leds):
        self.leds = {}
        for color, gpio in my_leds.items():
            self.leds[color] = GpioLED(gpio)

    def all_on(self):
        for color, led in self.leds.items():
            led.led_on()

    def all_off(self):
        for color, led in self.leds.items():
            led.led_off()


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
            return
          
        self.headers = {
            'Content-Type': 'application/json',
        }

        self.params = (
            ('apikey', api_key),
        )

        self.blink_url = supervisor_url + '/v1/blink'

    def blink(self):
        """Blink the Activity LED by sending a JSON request"""
        if self.blink_url:
            requests.post(self.blink_url,
                          headers=self.headers,
                          params=self.params)


class LEDService:
    """A service to blink LED's on a HGBot platform"""
    
    LED_OFF 	= 0
    LED_ON	= 1
    LED_BLINK	= 2
    LED_FAST	= 3
    
    ACTIVITY_ON_TIME	= 15
    LED_ON_TIME		= 2
    
    def __init__(self, num_leds=0):

        self.states = {}
        self.act_on_time = {}
        self.activity = ActivityLED()
        self.act_on_time["activity"] = 0
        self.states["activity"] = LEDService.LED_OFF
        
        if (num_leds == 1):
            self.ledbank = GpioLEDBank({
                "green": 4
            })
            self.states["green"] = LEDService.LED_OFF
        if (num_leds == 3):
            self.ledbank = GpioLEDBank({
                "green": 	4,
                "red":		12,
                "yellow":	15
            })
            self.states["green"] = LEDService.LED_OFF
            self.act_on_time["green"] = 0
            self.states["red"] = LEDService.LED_OFF
            self.act_on_time["red"] = 0
            self.states["yellow"] = LEDService.LED_OFF
            self.act_on_time["yellow"] = 0

    def callback(self, req):
        try:
        		(led, state) = req.cmd.split(':')
        except ValueError:
            return   # silently exit if not the correct format
        if led is "activity":
            if state is "blink":
                self.states["activity"] = LED_BLINK
                self.act_on_time = 15
            else:
                set.states["activity"] = LED_OFF
        if led in self.states:
            if state is "on":
                self.states[led] = LED_ON
            elif state is "blink":
                self.states[led] = LED_BLINK
            elif state is "fast":
                self.states[led] = LED_FAST
            else:
                self.states[led] = LED_OFF
                
    def process(self, duration=0.5):
    	for led, state in self.states.items():
            if state is LED_ON:
                self.ledbank.leds[led].led_on()
            elif state is LED_BLINK:
                if self.act_on_time[led] > 0:
                    self.act_on_time[led] = self.act_on_time[led] - duration
                else:
                    if led is "activity":
                        self.activity.blink()
                        self.act_on_time = ACTIVITY_ON_TIME
                    else:
                        self.ledbank.leds[led].led_blink()
                        self.act_on_time[led] = LED_ON_TIME
            elif state is LED_FAST:
            	  self.ledbank.leds[led].led_blink()
            else:
                self.ledbank.leds[led].led_off()
 
