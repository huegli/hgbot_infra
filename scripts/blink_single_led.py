#!/usr/bin/env python

from time import sleep

from hgbot_infra.leds import GpioLED

led = GpioLED(4)

led.led_on()
sleep(2)
led.led_off()
sleep(2)

for i in range(10):
    led.led_blink()
    sleep(0.2)
