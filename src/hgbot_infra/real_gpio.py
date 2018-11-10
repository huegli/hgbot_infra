"""Provide wrappers for low level access to actual RPI GGPIo's"""

import RPi.GPIO


def init_gpio():
    RPi.GPIO.setmode(RPi.GPIO.BCM)


def set_output(gpio):
    RPi.GPIO.setup(gpio, RPi.GPIO.OUT)


def set_on(gpio):
    RPi.GPIO.output(gpio, RPi.GPIO.HIGH)


def set_off(gpio):
    RPi.GPIO.output(gpio, RPi.GPIO.LOW)


def cleanup_gpio():
    RPi.GPIO.cleanup()
