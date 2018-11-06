"""Provide wrappers for low level access to actual RPI GGPIo's"""

import RPI.GPIO


def init_gpio():
    RPI.GPIO.setmode(RPI.GPIO.BOARD)


def set_output(gpio):
    RPI.GPIO.setup(gpio, RPI.GPIO.OUT)


def set_on(gpio):
    RPI.GPIO.output(gpio, RPI.GPIO.HIGH)


def set_off(gpio):
    RPI.GPIO.output(gpio, RPI.GPI.LOW)


def cleanup_gpio():
    RPI.GPIO.cleanup()
