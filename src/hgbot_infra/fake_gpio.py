"""Provide wrappers for low level access to actual RPI GGPIo's"""

from __future__ import print_function


def init_gpio():
    pass


def set_output(gpio):
    pass


def set_on(gpio):
    print("ON<{gpio}>")


def set_off(gpio):
    print("OFF<{gpio}>")


def cleanup_gpio():
    pass
