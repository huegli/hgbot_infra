from hgbot_infra.leds import GpioLED


def test_bare():
    assert True


def test_gpioLED_create():
    led = GpioLED(4)
    assert led.gpio == 4
