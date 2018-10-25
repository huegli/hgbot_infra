#!/usr/bin/env python

import os
import time

from hgbot_legacy import rover


if __name__ == "__main__":

    while not os.path.exists("/dev/input/js0"):
        time.sleep(10)

    if os.environ.get("HGBOT_LEGACY_FULL_I2C",0) == 1:
        rover(True)
    else:
        rover(False)
